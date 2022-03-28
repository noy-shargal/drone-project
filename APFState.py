import time

from shapely.geometry import Point
from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum
from Config import config
from DroneTypes import Position
from MyDroneClient import MyDroneClient
from apf.APFPathPlanner import APFPathPlanner
from vector import Vector


class APFState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.APF)
        self._agent = agent

    def _afine_point(self, x1,y1, x2,y2, ratio=0.85):
        x = x1 * (1.0-ratio) + x2 * ratio
        y = y1 * (1.0-ratio) + y2 * ratio
        return x, y

    def _rotate_to_face_target_and_scan(self,current_position, target, step=5):
        speed = 1.25
        m_line = Vector(current_position, target)
        pos = self._agent.client.getPose().pos
        current_position = Point(pos.x_m, pos.y_m)
        norm = m_line.get_norm()

        next_x = current_position.x - m_line._x / norm * step
        next_y = current_position.y - m_line._y / norm * step

        self._agent.client.flyToPosition(next_x, next_y, config.height, speed)
        time.sleep(step / speed)
        #self._agent.client.flyToPosition(current_position.x, current_position.y, config.height, speed / 16)
        self._agent.client.flyToPosition(next_x +  m_line._x / norm * 0.5*step, next_y/ norm * 0.5*step, config.height, speed / 16)
        time.sleep(2)
        # #full_lidar_scan, world_cords_dict = self._agent.client.full_lidar_scan_v2(1.4)
        # pos = self._agent.client.getPose().pos
        #
        # time.sleep(0.1)

        return

    def enter(self):
        print("ENTER APF STATE")

        if self._agent.astar_curr_point >= len(self._agent.path):
            return AlgoStateEnum.END
        client = self._agent.client

        goal = Position()
        #p = self._agent.path[self._agent.astar_curr_point].point()
        p = self._agent.path[-1].point()
        goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
        print("1APF GOAL :" + str(goal) + 'INDEX :' + str(self._agent.astar_curr_point))
        cur_pose = client.getPose()
        start = (cur_pose.pos.x_m, cur_pose.pos.y_m)
        goal_tupple = (goal.x_m, goal.y_m)
        #client.stop()
        #self._agent._apf_path_planner = APFPathPlanner(start, goal_tupple)
        curr_position = start
        self._agent._lidar_points_counter.start()
        num_steps = 0
        pos_list = list()
        velocity =  config.apf_velocity
        virtual_goal = False
        virtual_goal_steps = 0
        use_repulsion = True
        no_repulsion_step = 0
        while not self._agent._apf_path_planner.reached_goal(curr_position):
            if use_repulsion == True:
                next_position = self._agent._apf_path_planner.next_step(curr_position, self._agent._lidar_points)
            else:
                next_position = self._agent._apf_path_planner.next_step_no_repulsion(curr_position, self._agent._lidar_points)
            num_steps += 1
            pos_list.append(Point(*next_position))

            ################ LOCAL MINIMA DETECTION AND RECOVERY###########################################
            if num_steps == 10:
                is_local_minima = self._agent.is_local_minima(pos_list)
                pos_list = list()
                print("current position :" + str(curr_position))
                curr_point = Point(*curr_position)
                goal_point = Point(*self._agent._apf_path_planner._real_goal)
                dis = curr_point.distance(goal_point)

                print ("Distance to the target = "+ str(dis ))

                num_steps = 0
                if no_repulsion_step == 2:
                    no_repulsion_step = 0
                    use_repulsion = True

                if is_local_minima:
                    print("Local Minima")
                    client.stop()
                    #self._rotate_to_face_target_and_scan(curr_point, goal_point)
                    time.sleep(3)

                    if not self._agent.is_wall_ahead(30):
                        use_repulsion = False
                        no_repulsion_step += 1
                    else:
                        new_goal = self._agent.move_goal_on_dronee_y_axis()
                        print ("changing to virtual target "+ str(new_goal))

                        self._agent._apf_path_planner.set_goal(new_goal)
                        virtual_goal = True

                    #return AlgoStateEnum.LOCAL_MINIMA

            if virtual_goal:
                virtual_goal_steps +=1

            if  virtual_goal_steps == config.virtual_goal_steps:
                self._agent._apf_path_planner.set_goal(goal_tupple)
                print("changing back to original  target " + str(goal_tupple))
                virtual_goal = False
                virtual_goal_steps = 0

            ###################################################################################
            x = next_position[0]
            y = next_position[1]



            client.flyToPosition(x,y , config.height,velocity)
            # print("fly to position")
            # print(next_position[0], next_position[1])

            self._agent._collect_lidar_points()
            while not self._agent._apf_path_planner.reached_location(curr_position, next_position):
                curr_position = client.getPose().pos.x_m, client.getPose().pos.y_m
                point = Point(*curr_position)
                self._agent.add_path_point(point)

                self._agent._collect_lidar_points()

            if self._agent._apf_path_planner.reached_goal(curr_position):
                    print("APF REACHED LOCAL GOAL")
                    print("2APF GOAL :" + str(goal) + 'INDEX :' + str(self._agent.astar_curr_point))

                    self._agent.astar_curr_point +=1
                    #return AlgoStateEnum.TRANSISTION
                    return AlgoStateEnum.END
            curr_position = next_position
        print("3APF GOAL :" + str(goal) + 'INDEX :' + str(self._agent.astar_curr_point))
        self._agent.astar_curr_point += 1
        return AlgoStateEnum.TRANSISTION

    def exit(self):
        print("EXIT APF STATE")
