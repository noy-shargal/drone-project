import time

from shapely.geometry import Point
from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum
from Config import config
from DroneTypes import Position
from MyDroneClient import MyDroneClient
from apf.APFPathPlanner import APFPathPlanner


class APFState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.APF)
        self._agent = agent

    def _afine_point(self, x1,y1, x2,y2, ratio=0.85):
        x = x1 * (1.0-ratio) + x2 * ratio
        y = y1 * (1.0-ratio) + y2 * ratio
        return x, y

    def enter(self):
        print("ENTER APF STATE")

        if self._agent.astar_curr_point >= len(self._agent.path):
            return AlgoStateEnum.END
        client = self._agent.client

        goal = Position()
        p = self._agent.path[self._agent.astar_curr_point].point()
        goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
        print("1APF GOAL :" + str(goal) + 'INDEX :' + str(self._agent.astar_curr_point))
        cur_pose = client.getPose()
        start = (cur_pose.pos.x_m, cur_pose.pos.y_m)
        goal_tupple = (goal.x_m, goal.y_m)
        #client.stop()
        self._agent._apf_path_planner = APFPathPlanner(start, goal_tupple)
        curr_position = start
        self._agent._lidar_points_counter.start()
        num_steps = 0
        pos_list = list()
        velocity =  config.apf_velocity
        while not self._agent._apf_path_planner.reached_goal(curr_position):
            next_position = self._agent._apf_path_planner.next_step(curr_position, self._agent._lidar_points)
            num_steps += 1
            pos_list.append(Point(*next_position))

            ################ LOCAL MINIMA DETECTION ###########################################
            if num_steps == 15:
                is_local_minima = self._agent.is_local_minima(pos_list)
                pos_list = list()
                num_steps = 0
                if is_local_minima:
                    print("Local Minima")
                    return AlgoStateEnum.LOCAL_MINIMA
            ###################################################################################
            x = next_position[0]
            y = next_position[1]

            ############# WALL AHEAD ##########################################################
            # self._agent._clear_lidar_points()
            # is_wall_ahead, point = self._agent.is_wall_ahead()
            # x = next_position[0]
            # y = next_position[1]
            # if is_wall_ahead:
            #     print("WALL AHEAD !!! ")
            #
            #     velocity = config.apf_velocity - 2
            #     assert point is not None
            #     x, y = self._afine_point(client.getPose().pos.x_m, client.getPose().pos.y_m, point.x, point.y)
            #     print("AVOIDING WALL AHEAD -> "+str((x,y)))
            #     next_position = (x,y)
            #     # curr_position = client.getPose().pos.x_m, client.getPose().pos.y_m
            #     # client.flyToPosition(x, y, config.height, velocity)
            #     # while not self._agent._apf_path_planner.reached_location(curr_position, (x,y)):
            #     #     curr_position = client.getPose().pos.x_m, client.getPose().pos.y_m
            # else:
            #     velocity = config.apf_velocity
            ######################################################################################

            client.flyToPosition(x,y , config.height,velocity)
            # print("fly to position")
            # print(next_position[0], next_position[1])
            self._agent._collect_lidar_points()
            while not self._agent._apf_path_planner.reached_location(curr_position, next_position):
                curr_position = client.getPose().pos.x_m, client.getPose().pos.y_m
                self._agent._collect_lidar_points()

                if num_steps == 15:
                    is_local_minima = self._agent.is_local_minima(pos_list)
                    pos_list = list()
                    num_steps = 0
                    if is_local_minima:
                        print("Local Minima")
                point = Point(*curr_position)
                self._agent.add_path_point(point)
                if self._agent._apf_path_planner.reached_goal(curr_position):
                    print("APF REACHED LOCAL GOAL")
                    print("2APF GOAL :" + str(goal) + 'INDEX :' + str(self._agent.astar_curr_point))

                    self._agent.astar_curr_point +=1
                    return AlgoStateEnum.TRANSISTION
            curr_position = next_position
        print("3APF GOAL :" + str(goal) + 'INDEX :' + str(self._agent.astar_curr_point))
        self._agent.astar_curr_point += 1
        return AlgoStateEnum.TRANSISTION

    def exit(self):
        print("EXIT APF STATE")
