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

    def enter(self):
        print("ENTER APF STATE")

        client = self._agent.client

        goal = Position()
        p = self._agent.path[self._agent.astar_curr_point].point()
        goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height

        cur_pose = client.getPose()
        start = (cur_pose.pos.x_m, cur_pose.pos.y_m)
        goal_tupple = (goal.x_m, goal.y_m)
        client.stop()
        self._agent._apf_path_planner = APFPathPlanner(start, goal_tupple)
        curr_position = start
        self._agent._lidar_points_counter.start()

        reached_goal = False

        is_local_minima = False
        num_steps = 0

        pos_list = list()

        while not self._agent._apf_path_planner.reached_goal(curr_position):
            next_position = self._agent._apf_path_planner.next_step(curr_position, self._agent._lidar_points)
            num_steps += 1
            pos_list.append(Point(*next_position))

            ################ LOCAL MINIMA DETECTION ###########################################
            if num_steps == 10:
                is_local_minima = self._agent.is_local_minima(pos_list)
                pos_list = list()
                num_steps = 0
                if is_local_minima:
                    print("Local Minima")
            ###################################################################################

            self._agent._clear_lidar_points()
            client.flyToPosition(next_position[0], next_position[1], config.height,
                                              config.apf_velocity)
            # print("fly to position")
            # print(next_position[0], next_position[1])
            self._agent._collect_lidar_points()
            while not self._agent._apf_path_planner.reached_location(curr_position, next_position):
                curr_position = client.getPose().pos.x_m, client.getPose().pos.y_m
                self._agent._collect_lidar_points()

                if num_steps == 10:
                    is_local_minima = self._agent.is_local_minima(pos_list)
                    num_steps = 0
                    if is_local_minima:
                        print("Local Minima")

                if self._agent._apf_path_planner.reached_goal(curr_position):
                    print("APF REACHED LOCAL GOAL")
                    return AlgoStateEnum.ASTAR
            curr_position = next_position
        return AlgoStateEnum.ASTAR

    def exit(self):
        print("EXIT APF STATE")
