import math
import random
import time
from enum import Enum, unique

from shapely.geometry import Point

from AStarState import AStarState
from AlgoStateInterface import AlgoStateEnum
from DroneTypes import Position
from MyDroneClient import MyDroneClient
from ASTARPathPlanner import ASTARPathPlanner
from Config import config
from apf.APFPathPlanner import APFPathPlanner
from apf.Countdowner import Countdowner
from utils import getPointInRealWorldCoords
from apf.config import current_config as apf_config




class SmartAgent_v1:

    def __init__(self):


        self._apf_path_planner = None
        self._astar_path_planner = ASTARPathPlanner()
        self.astar_curr_point = 0
        self.client = MyDroneClient()
        self.path = self._astar_path_planner.get_path()
        self.obs = self._astar_path_planner.get_obstacles_object()
        self._lidar_points_counter = Countdowner(5.0)
        self._lidar_points = list()

    def connect_and_spawn(self):
        self.client.reset()
        print("Connecting.....")
        self.client.connect()
        time.sleep(2)
        self.client.setAtPosition(config.source.x, config.source.y, config.height)
        time.sleep(2)
        print(self.client.isConnected())
        time.sleep(2)

    def reached_goal_2D(self, curr_pos: Position, goal: Position):
        diff_x = curr_pos.x_m - goal.x_m
        diff_y = curr_pos.y_m - goal.y_m
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        if dist < 5.0:
            return True
        return False

    def point_reached_goal_2D(self, curr_pos: Point, goal: Point):
        diff_x = curr_pos.x - goal.x
        diff_y = curr_pos.y - goal.y
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        if dist < 5.0:
            return True
        return False

    def position_to_point(self, pos: Position):
        return Point(pos.x_m, pos.y_m)


    def fly_to_destination(self):

        state = AStarState(self)
        print("Init position " + str([config.source.x, config.source.y, config.height]))
        self.astar_curr_point = 1
        while state.state_enum() != AlgoStateEnum.END:
            state = state.enter()

        print("Reached Goal !!! ")


        prev_point_num = 0
        point_num = 1
        need_fly_command = True
        real_path = list()
        client = self.client
        goal = Position()

        while True:

            lidar_data = client.full_lidar_scan(0.0, 0.04, True)
            p = self.path[point_num].point()
            goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height


            sensing_obstacle, points_list, pose = client.senseObstacle()

            if sensing_obstacle:
                point = Point(points_list[0], points_list[1])
                world_point = getPointInRealWorldCoords(point.x, point.y, pose)
                if not self.obs.is_point_in_obstacles_map(Point(*world_point)):  # new obstacle

                    print("APF MODE")
                    cur_pos = client.getPose()
                    client.flyToPosition(cur_pos.pos.x_m, cur_pos.pos.y_m, cur_pos.pos.z_m, 0.1)
                    tuple_goal = (goal.x_m, goal.y_m)
                    is_new_unkmown_obstacle = True
                    while is_new_unkmown_obstacle:
                        self.apf_fly_to_destination(tuple_goal)
                        point_num += 1
                        p = self.path[point_num].point()
                        goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
                        tuple_goal = (goal.x_m, goal.y_m)
                        is_new_unkmown_obstacle = client.isNewObstaclefullSenseObstacle(self.obs)



                    print("ASTAR MODE")
                    point_num += 1
                    need_fly_command = True


            if point_num >= len(self.path) + 1:
                break  # finish

            if need_fly_command:
                client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.astar_velocity)
                need_fly_command = False
                print("Flying to point number: " + str(point_num) + str([goal.x_m, goal.y_m, goal.z_m]))

            if self.reached_goal_2D(client.getPose().pos, goal):
                print("Reached goal number : " + str(point_num))
                prev_point_num = point_num
                point_num += 1
                need_fly_command = True
                pos = client.getPose().pos

                if point_num == len(self.path):
                    print("Reached destination at (" + str(client.getPose().pos.x_m) + ", " + str(
                        client.getPose().pos.y_m) + ") ")
                    break

    def fly_to_destination_v2(self):
        pass






    def is_local_minima(self, pos_list):
        first_pos = pos_list[0]
        last_pos = pos_list[len(pos_list) - 1]
        if first_pos.distance(last_pos) < apf_config.grid_size * apf_config.window_size * 1.5:
            return True
        return False

    def escape_local_minima(self):

        lidar_data_list = []
        for i in range(10):
            self.client.rotateByAngle(30, 0.5)
            lidar_data = self.client.full_lidar_scan(0.3, 0.04, True)
            lidar_data_list.append(lidar_data)
            time.sleep(0.1)
        y = 9

    def apf_fly_to_destination(self, goal):
        cur_pose = self.client.getPose()
        start = (cur_pose.pos.x_m, cur_pose.pos.y_m)
        self._apf_path_planner = APFPathPlanner(start, goal)
        curr_position = start
        self._lidar_points_counter.start()
        reached_goal = False
        is_local_minima = False
        num_steps = 0
        pos_list = list()
        while not self._apf_path_planner.reached_goal(curr_position):
            next_position = self._apf_path_planner.next_step(curr_position, self._lidar_points)
            num_steps += 1
            pos_list.append(Point(*next_position))
            if num_steps == 10:
                is_local_minima = self.is_local_minima(pos_list)
                pos_list = list()
                num_steps = 0
                if is_local_minima:
                    print("Local Minima")
                    self.escape_local_minima()
            self._clear_lidar_points()
            self.client.flyToPosition(next_position[0], next_position[1], config.height,
                                      config.apf_velocity)
            print("fly to position")
            print(next_position[0], next_position[1])
            self._collect_lidar_points()
            while not self._apf_path_planner.reached_location(curr_position, next_position):
                curr_position = self.client.getPose().pos.x_m, self.client.getPose().pos.y_m
                self._collect_lidar_points()

                if num_steps == 10:
                    is_local_minima = self.is_local_minima(pos_list)
                    num_steps = 0
                    if is_local_minima:
                        print("Local Minima")

                if self._apf_path_planner.reached_goal(curr_position):
                    print("APF REACHED LOCAL GOAL")
                    reached_goal = True
                    break
            if reached_goal:
                break;
            curr_position = next_position

    def _collect_lidar_points(self):
        countdowner = Countdowner(0.8)
        countdowner.start()
        while countdowner.running():
            sensed_obstacle, lidar_data, pose = self.client.senseObstacle()
            if sensed_obstacle:
                parsed_lidar_data = self.client.parse_lidar_data(lidar_data)
                for lidar_sample in parsed_lidar_data:
                    x, y = self.client.getPointInRealWorldCoords(*lidar_sample, pose)
                    if self._apf_path_planner.new_obstacle((x, y)):
                        point = (round(x, 1), round(y, 1))
                        if not point in self._lidar_points:
                            print(point)
                            self._lidar_points.append(point)

    def _clear_lidar_points(self):
        if not self._lidar_points_counter.running():
            self._lidar_points = self._lidar_points[-20:]
            self._lidar_points_counter.start()

    @property
    def client(self):
        return self.client

    @client.setter
    def client(self, value):
        self._client = value
