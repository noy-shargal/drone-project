import math
import random
import time
from enum import Enum, unique
from typing import List

import numpy as np
from shapely.geometry import Point

from AStarState import AStarState
from AlgoFSM import AlgoFSM
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
        print("Finished A-Star Algorithm")
        self.astar_curr_point = 0
        self.client = MyDroneClient()
        self.path = self._astar_path_planner.get_path()
        self.obs = self._astar_path_planner.get_obstacles_object()
        self._lidar_points_counter = Countdowner(5.0)
        self._lidar_points = list()

        self._algo_fsm = AlgoFSM(self, AlgoStateEnum.ASTAR, AlgoStateEnum.END)


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

        print("Init position " + str([config.source.x, config.source.y, config.height]))
        self.astar_curr_point = 1

        state_enum = self._algo_fsm.init_state_enum()
        while state_enum != AlgoStateEnum.END:
            next_state_enum = self._algo_fsm.change_state(state_enum)
            state_enum = next_state_enum

        print("Reached Goal !!! ")

    def is_local_minima_in_map(self, pos_list: List):
        for p in pos_list:
            if self.obs.is_point_in_obstacles_map(p):
                return True
        return False



    def is_local_minima(self, pos_list):
        first_pos = pos_list[0]
        last_pos = pos_list[len(pos_list) - 1]
        if first_pos.distance(last_pos) < apf_config.grid_size * apf_config.window_size * 1.5:
            if self.is_local_minima_in_map(pos_list):
                print("This local minima ia in MAP - you should have done better !!!")
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

    def rotate_for_unknown_obstacles(self):

        lidar_data_list = []
        for i in range(8):
            self.client.rotateByAngle(90, 0.5)
            pose = self.client.getPose()
            lidar_data = self.client.full_lidar_scan_v2(0.3, pose, 0.04, True)
            for p in lidar_data:
                if p != np.float(np.inf):
                    point = Point(p.x, p.y)
                    if not self.obs.is_point_in_obstacles_map(point):
                        return True
            lidar_data_list.append(lidar_data)
            time.sleep(0.1)
        return False

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




