import math
import time
from typing import List
import numpy as np
from shapely.geometry import Point
from AlgoFSM import AlgoFSM
from AlgoStateInterface import AlgoStateEnum
from DroneTypes import Position
from MapDrawer import MapDrawer
from MyDroneClient import MyDroneClient, LidarPointInfo
from ASTARPathPlanner import ASTARPathPlanner
from Config import config
from apf.Countdowner import Countdowner
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

        self._real_path = list()
        self._algo_fsm = AlgoFSM(self, AlgoStateEnum.ASTAR, AlgoStateEnum.END)
        self.apf_sleep_after_transition = 0.0

    def connect_and_spawn(self):
        self.client.reset()
        print("Connecting.....")
        self.client.connect()
        time.sleep(4)
        self.client.setAtPosition(config.source.x, config.source.y, config.height)
        time.sleep(3)
        print(self.client.isConnected())
        time.sleep(8)

    def position_to_point(self, pos: Position):
        return Point(pos.x_m, pos.y_m)

    def add_path_point(self, point: Point):
        self._real_path_point_couner += 1

        if self._real_path_ration == 20:
            self._real_path_point_couner = 1
            self._real_path.append(point)
    def reached_goal_2D(self, curr_pos: Position, goal: Position):
        diff_x = curr_pos.x_m - goal.x_m
        diff_y = curr_pos.y_m - goal.y_m
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        if dist < 5.0:
            return True
        return False

    def point_reached_goal_2D(self, curr_pos: Point, goal: Point, threshold=5):
        diff_x = curr_pos.x - goal.x
        diff_y = curr_pos.y - goal.y
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        if dist <threshold:
            return True
        return False

    def position_to_point(self, pos: Position):
        return Point(pos.x_m, pos.y_m)


    def fly_to_destination(self):

        print("Init position " + str([config.source.x, config.source.y, config.height]))
        self.astar_curr_point = 1
        state_enum = self._algo_fsm.init_state_enum()
        while state_enum != AlgoStateEnum.TERMINAL:
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
                curr_pos = self.client.getPose().pos.x_m, self.client.getPose().pos.x_m
                print("local minima:"+str(curr_pos)+" ia in MAP - you should have done better !!!")
            return True
        return False

    def _is_obstacle_in_angle_range(self, lidar_data,  left_angle, right_angle):
        left_index =   self.client._angle_to_index(left_angle, config.lidar_theta_resolution)
        right_index = self.client._angle_to_index(right_angle, config.lidar_theta_resolution)
        middle = self.client.zero_angle_to_index(config.lidar_theta_resolution)

        left_obs = False
        right_obs = False

        found_left_index = -1
        for i_left in range(left_index,middle +1 ):
            if lidar_data[i_left] != np.float(np.inf):
                left_obs = True
                found_left_index = i_left
                break

        found_right_index = -1
        for i_right in range(right_index, middle -1 , -1):
            if lidar_data[i_right] != np.float(np.inf):
                right_obs = True
                found_right_index = i_right
                break

        return left_obs ,right_obs, found_left_index, found_right_index


    def is_wall_ahead(self, angle_fov=config.wall_detection_angle_fov):
        lidar_data, lidar_coord_dict = self.client.full_lidar_scan_v2(0.5, 0.04)
        left_obs, right_obs, i_left, i_right = self._is_obstacle_in_angle_range(lidar_data, -0.5 * angle_fov, 0.5 * angle_fov)

        if left_obs and right_obs:
            lidar_point_info = lidar_coord_dict[i_left]
            print("WALL AHEAD  "+str(lidar_point_info.r)+" meters ahead !!!!")
            if lidar_point_info.r >= 8:
                point = Point(lidar_point_info.x, lidar_point_info.y)
                return True, point

        if right_obs:
            print("WALL AHEAD ON THE RIGHT !!!!")
            lidar_point_info = lidar_coord_dict[i_right]
            point = Point(lidar_point_info.x, lidar_point_info.y)
            return True, point

        # if left_obs:
        #     print("WALL AHEAD ON THE LEFT !!!!")
        #     lidar_point_info = lidar_coord_dict[i_left]
        #     point = Point(lidar_point_info.x, lidar_point_info.y)
        #     return True, point
        return False, None

    def escape_local_minima(self):

        lidar_data_list = []
        for i in range(10):
            self.client.rotateByAngle(30, 0.5)
            lidar_data = self.client.full_lidar_scan(0.3, 0.04, True)
            lidar_data_list.append(lidar_data)
            time.sleep(0.1)

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
                        self._apf_path_planner.add_new_obstacle((x, y))
                        if not point in self._lidar_points:
                            print(point)
                            self._lidar_points.append(point)

    def _clear_lidar_points(self):
        if not self._lidar_points_counter.running():
            self._lidar_points = self._lidar_points[-20:]
            self._lidar_points_counter.start()

    def show_real_path(self):

        start_point = self.path[0].point()
        destination_point = self.path[-1].point()

        if config.show_map:
            md = MapDrawer()
            md.set_source(start_point)
            md.set_destination(destination_point)
            md.set_real_path(self._real_path)
            md.show()
            time.sleep(5)


