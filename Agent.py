from PathPlanner import PathPlanner
from MyDroneClient import MyDroneClient
from Countdowner import Countdowner
from typing import Tuple
import time
from config import current_config


class Agent:

    def __init__(self):
        self._path_planner = PathPlanner()
        self._drone_client = MyDroneClient()
        self._start = current_config.start_position
        self._height = current_config.height
        self._velocity = current_config.velocity
        self._lidar_points_counter = Countdowner(5.0)
        self._lidar_points_counter.start()
        self._lidar_points = set()

    def connect_and_spawn(self):
        self._drone_client.connect()
        time.sleep(2)
        self._drone_client.setAtPosition(*self._start, self._height)
        time.sleep(2)

    def fly_to_destination(self):
        curr_position = self._start
        while not self._path_planner.reached_goal(curr_position):
            next_position = self._path_planner.next_step(curr_position, self._lidar_points)
            self._clear_lidar_points()
            self._drone_client.flyToPosition(next_position[0], next_position[1], self._height, self._velocity)
            self._collect_lidar_points()
            while not self._path_planner.reached_location(curr_position, next_position):
                curr_position = self._drone_client.getPose().pos.x_m, self._drone_client.getPose().pos.y_m
                self._collect_lidar_points()
            curr_position = next_position

    def _collect_lidar_points(self):
        countdowner = Countdowner(1.0)
        countdowner.start()
        while countdowner.running():
            sensed_obstacle, lidar_data, pose = self._drone_client.senseObstacle()
            if sensed_obstacle:
                parsed_lidar_data = self._drone_client.parse_lidar_data(lidar_data)
                for lidar_sample in parsed_lidar_data:
                    x, y = self._drone_client.getPointInRealWorldCoords(*lidar_sample, pose)
                    if self._path_planner.new_obstacle((x, y)):
                        point = (round(x, 2), round(y, 2))
                        if not point in self._lidar_points:
                            print(point)
                            self._lidar_points.add(point)

    def _clear_lidar_points(self):
        if not self._lidar_points_counter.running():
            self._lidar_points = set()
            self._lidar_points_counter.start()
