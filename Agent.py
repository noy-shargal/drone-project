from PathPlanner import PathPlanner
from MyDroneClient import MyDroneClient
from Countdowner import Countdowner
from typing import Tuple
import time
from config import current_config


class Agent:
    HEIGHT = -30
    VELOCITY = 6

    def __init__(self):
        self._path_planner = PathPlanner()
        self._drone_client = MyDroneClient()
        self._start = current_config.start_position
        self._height = current_config.height
        self._velocity = current_config.velocity
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
        countdowner = Countdowner(0.1)
        countdowner.start()
        while countdowner.running():
            sensed_obstacle, lidar_data, pose = self._drone_client.senseObstacle()
            if sensed_obstacle:
                parsed_lidar_data = self._drone_client.parse_lidar_data(lidar_data)
                for lidar_sample in parsed_lidar_data:
                    x, y = self._drone_client.getPointInRealWorldCoords(*lidar_sample, pose)
                    if self._path_planner.new_obstacle((x, y)):
                        print((round(x, 2), round(y, 2)))
                        self._lidar_points.add((round(x, 2), round(y, 2)))

    def _clear_lidar_points(self):
        self._lidar_points = set()
