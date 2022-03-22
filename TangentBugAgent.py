import math
import time

from shapely.geometry import Point

from DroneTypes import Position
from MyDroneClient import MyDroneClient
from PathPlanner import PathPlanner
from TangentBug import TangentBug
from Config  import config
from utils import get_parallelogram_missing_point


class TangentBugAgent:

    def __init__(self):
        self._client = MyDroneClient()
        self._tangent_bug = TangentBug(config.destination)

    def connect_and_spawn(self):
        self._client.reset()
        print("Connecting.....")
        self._client.connect()
        time.sleep(2)
        self._client.setAtPosition(config.source.x, config.source.y , config.height)
        time.sleep(2)
        print(self._client.isConnected())
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

    def fly_to_destination(self):

        curr_pos = config.source
        next_step = curr_pos
        while not self.point_reached_goal_2D(next_step, config.destination):
            full_lidar_scan = self._client.full_lidar_scan()
            curr_pose = self._client.getPose()
            next_step = self._tangent_bug.step(curr_pose, full_lidar_scan)
            self._client.flyToPosition(next_step.x, next_step.y, config.height, config.velocity)
            time.sleep(0.5)
            print(f"current position: ({next_step.x}, {next_step.y})")

    @property
    def client(self):
        return self._client


