import math
import time

from shapely.geometry import Point

from DroneTypes import Position
from MyDroneClient import MyDroneClient
from PathPlanner import PathPlanner
from TangentBug import TangentBug
from Config  import config
from utils import get_parallelogram_missing_point


class Agent:

    def __init__(self):
        self._path_planner = PathPlanner()
        self._client = MyDroneClient()

        # self._lidar_points_counter = Countdowner(5.0)
        # self._lidar_points = list()
        self._path = self._path_planner.get_path()

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

    def position_to_point(self, pos: Position):
        return Point(pos.x_m, pos.y_m)

    def fly_to_destination(self):

        print("Init position " + str([config.source.x, config.source.y, config.height]))
        prev_point_num = 0
        point_num = 1
        need_fly_command = True
        real_path = list()
        client = self._client
        goal = Position()

        while True:

            lidar_data = client.getLidarData()

            p = self._path[point_num].point()
            goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
            if need_fly_command:
                client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.velocity)
                need_fly_command = False
                print("Flying to point number: " + str(point_num) + str([goal.x_m, goal.y_m, goal.z_m]))

            if self.reached_goal_2D(client.getPose().pos, goal):
                print("Reached goal number : " + str(point_num))
                prev_point_num = point_num
                point_num += 1
                need_fly_command = True
                pos = client.getPose().pos
                real_path.append(self.position_to_point(pos))
                if point_num == len(self._path):
                    print("Reached destination at (" + str(client.getPose().pos.x_m) + ", " + str(
                        client.getPose().pos.y_m) + ") ")
                    break

            # sensing_obstacle, points_list, pose = client.senseObstacle()
            # if sensing_obstacle:

    @property
    def client(self):
        return self._client


