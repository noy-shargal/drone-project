import math
import random
import time
from enum import Enum, unique

from shapely.geometry import Point
from DroneTypes import Position
from MyDroneClient import MyDroneClient
from ASTARPathPlanner import ASTARPathPlanner
from Config import config
from apf.APFPathPlanner import APFPathPlanner
from apf.Countdowner import Countdowner
from utils import getPointInRealWorldCoords


@unique
class AlgoState(Enum):
    ASTAR = 1
    APF = 2


class Agent:

    def __init__(self):

        self._algo = AlgoState.ASTAR

        self._astar_path_planner = ASTARPathPlanner()
        self._client = MyDroneClient()
        self._path = self._astar_path_planner.get_path()
        self._obs = self._astar_path_planner.get_obstacles_object()
        self._lidar_points_counter = Countdowner(5.0)
        self._lidar_points = list()

    def connect_and_spawn(self):
        self._client.reset()
        print("Connecting.....")
        self._client.connect()
        time.sleep(2)
        self._client.setAtPosition(config.source.x, config.source.y, config.height)
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
                client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.astar_velocity)
                need_fly_command = False
                print("Flying to point number: " + str(point_num) + str([goal.x_m, goal.y_m, goal.z_m]))

            if self.reached_goal_2D(client.getPose().pos, goal):
                print("Reached goal number : " + str(point_num))
                prev_point_num = point_num
                point_num += 1
                need_fly_command = True
                pos = client.getPose().pos

                if point_num == len(self._path):
                    print("Reached destination at (" + str(client.getPose().pos.x_m) + ", " + str(
                        client.getPose().pos.y_m) + ") ")
                    break

            sensing_obstacle, points_list, pose = client.senseObstacle()

            if sensing_obstacle:
                point = Point(points_list[0], points_list[1])
                world_point = getPointInRealWorldCoords(point.x, point.y, pose)
                if not self._obs.is_point_in_obstacles_map(Point(*world_point)):  # new obstacle
                    self._algo = AlgoState.APF
                    print("APF MODE")
                    tuple_goal = (goal.x_m, goal.y_m)
                    self.apf_fly_to_destination(tuple_goal)
                    self._algo = AlgoState.ASTAR
                    print("ASTAR MODE")



    def apf_fly_to_destination(self, goal):
        cur_pose = self._client.getPose()
        start = (cur_pose.pos.x_m, cur_pose.pos.y_m)
        apf_path_planner = APFPathPlanner(start, goal)
        curr_position = start
        self._lidar_points_counter.start()
        while not apf_path_planner.reached_goal(curr_position):
            next_position = apf_path_planner.next_step(curr_position, self._lidar_points)
            self._clear_lidar_points()
            random_next_position = (
                next_position[0] + 0.5 * random.random() - 0.25, next_position[1] + 0.5 * random.random() - 0.25)
            self._client.flyToPosition(random_next_position[0], random_next_position[1], config.height,
                                       config.apf_velocity)
            print("fly to position")
            print(next_position[0], next_position[1])
            self._collect_lidar_points()
            while not apf_path_planner.reached_location(curr_position, next_position):
                curr_position = self._client.getPose().pos.x_m, self._client.getPose().pos.y_m
                self._collect_lidar_points()
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
        return self._client
