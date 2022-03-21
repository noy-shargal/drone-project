import math

from shapely.geometry import Point

import DroneTypes
from DroneClient import DroneClient
import numpy as np
from DroneTypes import Position


class MyDroneClient(DroneClient):

    def __init__(self):
        super().__init__()
        self.target_params = None
        self.stopped = False
        self._state = "GO_TOP_POINT"

    def foolowWall(self):
        self._state = "FOLLOW_OBSTACLE_WALL"

    def goToPoint(self):
        self._state = "GO_TO_POINT"

    def getState(self):
        return self._state

    def toCoords(self, pose):

        x = pose.pos.x_m
        y = pose.pos.y_m
        z = pose.pos.z_m
        return x, y, z

    def stop(self):
        if self.stopped:
            return

        self.stopped = True
        pose_res = self.getPose()
        x, y, z = self.toCoords(pose_res)
        self.setAtPosition(x, y, z)

    def resume(self):
        super().flyToPosition(self.target_params)

    def flyToPosition(self, x: float, y: float, z: float, v: float):
        self.target_params = x, y, z, v
        super().flyToPosition(x, y, z, v)

    def getLidarData(self):
        point_cloud = DroneTypes.PointCloud()
        lidar_data = self.client.getLidarData('Lidar1')
        # point_cloud.points = lidar_data.point_cloud
        return lidar_data


        # point_cloud = DroneTypes.PointCloud()
        # lidar_data = self.client.getLidarData()
        #
        # point_cloud.points = lidar_data.point_cloud
        #
        # return point_cloud, self.getPose()

    def senseObstacle(self):
        #lidar_data, pose = self.getLidarData()
        lidar_data  = self.getLidarData()
        if lidar_data.point_cloud == [0.0]:
            return False, [0.0], lidar_data.pose
        return True, lidar_data.point_cloud, lidar_data.pose

    def getPointInRealWorldCoords(self, x_drone, y_drone, pose):
        theta = pose.orientation.z_rad
        x_world = x_drone * np.cos(theta) - y_drone * np.sin(theta) + pose.pos.x_m
        y_world = x_drone * np.sin(theta) + y_drone * np.cos(theta) + pose.pos.y_m
        return x_world, y_world

    def getPointInPolarCoords(self, rel_x, rel_y):
        r = math.sqrt(rel_x * rel_x + rel_y * rel_y)
        theta = math.atan2(rel_y, rel_x)
        return r, theta


    def position_to_point(self, pos: Position):
        x = pos.x_m
        y = pos.y_m
        return Point(x, y)

    def reached_goal_2D(self, curr: Position, goal: Position):
        diff_x = curr.x_m - goal.x_m
        diff_y = curr.y_m - goal.y_m
        dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)

        if dist < 5.0:
            return True
        return False
