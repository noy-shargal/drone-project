import math

import numpy as np
from shapely.geometry import LineString, Point

from utils import getPointInRealWorldCoords


class ThinWallObstacle:
    def __init__(self, first_endpoint: Point, second_endpoint: Point):
        self._first_endpoint = first_endpoint
        self._second_endpoint = second_endpoint
        self._line = LineString([self._first_endpoint, self._second_endpoint])
        self._compare_epsilon = 0.5

    def intersects_line(self, line: LineString):
        return line.intersects(self._line)

    def point_is_endpoint(self, point):
        return self._first_endpoint.distance(point) < self._compare_epsilon or self._second_endpoint.distance(
            point) < self._compare_epsilon

    def get_closest_point_to_target(self, target: Point):
        d_min1 = target.distance(self._first_endpoint)
        d_min2 = target.distance(self._second_endpoint)
        if d_min1 < d_min2:
            return d_min1, self._first_endpoint
        return d_min2, self._second_endpoint


class ObstacleBuilder:
    def __init__(self, full_lidar_scan, current_pose, verbose = False):
        self.obstacles = list()
        self.first_endpoint_r_index = None
        self.trend = None
        self.padded_full_lidar_scan = np.array(np.concatenate(([np.float(np.inf)], full_lidar_scan,
                                                               [np.float(np.inf)])))
        self.current_pose = current_pose
        self.verbose = verbose

    def build_obstacles(self):
        for i in range(1, len(self.padded_full_lidar_scan) - 1):
            if self.padded_full_lidar_scan[i] < np.float(np.inf) and self.padded_full_lidar_scan[i - 1] == np.float(np.inf):
                self.first_endpoint_r_index = i
            if self.padded_full_lidar_scan[i + 1] == np.float(np.inf) and self.padded_full_lidar_scan[i] < np.float(np.inf):
                second_endpoint_r_index = i
                assert self.first_endpoint_r_index is not None
                new_obs = self._build_obstacle(self.first_endpoint_r_index, second_endpoint_r_index, self.current_pose,
                                               self.padded_full_lidar_scan)
                self.obstacles.append(new_obs)
                self.first_endpoint_r_index = None
            elif self.first_endpoint_r_index is not None and self.first_endpoint_r_index < i:
                current_trend = np.sign(self.padded_full_lidar_scan[i] - self.padded_full_lidar_scan[i-1])
                if self.trend is None:
                    self.trend = current_trend
                elif current_trend != self.trend:
                    second_endpoint_r_index = i
                    new_obs = self._build_obstacle(self.first_endpoint_r_index, second_endpoint_r_index, self.current_pose,
                                                   self.padded_full_lidar_scan)
                    self.obstacles.append(new_obs)
                    self.first_endpoint_r_index = None

    def _build_obstacle(self, first_endpoint_r_index, second_endpoint_r_index, current_pose, full_lidar_scan):
        first_endpoint_r = full_lidar_scan[first_endpoint_r_index]
        first_theta = self._angle_index_to_value(first_endpoint_r_index - 0.5, len(full_lidar_scan))
        second_endpoint_r = full_lidar_scan[second_endpoint_r_index]
        second_theta = self._angle_index_to_value(second_endpoint_r_index + 0.5, len(full_lidar_scan))

        first_endpoint_in_world_coordinates = self._calculate_world_coordinates(first_endpoint_r, first_theta,
                                                                                current_pose)
        second_endpoint_in_world_coordinates = self._calculate_world_coordinates(second_endpoint_r, second_theta,
                                                                                 current_pose)
        obs = ThinWallObstacle(first_endpoint_in_world_coordinates, second_endpoint_in_world_coordinates)
        return obs

    @staticmethod
    def _calculate_world_coordinates(r, theta, current_pose):
        real_theta = theta - 90
        real_theta = real_theta * math.pi /180

        x_drone = r * np.cos(real_theta)
        y_drone = r * np.sin(real_theta)

        xw, yw = getPointInRealWorldCoords(x_drone, y_drone, current_pose)

        return Point(xw, yw)

    @staticmethod
    def _angle_index_to_value(angle_index, num_of_values):
        return angle_index * 180 / num_of_values