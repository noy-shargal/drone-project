import math
from typing import Dict

import numpy as np
from shapely.geometry import Point, LineString

from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum
from Config import config
from MyDroneClient import LidarPointInfo
from SmartAgent_v1 import SmartAgent_v1


class Vector:
    def __init__(self, p1: Point, p2: Point):
        self._x = p2.x - p1.x
        self._y = p2.y - p1.y

    def normalize(self):
        norm = self.get_norm()
        self._x /= norm
        self._y /= norm

    def get_norm(self):
        return math.sqrt(self._x**2 + self._y**2)

    def multiple_by_scalar(self, scalar):
        self._x *= scalar
        self._y *= scalar

    def get_direction(self):
        return self._x, self._y

    def inner_product(self, other):
        return self._x*other._x + self._y*other._y

    def remove_vector(self, other):
        self._x -= other._x
        self._y -= other._y

    def remove_projection(self, other):
        projection_scalar = self.inner_product(other) / self.get_norm()
        projection_vector = Vector(Point(0,0), Point(self._x, self._y))
        projection_vector.multiple_by_scalar(projection_scalar)

        output = Vector(Point(0,0), Point(other._x, other._y))
        output.remove_vector(projection_vector)
        output.normalize()
        return output


class LocalMinimaState(AlgoStateInterface):
    def __init__(self, agent: SmartAgent_v1):
        super().__init__(AlgoStateEnum.LOCAL_MINIMA)
        self._agent = agent

    def enter(self):
        print("ENTER LOCAL MINIMA STATE")
        # initialize
        self._stop()
        m_line, d_min = self._calculate_m_line()  # LineString
        self._rotate_to_face_target(m_line)
        full_lidar_scan, world_cords_dict = self._agent.client.full_lidar_scan_v2()
        vector_to_obstacle, direction_vector = self._initialize_vectors(full_lidar_scan, world_cords_dict, m_line)
        while not self._crossed_M_line_at_lower_distance(m_line, d_min):
            while not self._wall(full_lidar_scan, direction_vector) and self._wall(full_lidar_scan, vector_to_obstacle):
                full_lidar_scan = self._agent.client.full_lidar_scan()
                next_position = self._calculate_next_position(direction_vector)
                self._fly_to_position_and_wait(next_position)
            if self._wall(full_lidar_scan, direction_vector):
                vector_to_obstacle, direction_vector = self._rotate_vectors_wall_ahead(vector_to_obstacle,
                                                                                       direction_vector)
            if not self._wall(full_lidar_scan, vector_to_obstacle):
                vector_to_obstacle, direction_vector = self._rotate_vectors_wall_completed(vector_to_obstacle,
                                                                                           direction_vector)
        return AlgoStateEnum.TRANSISTION

    def exit(self):
        print("EXIT LOCAL MINIMA STATE")
        return

    def _stop(self):
        self._agent.client.stop()
        return

    def _calculate_m_line(self):
        target = self._agent.path[self._agent.astar_curr_point].point()
        pos = self._agent.client.getPose().pos
        current_position = Point(pos.x_m, pos.y_m)
        line = Vector(current_position, target)
        d_min = current_position.distance(target)
        return line, d_min

    def _rotate_to_face_target(self, m_line):
        pass

    def _initialize_vectors(self, full_lidar_scan, world_coords: Dict[int, LidarPointInfo], m_line):
        angle_index = np.argmin(full_lidar_scan)[0].item()
        point_info = world_coords[angle_index]
        nearest_obs_point = Point(point_info.x, point_info.y)
        pos = self._agent.client.getPose().pos
        current_position = Point(pos.x_m, pos.y_m)
        vector_to_obstacle = Vector(current_position, nearest_obs_point)

        direction_vector = vector_to_obstacle.remove_projection(m_line)

        return vector_to_obstacle, direction_vector

    def _crossed_M_line_at_lower_distance(self, m_line, d_min):
        pass

