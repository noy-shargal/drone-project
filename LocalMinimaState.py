import math
import time
from typing import Dict

import numpy as np
from shapely.geometry import Point, LineString

from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum
from Config import config
from MyDroneClient import LidarPointInfo


class Vector:
    def __init__(self, p1: Point, p2: Point):
        self._x = p2.x - p1.x
        self._y = p2.y - p1.y

    def normalize(self):
        norm = self.get_norm()
        self._x /= norm
        self._y /= norm

    def get_norm(self):
        return math.sqrt(self._x ** 2 + self._y ** 2)

    def multiple_by_scalar(self, scalar):
        self._x *= scalar
        self._y *= scalar

    def get_direction(self):
        return self._x, self._y

    def inner_product(self, other):
        return self._x * other._x + self._y * other._y

    def remove_vector(self, other):
        self._x -= other._x
        self._y -= other._y

    def opposite(self):
        return Vector(Point(self._x, self._y), Point(0, 0))

    def remove_projection(self, other):
        projection_scalar = self.inner_product(other) / self.get_norm()
        projection_vector = Vector(Point(0, 0), Point(self._x, self._y))
        projection_vector.multiple_by_scalar(projection_scalar)

        output = Vector(Point(0, 0), Point(other._x, other._y))
        output.remove_vector(projection_vector)
        output.normalize()
        return output

    def get_as_point(self):
        return Point(self._x, self._y)

    def get_angle(self):
        return math.atan2(self._y, self._x) * 180 / math.pi


class LocalMinimaState(AlgoStateInterface):
    def __init__(self, agent):
        super().__init__(AlgoStateEnum.LOCAL_MINIMA)
        self._agent = agent

    def enter(self):
        print("ENTER LOCAL MINIMA STATE")
        # initialize
        self._stop()
        time.sleep(0.5)
        m_line, d_min, start_position = self._calculate_m_line()  # LineString
        full_lidar_scan, world_cords_dict = self._rotate_to_face_target_and_scan(m_line)

        obstacle_vector, direction_vector = self._initialize_vectors(full_lidar_scan, world_cords_dict, m_line)
        current_position = start_position
        next_position = None
        while not self._crossed_m_line_at_lower_distance(m_line, d_min, start_position, next_position,
                                                         current_position):
            while not self._wall(full_lidar_scan, direction_vector) and self._wall(full_lidar_scan, obstacle_vector):
                next_position = self._calculate_next_position(direction_vector)
                self._fly_to_position_and_wait(next_position)
                full_lidar_scan = self._agent.client.full_lidar_scan(1.4)
            if self._wall(full_lidar_scan, direction_vector):
                obstacle_vector, direction_vector = self._rotate_vectors_wall_ahead(obstacle_vector,
                                                                                    direction_vector)
            if not self._wall(full_lidar_scan, obstacle_vector):
                obstacle_vector, direction_vector = self._rotate_vectors_wall_completed(obstacle_vector,
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
        return line, d_min, current_position

    def _rotate_to_face_target_and_scan(self, m_line, step=1):
        pos = self._agent.client.getPose().pos
        current_position = Point(pos.x_m, pos.y_m)
        norm = m_line.get_norm()

        next_x = current_position.x - m_line._x / norm * step
        next_y = current_position.y - m_line._y / norm * step

        self._agent.client.flyToPosition(next_x, next_y, config.height, 0.25)
        time.sleep(step/0.25)
        self._agent.client.flyToPosition(current_position.x, current_position.y, config.height, 0.25)

        full_lidar_scan, world_cords_dict = self._agent.client.full_lidar_scan_v2(1.4)

        time.sleep(0.1)

        return full_lidar_scan, world_cords_dict

    def _initialize_vectors(self, full_lidar_scan, world_coords: Dict[int, LidarPointInfo], m_line):
        angle_index = np.argmin(full_lidar_scan).item()
        point_info = world_coords[angle_index]
        nearest_obs_point = Point(point_info.x, point_info.y)
        pos = self._agent.client.getPose().pos
        current_position = Point(pos.x_m, pos.y_m)
        vector_to_obstacle = Vector(current_position, nearest_obs_point)
        vector_to_obstacle.normalize()

        direction_vector = vector_to_obstacle.remove_projection(m_line)

        return vector_to_obstacle, direction_vector

    def _crossed_m_line_at_lower_distance(self, m_line: Vector, d_min: float, start_position: Point,
                                          next_position: Point,
                                          current_position: Point,
                                          threshold=1):
        if next_position is None:
            return False
        target = self._agent.path[self._agent.astar_curr_point].point()
        distance_to_target = next_position.distance(target)
        if distance_to_target > (d_min - threshold):
            return False
        next_in_start = Point(next_position.x - start_position.x, next_position.y - start_position.y)
        current_in_start = Point(current_position.x - start_position.x, current_position.y - start_position.y)

        step_line_string = LineString([current_in_start, next_in_start])
        m_line_string = LineString([Point(0, 0), m_line.get_as_point()])

        return step_line_string.intersects(m_line_string)

    def _wall(self, full_lidar_scan, vector):
        threshold = 15
        angle = vector.get_angle()
        left_angle = angle - threshold
        right_angle = angle + threshold

        left_index = self._agent.client._angle_to_index(left_angle)
        right_index = self._agent.client._angle_to_index(right_angle)

        for angle_index in range(left_index, right_index + 1):
            if full_lidar_scan[angle_index] < np.float(np.inf):
                return True
        return False

    def _calculate_next_position(self, direction_vector: Vector):
        pos = self._agent.client.getPose().pos
        x, y = pos.x_m, pos.y_m
        vx, vy = direction_vector._x, direction_vector._y
        x = x + vx
        y = y + vy
        return Point(x, y)

    def _fly_to_position_and_wait(self, next_position: Point):
        pos = self._agent.client.getPose().pos
        x, y = pos.x_m, pos.y_m
        curr_pos = Point(x, y)
        self._agent.client.flyToPosition(next_position.x, next_position.y, config.height, config.bug2_velocity)
        while not self._agent.point_reached_goal_2D(curr_pos, next_position, 0.5):
            pos = self._agent.client.getPose().pos
            x, y = pos.x_m, pos.y_m
            curr_pos = Point(x, y)

    def _rotate_vectors_wall_ahead(self, obstacle_vector, direction_vector):
        new_obstacle_vector = direction_vector
        new_direction_vector = obstacle_vector.opposite()
        return new_obstacle_vector, new_direction_vector

    def _rotate_vectors_wall_completed(self, obstacle_vector, direction_vector):
        new_obstacle_vector = direction_vector.opposite()
        new_direction_vector = obstacle_vector
        return new_obstacle_vector, new_direction_vector
