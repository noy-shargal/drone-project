import math
import time
from typing import Dict

import numpy as np
import numpy.linalg as LA

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

    def __str__(self):
        return str(self._x) + '_' + str(self._y)


class LocalMinimaState(AlgoStateInterface):
    STEP_SIZE = 2
    WALL_AHEAD_DISTANCE = 4
    KEEP_DISTANCE_FROM_WALL = 6

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.LOCAL_MINIMA)
        self._agent = agent

    def enter(self):
        print("ENTER LOCAL MINIMA STATE")
        # initialize
        self._stop()
        client = self._agent.client
        time.sleep(0.5)
        m_line, d_min, start_position = self._calculate_m_line()  # LineString
        full_lidar_scan, world_cords_dict, pos = self._rotate_to_face_target_and_scan(m_line)

        obstacle_vector, direction_vector = self._initialize_vectors(full_lidar_scan, world_cords_dict, m_line, pos)
        current_position = start_position
        next_position = None
        num_of_steps = 0
        while not self._crossed_m_line_at_lower_distance(m_line, d_min, start_position, next_position,
                                                         current_position):
            if not self._wall_word_coordinates(world_cords_dict, current_position, direction_vector,
                                               self.WALL_AHEAD_DISTANCE) and self._wall_word_coordinates(
                world_cords_dict, current_position, obstacle_vector, 4.5 * self.WALL_AHEAD_DISTANCE):
                print("REGULAR STEP")
                distance_to_wall = self._distance_to_wall_world_coordinates(world_cords_dict, current_position,
                                                                            obstacle_vector)
                if distance_to_wall > self.KEEP_DISTANCE_FROM_WALL:
                    multiply = 0.25
                else:
                    multiply = -0.25
                next_position = self._calculate_next_position(direction_vector, 1, obstacle_vector, multiply)
                world_cords_dict = self._fly_to_position_and_wait(next_position)
            if self._wall_word_coordinates(world_cords_dict, current_position, direction_vector,
                                           self.WALL_AHEAD_DISTANCE, 1.5, True):
                print("TURN BECAUSE OF WALL")

                obstacle_vector, direction_vector = self._rotate_vectors_wall_ahead(obstacle_vector,
                                                                                    direction_vector)
                next_position = self._calculate_next_position(direction_vector)
                world_cords_dict = self._fly_to_position_and_wait(next_position)
                self._agent.client.full_lidar_scan_v2(1.4)
            elif not self._wall_word_coordinates(
                    world_cords_dict, current_position, obstacle_vector,
                    4.5 * self.WALL_AHEAD_DISTANCE):
                print("TURN BECAUSE OF NO WALL")
                obstacle_vector, direction_vector = self._rotate_vectors_wall_completed(obstacle_vector,
                                                                                        direction_vector)
                next_position = self._calculate_next_position(direction_vector, 3, obstacle_vector, -1)
                world_cords_dict = self._fly_to_position_and_wait(next_position)
            pos = client.getPose().pos
            current_position = Point(pos.x_m, pos.y_m)
            num_of_steps += 1
            if num_of_steps == 100:
                break
        self._stop()
        time.sleep(0.5)
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

    def _rotate_to_face_target_and_scan(self, m_line, step=10):
        speed = 1.25
        pos = self._agent.client.getPose().pos
        current_position = Point(pos.x_m, pos.y_m)
        norm = m_line.get_norm()

        next_x = current_position.x - m_line._x / norm * step
        next_y = current_position.y - m_line._y / norm * step

        self._agent.client.flyToPosition(next_x, next_y, config.height, speed)
        time.sleep(step / speed)
        self._agent.client.flyToPosition(current_position.x, current_position.y, config.height, speed / 16)
        time.sleep(5)
        full_lidar_scan, world_cords_dict = self._agent.client.full_lidar_scan_v2(1.4)
        pos = self._agent.client.getPose().pos

        time.sleep(0.1)

        return full_lidar_scan, world_cords_dict, pos

    def _initialize_vectors(self, full_lidar_scan, world_coords: Dict[int, LidarPointInfo], m_line, pos):
        left_relevant_angle = len(full_lidar_scan) // 2 - 1
        right_relevant_angle = len(full_lidar_scan) // 2 + 2
        angle_index = np.argmin(full_lidar_scan[left_relevant_angle:right_relevant_angle]).item()
        angle_index += left_relevant_angle

        second_index = angle_index - 1 if full_lidar_scan[angle_index - 1] < full_lidar_scan[
            angle_index + 1] else angle_index + 1
        if not world_coords[second_index].valid:
            second_index = angle_index
        point_info = world_coords[angle_index]
        second_point_info = world_coords[second_index]
        nearest_obs_point = Point(1 / 2 * (point_info.x + second_point_info.x),
                                  1 / 2 * (point_info.y + second_point_info.y))
        current_position = Point(pos.x_m, pos.y_m)
        vector_to_obstacle = Vector(current_position, nearest_obs_point)
        vector_to_obstacle.normalize()

        direction_vector = vector_to_obstacle.remove_projection(m_line)

        return vector_to_obstacle, direction_vector

    def _crossed_m_line_at_lower_distance(self, m_line: Vector, d_min: float, start_position: Point,
                                          next_position: Point,
                                          current_position: Point,
                                          threshold=10,
                                          simple_mode=True):
        if next_position is None:
            return False
        target = self._agent.path[self._agent.astar_curr_point].point()
        distance_to_target = next_position.distance(target)
        print("DISTANCE TO TARGET :"+str(distance_to_target))
        if distance_to_target > (d_min - threshold):
            return False
        if simple_mode:
            return True
        next_in_start = Point(next_position.x - start_position.x, next_position.y - start_position.y)
        current_in_start = Point(current_position.x - start_position.x, current_position.y - start_position.y)

        step_line_string = LineString([current_in_start, next_in_start])
        m_line_string = LineString([Point(0, 0), m_line.get_as_point()])

        return step_line_string.intersects(m_line_string)

    def _wall(self, full_lidar_scan, vector, distance_to_wall=np.float(np.inf)):
        threshold = 5
        angle = vector.get_angle()
        left_angle = angle - threshold
        right_angle = angle + threshold

        left_index = self._agent.client._angle_to_index(left_angle)
        right_index = self._agent.client._angle_to_index(right_angle)

        for angle_index in range(left_index, right_index + 1):
            if left_index >= len(full_lidar_scan) and right_index <= 2 * len(full_lidar_scan):
                return False
            if full_lidar_scan[angle_index] < distance_to_wall:
                return True
        return False

    def _wall_word_coordinates(self, world_coordinates: Dict[int, LidarPointInfo], current_location: Point,
                               vector: Vector,
                               distance=np.float(np.inf),
                               threshold=2.5,
                               verbose=False):
        for point_info in world_coordinates.values():
            if point_info.valid:
                angle = self._calculate_angle(current_location, vector, point_info.x, point_info.y)
                if angle < threshold:
                    wall_point = Point(point_info.x, point_info.y)
                    distance_to_wall = current_location.distance(wall_point)
                    if distance_to_wall < distance:
                        if verbose:
                            print(f"WALL IN {distance_to_wall} METERS AND {angle} degrees")
                        return True
        return False

    def _distance_to_wall_world_coordinates(self, world_coordinates: Dict[int, LidarPointInfo], current_location: Point,
                                            vector: Vector):
        threshold = 2.5
        for point_info in world_coordinates.values():
            if point_info.valid:
                angle = self._calculate_angle(current_location, vector, point_info.x, point_info.y)
                if angle < threshold:
                    wall_point = Point(point_info.x, point_info.y)
                    distance_to_wall = current_location.distance(wall_point)
                    return distance_to_wall
        return np.float(np.inf)

    def _calculate_next_position(self, direction_vector: Vector, multiple=1, additional_vector=None,
                                 additional_multiply=0.01):
        pos = self._agent.client.getPose().pos
        x, y = pos.x_m, pos.y_m
        vx, vy = direction_vector._x, direction_vector._y
        x = x + vx * self.STEP_SIZE * multiple
        y = y + vy * self.STEP_SIZE * multiple
        if additional_vector is not None:
            vx_2, vy_2 = additional_vector._x, additional_vector._y
            x = x + vx_2 * self.STEP_SIZE * additional_multiply
            y = y + vy_2 * self.STEP_SIZE * additional_multiply
        return Point(x, y)

    def _fly_to_position_and_wait(self, next_position: Point):
        pos = self._agent.client.getPose().pos
        x, y = pos.x_m, pos.y_m
        curr_pos = Point(x, y)
        self._agent.client.flyToPosition(next_position.x, next_position.y, config.height, config.bug2_velocity)
        time.sleep(0.5)
        _, world_cords_dict = self._agent.client.full_lidar_scan_v2(1.4)
        while not self._agent.point_reached_goal_2D(curr_pos, next_position, 0.5):
            _, world_cords_dict = self._agent.client.full_lidar_scan_v2(1.4)
            pos = self._agent.client.getPose().pos
            x, y = pos.x_m, pos.y_m
            curr_pos = Point(x, y)
        return world_cords_dict

    def _rotate_vectors_wall_ahead(self, obstacle_vector, direction_vector):
        new_obstacle_vector = direction_vector
        new_direction_vector = obstacle_vector.opposite()
        return new_obstacle_vector, new_direction_vector

    def _rotate_vectors_wall_completed(self, obstacle_vector, direction_vector):
        new_obstacle_vector = direction_vector.opposite()
        new_direction_vector = obstacle_vector
        return new_obstacle_vector, new_direction_vector

    @staticmethod
    def _calculate_angle(current_location: Point, vector: Vector, x: float, y: float):
        numpy_vector = np.array([current_location.x + vector._x, current_location.y + vector._y])
        numpy_wall = np.array([x, y])
        inner = np.inner(numpy_vector, numpy_wall)
        norms = LA.norm(numpy_vector) * LA.norm(numpy_wall)

        cos = inner / norms
        rad = np.arccos(np.clip(cos, -1.0, 1.0))
        deg = np.rad2deg(rad)
        return deg
