import numpy as np
from shapely.geometry import LineString, Point


class ThinWallObstacle:
    def __init__(self, first_endpoint: Point, second_endpoint: Point):
        self._first_endpoint = first_endpoint
        self._second_endpoint = second_endpoint
        self._line = LineString([self._first_endpoint, self._second_endpoint])

    def intersects_line(self, line: LineString):
        return line.intersects(self._line)

    def get_closest_point_to_target(self, target: Point):
        d_min1 = target.distance(self._first_endpoint)
        d_min2 = target.distance(self._second_endpoint)
        if d_min1 < d_min2:
            return d_min1, self._first_endpoint
        return d_min2, self._second_endpoint

    def get_positive_direction(self, previous_step, target: Point, curr_pos: Point):

        to_first_endpoints = np.array([self._first_endpoint.x - curr_pos.x,
                                       self._first_endpoint.y - curr_pos.y])
        to_second_endpoints = np.array([self._second_endpoint.x - curr_pos.x,
                                        self._second_endpoint.y - curr_pos.y])

        from_first_endpoints = np.array([target.x - self._first_endpoint.x,
                                        target.y - self._first_endpoint.y])
        from_second_endpoints = np.array([target.x - self._second_endpoint.x,
                                         target.y - self._second_endpoint.y])

        if previous_step is None:
            dist1 = np.linalg.norm(from_first_endpoints) + np.linalg.norm(to_first_endpoints)
            dist2 = np.linalg.norm(from_second_endpoints) + np.linalg.norm(to_second_endpoints)
            if dist1 < dist2:
                return self._first_endpoint
            return self._second_endpoint
        if np.dot(from_first_endpoints, previous_step) > np.dot(from_second_endpoints, previous_step):
            return self._first_endpoint
        return self._second_endpoint
