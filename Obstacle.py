import numpy as np
from shapely.geometry import LineString, Point


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
