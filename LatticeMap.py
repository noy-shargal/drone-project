import numpy as np
from typing import Tuple, List
import math
from shapely.geometry import Polygon, Point


class LatticeMap:

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, unit_size: float, type):
        self._min_x = min_x
        self._max_x = max_x
        self._min_y = min_y
        self._max_y = max_y
        self._unit_size = unit_size

        self._size_x = int((max_x - min_x) / unit_size)
        self._size_y = int((max_y - min_y) / unit_size)

        self._map = np.ndarray((self._size_x, self._size_y), dtype=type)

    def value(self, x: float, y: float):
        x, y = self._coord_to_index(x, y)
        return self._map[x][y]

    def _coord_to_index(self, x: float, y: float):
        return (x - self._min_x) / self._unit_size, (y - self._min_y) / self._unit_size

    def _dist(self, cell1: Tuple, cell2: Tuple):
        return math.fabs(cell1[0] - cell2[0]) + math.fabs(cell1[1] - cell2[1])


class AttractionMap(LatticeMap):

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, unit_size: float, type, start: Tuple,
                 goal: Tuple, d: float, k: float):
        super(AttractionMap, self).__init__(min_x, max_x, min_y, max_y, unit_size, type)
        self._start = start
        self._goal = goal

        self._start_i_j = self._coord_to_index(*start)
        self._goal_i_j = self._coord_to_index(*goal)
        self._d = d
        self._k = k

        self._init_lattice()

    def _attraction_value(self, i, j):
        dist = self._dist(self._goal_i_j, (i, j))
        if dist >= self._d:
            return self._k * dist * self._d - 0.5 * self._k * self._d ** 2
        return (self._k / 2.0) / dist ** 2

    def _init_lattice(self):
        for i in range(self._size_x):
            for j in range(self._size_y):
                self._map[i][j] = self._attraction_value(i, j)


class RepulsionMap(LatticeMap):

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, unit_size: float, type,
                 polygons_list: List, q_star: float, s: float):
        super(RepulsionMap, self).__init__(min_x, max_x, min_y, max_y, unit_size, type)

        self._obstacles = polygons_list
        self._q_star = q_star
        self._s = s

    def _index_to_coord(self, i, j):
        x = self._min_x + (i + 0.5) * self._unit_size
        y = self._min_y + (j + 0.5) * self._unit_size
        return x, y

    def _repulsion_from_obstacle(self, obs: Polygon, i, j):
        x, y = self._index_to_coord(i, j)
        point = Point(x, y)
        distance = obs.exterior.distance(point)
        if distance > self._q_star:
            return 0.0
        return 0.5 * self._s * (1.0 / distance - 1.0 / self._q_star) ** 2

    def _repulsion_value(self, i, j):
        total_repulsion = 0.0
        for obs in self._obstacles:
            total_repulsion += self._repulsion_from_obstacle(obs, i, j)
        return total_repulsion

    def _init_lattice(self):
        for i in range(self._size_x):
            for j in range(self._size_y):
                self._map[i][j] = self._repulsion_value(i, j)


# class ObstacleMap (LatticeMap):
#
#     def __init__(self, obstacles_list):
#         super(ObstacleMap, self).__init__(min_x, max_x, min_y, max_y, unit_size, type)
#
#         self._obstacles = polygons_list
#         self._q_star = q_star
#         self._s = s

