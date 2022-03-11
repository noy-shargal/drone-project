import numpy as np
from typing import Tuple, List, Dict
import math
import os

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
        i, j = self.coord_to_index(x, y)
        return self._map[i][j]

    def get_local_values(self, x, y):
        i, j = self.coord_to_index(x, y)
        return self._map[i - 1:i + 2, j - 1:j + 2]

    def index_to_coord(self, i, j):
        x = self._min_x + (i + 0.5) * self._unit_size
        y = self._min_y + (j + 0.5) * self._unit_size
        return x, y

    def coord_to_index(self, x: float, y: float):
        return int((x - self._min_x) / self._unit_size), int((y - self._min_y) / self._unit_size)

    def _dist(self, cell1: Tuple, cell2: Tuple):
        return math.fabs(cell1[0] - cell2[0]) + math.fabs(cell1[1] - cell2[1])


class AttractionMap(LatticeMap):

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, unit_size: float, type, start: Tuple,
                 goal: Tuple, d: float):
        super(AttractionMap, self).__init__(min_x, max_x, min_y, max_y, unit_size, type)
        self._start = start
        self._goal = goal

        self._start_i_j = self.coord_to_index(*start)
        self._goal_i_j = self.coord_to_index(*goal)
        self._d = d
        self._min_value = np.float(np.inf)
        self._max_value = np.float(-np.inf)
        self._init_lattice()

    def _attraction_value(self, i, j):
        dist = self._dist(self._goal_i_j, (i, j))
        if dist >= self._d:
            return dist * self._d - 0.5 * self._d ** 2
        return (1 / 2.0) * dist ** 2

    def _init_lattice(self):

        for i in range(self._size_x):
            for j in range(self._size_y):
                self._map[i, j] = self._attraction_value(i, j)
                self._max_value = max(self._max_value, self._attraction_value(i, j))
                self._min_value = min(self._min_value, self._attraction_value(i, j))

    @property
    def min_value(self):
        return self._min_value

    @property
    def max_value(self):
        return self._max_value


class RepulsionMap(LatticeMap):
    _SAVED_MAP_PATH = path = "repulsion_map.npy"

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, unit_size: float, type,
                 polygons_dict: Dict, q_star: float):
        super(RepulsionMap, self).__init__(min_x, max_x, min_y, max_y, unit_size, type)

        self._obstacles = polygons_dict
        self._q_star = q_star
        self._init_lattice()

    def _repulsion_from_obstacle(self, obs: Polygon, i, j):
        x, y = self.index_to_coord(i, j)
        point = Point(x, y)
        distance = obs.exterior.distance(point)

        if obs.contains(point) or distance == 0.0:
            return np.float32(np.inf)

        if distance > self._q_star:
            return 0.0

        return 0.5 * (1.0 / distance - 1.0 / self._q_star) ** 2

    def _repulsion_value(self, i, j):
        total_repulsion = 0.0
        for obs in self._obstacles.values():
            total_repulsion += self._repulsion_from_obstacle(obs, i, j)
        return total_repulsion

    def _init_lattice(self):
        if os.path.isfile(self._calculate_save_path()):
            self._load_map()
            return
        for i in range(self._size_x):
            for j in range(self._size_y):
                self._map[i, j] = self._repulsion_value(i, j)
        self._save_map()

    def _save_map(self):
        np.save(self._calculate_save_path(), self._map)

    def _load_map(self):
        self._map = np.load(self._calculate_save_path())

    def _calculate_save_path(self):
        return str(self._unit_size) + 'm_' + self._SAVED_MAP_PATH


class ObstacleMap(LatticeMap):

    def __init__(self, obstacles_dict, min_x, max_x, min_y, max_y, unit_size, dtype):

        super(ObstacleMap, self).__init__(min_x, max_x, min_y, max_y, unit_size, dtype)
        self._obstacles = obstacles_dict
        self._init_map()

    def _obstacle_value(self, i, j):
        x, y = self.index_to_coord(i, j)
        p = Point(x, y)
        return any([obs.contains(p) for obs in self._obstacles.values()])

    def _init_map(self):
        for i in range(self._size_x):
            for j in range(self._size_y):
                self._map[i, j] = self._obstacle_value(i, j)
