import numpy as np
from typing import Tuple, List, Dict
import math
import os

from shapely.geometry import Polygon, Point
from config import current_config, InfiniteRepulsionConfig


class LatticeMap:

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, data_type=current_config.data_type):
        self._min_x = min_x
        self._max_x = max_x
        self._min_y = min_y
        self._max_y = max_y
        self._unit_size = current_config.grid_size

        self._size_x = int((max_x - min_x) / current_config.grid_size)
        self._size_y = int((max_y - min_y) / current_config.grid_size)
        self._data_type = data_type

        self._map = np.ndarray((self._size_x, self._size_y), dtype=data_type)

    def value(self, x: float, y: float):
        i, j = self.coord_to_index(x, y)
        return self._map[i][j]

    def get_local_values(self, x, y):
        i, j = self.coord_to_index(x, y)
        output = dict()
        # Top
        for x_iterator in range(-(current_config.window_size-1)//2, current_config.window_size//2+1, 1):
            position = (i + x_iterator, j - current_config.window_size//2)
            output[position] = self._map[position[0], position[1]]
        # Bottom
        for x_iterator in range(-(current_config.window_size-1)//2, current_config.window_size//2+1, 1):
            position = (i + x_iterator, j + current_config.window_size//2)
            output[position] = self._map[position[0], position[1]]
        # Left
        for y_iterator in range(-(current_config.window_size-1)//2 + 1, current_config.window_size//2, 1):
            position = (i - current_config.window_size//2, j + y_iterator)
            output[position] = self._map[position[0], position[1]]
        # Right
        for y_iterator in range(-(current_config.window_size-1)//2 + 1, current_config.window_size//2, 1):
            position = (i + current_config.window_size//2, j + y_iterator)
            output[position] = self._map[position[0], position[1]]
        return output

    def index_to_coord(self, i, j):
        x = self._min_x + (i + 0.5) * self._unit_size
        y = self._min_y + (j + 0.5) * self._unit_size
        return x, y

    def coord_to_index(self, x: float, y: float):
        return int((x - self._min_x) / self._unit_size), int((y - self._min_y) / self._unit_size)

    def _l1_dist(self, cell1: Tuple, cell2: Tuple):
        return math.fabs(cell1[0] - cell2[0]) + math.fabs(cell1[1] - cell2[1])

    def _dist(self, cell1: Tuple, cell2: Tuple):
        return math.sqrt((cell1[0] - cell2[0]) ** 2 + (cell1[1] - cell2[1]) ** 2)


class AttractionMap(LatticeMap):

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float):
        super(AttractionMap, self).__init__(min_x, max_x, min_y, max_y)
        self._start = current_config.start_position
        self._goal = current_config.end_position

        self._start_i_j = self.coord_to_index(*self._start)
        self._goal_i_j = self.coord_to_index(*self._goal)
        self._d = current_config.d
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
    _SAVED_MAP_PATH = "repulsion_map.npy"

    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, polygons_dict: Dict):
        super(RepulsionMap, self).__init__(min_x, max_x, min_y, max_y)

        self._obstacles = polygons_dict
        self._q_star = current_config.q_star
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

    def infinite_repulsion_by_distance(self, distance):
        if distance == 0.0:
            return np.float32(np.inf)

        if distance > self._q_star:
            return 0.0

        return 0.5 * (1.0 / distance - 1.0 / self._q_star) ** 2

    def finite_repulsion_by_distance(self, distance):
        if distance > self._q_star:
            return 0.0

        return ((self._q_star - distance)/self._q_star)**current_config.eta

    def _repulsion_value(self, i, j):
        total_repulsion = 0.0
        for obs in self._obstacles.values():
            total_repulsion += self._repulsion_from_obstacle(obs, i, j)
        return total_repulsion

    def repulsion_by_distance(self, distance):
        if type(current_config) is InfiniteRepulsionConfig:
            return self.infinite_repulsion_by_distance(distance)
        return self.finite_repulsion_by_distance(distance)

    def _init_lattice(self):
        if not current_config.use_obstacles_map:
            self._map = np.zeros((self._size_x, self._size_y), dtype=self._data_type)
            return
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
        return 'unit_size=' + str(self._unit_size) + 'q_star=' + str(self._q_star) + self._SAVED_MAP_PATH

    def update_map(self, x, y):
        i, j = self.coord_to_index(x, y)
        new_vals = np.ones((3, 3)) * np.inf
        self._map[i - 1:i + 2, j - 1:j + 2] = new_vals


class ObstacleMap(LatticeMap):
    _SAVED_MAP_PATH = "obstacles_map.npy"

    def __init__(self, obstacles_dict, min_x, max_x, min_y, max_y, data_type=np.int8):

        super(ObstacleMap, self).__init__(min_x, max_x, min_y, max_y, data_type)
        self._obstacles = obstacles_dict
        self._init_map()

    def _obstacle_value(self, i, j):
        x, y = self.index_to_coord(i, j)
        p = Point(x, y)
        return any([obs.contains(p) or obs.exterior.distance(p) < self._unit_size for obs in self._obstacles.values()])

    def _init_map(self):
        if not current_config.use_obstacles_map:
            self._map = np.zeros((self._size_x, self._size_y), dtype=self._data_type)
            return
        if os.path.isfile(self._calculate_save_path()):
            self._load_map()
            return
        for i in range(self._size_x):
            for j in range(self._size_y):
                self._map[i, j] = self._obstacle_value(i, j)

    def update_map(self, x, y, value):
        i, j = self.coord_to_index(x, y)
        self._map[i, j] = value

    def new_obstacle(self, x, y):
        i, j = self.coord_to_index(x, y)
        return not self._map[i, j]

    def _calculate_save_path(self):
        return 'unit_size=' + str(self._unit_size) + self._SAVED_MAP_PATH

    def _load_map(self):
        self._map = np.load(self._calculate_save_path())
