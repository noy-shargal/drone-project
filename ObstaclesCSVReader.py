import csv
import itertools
from copy import deepcopy

from shapely.geometry import Polygon, Point, box, LineString


class ObstaclesCSVReader:

    def __init(self, file_name='obstacles_100m_above_sea_level.csv'):

        self._points_map = dict()  # obs_id -> list() of points
        self._polygons_map = dict  # obs_id -> list() of polygons
        self._read_csv(file_name)
        self._min_x, self._max_x, self._min_y, self._max_y = self._calc_boundaries()

        self._is_map_inited = True

    def _calc_boundaries(self):

        all_points = list(itertools.chain(*self._points_map.values()))
        all_x = [p.x for p in all_points]
        all_y = [p.y for p in all_points]
        max_x = max(all_x)
        min_x = min(all_x)
        max_y = max(all_y)
        min_y = min(all_y)
        return min_x, max_x, min_y, max_y

    def _read_csv(self, file_name='obstacles_100m_above_sea_level.csv'):
        with open(file_name, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            next(reader, None)  # skip the headers
            for row in reader:
                obs_id = row[4]
                obs_list = self.get_obstacle_points_list(obs_id)
                point = self._get_coord(row)
                obs_list.append(point)

            for polygon_id, points in self._points_map.items():
                polygon = Polygon(points)
                self._polygons_map[polygon_id] = polygon

    def get_obstacle_points_list(self, id):

        if id not in self._points_map.keys():
            self._points_map[id] = list()
        return self._points_map[id]

    @staticmethod
    def _get_coord(row):
        x_str = row[1]
        y_str = row[2]
        x = float(x_str)
        y = float(y_str)
        point = Point(x, y)
        return point

    @property
    def polygons_map(self):
        return deepcopy(self._polygons_map)

    def get_boundaries(self):
        padding = 100.0
        return self._min_x - padding, self._max_x + padding, self._min_y - padding, self._max_y + padding


