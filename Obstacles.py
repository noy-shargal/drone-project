import csv
from shapely.geometry import Polygon,Point,box, mapping


class Obstacles:

    def __init__(self):
        self._points_map = dict()
        self._polygons_map = dict()
        self._is_map_inited = False

    def get_obstacle_points_list(self, id):

        if id not in self._points_map.keys():
            self._points_map[id] = list()
        return self._points_map[id]


    def read_csv(self):
        with open('obstacles_100m_above_sea_level.csv', newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            next(reader, None)  # skip the headers
            for row in reader:
                obs_id = row[4]
                obs_list = self.get_obstacle_points_list(obs_id)
                x_str = row[1]
                y_str = row[2]

                x = float(x_str)
                y = float(y_str)
                point = Point(x,y)
                obs_list.append(point)

            for polygon_id, points in self._points_map.items():
                polygon = Polygon(points)
                self._polygons_map[polygon_id] = polygon.convex_hull

            self._is_map_inited = True

    def is_position_blocked(self, x, y, z):

        key = x, y, z
        return  self._points_map[key] is not None

    def get_polygons(self):

        return self._polygons_map.values()