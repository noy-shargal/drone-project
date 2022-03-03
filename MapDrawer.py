import matplotlib.pyplot as plt
import numpy as np

from  LatticeMap import AttractionMap




class MapDrawer:

    def __init__(self, min_x, max_x, min_y, max_y):

        self._fig, self._ax = plt.subplots()
        self._min_x = min_x
        self._max_x = max_x
        self._min_y = min_y
        self._max_y = max_y

        self._ax.set_xlim(min_x, max_x)
        self._ax.set_ylim(min_y, max_y)
        self._ax.set_xlabel('X - Axis')
        self._ax.set_ylabel('Y - Axis')
        self._source = None
        self._destination = None
        self._polygons = None

    def _normalize_balue_to_grey_scale(self, value, min_value, max_value):
        ret_value = 255*value /(max_value - min_value)
        return ret_value

    def paint_attraction_map(self, attraction_map: AttractionMap):
        min_value = attraction_map.min_value
        max_value = attraction_map.max_value
        for x in range(int(self._min_x), int(self._max_x), 3):
            for y in range(int(self._min_y), int(self._max_y)):
                value  = attraction_map.value(x,y)
                norm_value = self._normalize_balue_to_grey_scale(value, min_value, max_value)
                hex_norm_value = hex(int(norm_value))
                color = '#'+str(hex_norm_value)+str(hex_norm_value)+str(hex_norm_value)
               # self._ax.add_patch(plt.Line2D([x, x+1], [y, y+1] , , linewidth=2))

    def add_polygons(self, polygons):

        self._polygons = polygons
        for poly_id,poly in polygons.items():
            if poly_id == 'start':
                self._ax.add_patch(plt.Polygon(poly.exterior.coords, color='green'))
            elif poly_id == 'goal':
                self._ax.add_patch(plt.Polygon(poly.exterior.coords, color='red'))
            else:
                self._ax.add_patch(plt.Polygon(poly.exterior.coords, color='#c9871c'))

    # def set_source(self, polygon):
    #
    #     self._source = polygon
    #     self._ax.add_artist(PolygonPatch(polygon, fc='red') )
    #
    # def set_destination(self, polygon, ):
    #     self._destination = polygon
    #     self._ax.add_artist(PolygonPatch(polygon, fc='green'))

    def set_path(self, path):

        for i in range(len(path) -1):
            p1 = path[i]
            p2 = path[i+1]
            self._ax.add_patch(plt.Line2D([p1[0], p2[0]], [p1[1], p2[1]], color='green', linewidth=2))

    @staticmethod
    def show():
        plt.show()

