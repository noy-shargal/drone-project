import matplotlib.pyplot as plt
from descartes import PolygonPatch


class MapDrawer:

    def __init__(self, min_x, max_x, min_y, max_y):

        self._fig, self._ax = plt.subplots()
        self._ax.set_xlim(min_x, max_x)
        self._ax.set_ylim(min_y, max_y)
        self._ax.set_xlabel('X - Axis')
        self._ax.set_ylabel('Y - Axis')
        self._source = None
        self._destination = None
        self._polygons = None

    def add_polygons(self, polygons):

        self._polygons = polygons
        for poly_id,poly in polygons.items():
            if poly_id == 'start':
                self._ax.add_patch(plt.Polygon(poly.exterior.coords, color='green'))
            elif poly_id == 'destination':
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

