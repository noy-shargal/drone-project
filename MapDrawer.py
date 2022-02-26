import matplotlib.pyplot as plt
from descartes import PolygonPatch
from shapely.geometry import Polygon,Point,box, mapping
# import pyvisgraph as vg
# from pyvisgraph.graph import Graph, Edge
# from pyvisgraph.graph import Point as VGPoint


class MapDrawer:

    def __init__(self):

        self._fig, self._ax = plt.subplots()

        # Visualization - set xlim and ylim
        # border = Polygon([(-1600.0, -1300.0), (-1600.0, 100.0), (-300.0, 100.0), (-300.0, -1300.0)])  # Large square
        #
        self._ax.set_xlim(300, -1700.0)
        self._ax.set_ylim(-1300.0, 200.0)

        self._ax.set_xlabel('X - Axis')
        self._ax.set_ylabel('Y - Axis')


        self._source = None
        self._destination = None
        self._polygons = None
        # self._ax.add_artist(PolygonPatch(border))

    def add_polygons(self, polygons):

        self._polygons = polygons
        for poly in polygons:
            self._ax.add_artist(PolygonPatch(poly))

    def set_source(self, polygon):
        self.source = polygon
        self._ax.add_artist(PolygonPatch(polygon, fc='red') )

    def set_destination(self, polygon, ):
        self.destination = polygon
        self._ax.add_artist(PolygonPatch(polygon, fc='green'))

    def show(self):
        plt.show()

    def show_visibilty_graph(self):
        pass