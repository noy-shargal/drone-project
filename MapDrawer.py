import matplotlib.pyplot as plt
from descartes import PolygonPatch
from shapely.geometry import Polygon


class MapDrawer:

    def __init__(self):

        self._fig, self._ax = plt.subplots()
        # Visualization - set xlim and ylim
        # border = Polygon([(-1600.0, -1300.0), (-1600.0, 100.0), (-300.0, 100.0), (-300.0, -1300.0)])  # Large square

        self._ax.set_xlim(-1700.0, 300.0)
        self._ax.set_ylim(-1300.0, 200.0)
        # self._ax.add_artist(PolygonPatch(border))

    def add_polygons(self, polygons):

        for poly in polygons:
            self._ax.add_artist(PolygonPatch(poly))


    def show(self):
        plt.show()
