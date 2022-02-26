import matplotlib.pyplot as plt
from descartes import PolygonPatch


class MapDrawer:

    def __init__(self):

        self._fig, self._ax = plt.subplots()
        self._ax.set_xlim(300, -1700.0)
        self._ax.set_ylim(-1300.0, 200.0)
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

    def add_edges(self, edges):

        for edge in edges:
            p1, p2 = edge.one.point(), edge.two.point()
            self._ax.add_patch(plt.Line2D([p1.x, p2.x], [p1.y, p2.y], color='#2f33a3', linewidth=1))

    def set_source(self, polygon):

        self._source = polygon
        self._ax.add_artist(PolygonPatch(polygon, fc='red') )

    def set_destination(self, polygon, ):
        self._destination = polygon
        self._ax.add_artist(PolygonPatch(polygon, fc='green'))

    @staticmethod
    def show():
        plt.show()

