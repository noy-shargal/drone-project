import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, box, LineString



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
            self._ax.add_patch(plt.Line2D([p1.x, p2.x], [p1.y, p2.y], color='#2f33a3', linewidth=1, alpha=0.5))

    def set_source(self, src: Point):

        self._source = src
        circle = plt.Circle((src.x, src.y),radius=10, color='green')
        self._ax.add_artist(circle)

    def set_destination(self, dst: Point):
        self._destination = dst
        circle = plt.Circle((dst.x, dst.y),radius=10, color='red', edgecolor='black')
        self._ax.add_artist(circle)

    def set_point(self, dst: Point):
        self._destination = dst
        circle = plt.Circle((dst.x, dst.y),radius=10, color='black')
        self._ax.add_artist(circle)

    def set_path(self, path):

        for i in range(len(path) -1):
            p1 = path[i].point()
            p2 = path[i+1].point()

            self._ax.add_patch(plt.Line2D([p1.x, p2.x], [p1.y, p2.y], color='green', linewidth=2))

    def set_real_path(self, path):

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            self._ax.add_patch(plt.Line2D([p1.x, p2.x], [p1.y, p2.y], color='red', linewidth=2))

    @staticmethod
    def show():
        plt.show()

