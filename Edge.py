from shapely.geometry import Point
import Vertex


class Edge:

    def __init__(self, one: Vertex, two: Vertex):
        self.one = one
        self.two = two
        self.distance = one.point().distance(two.point())

