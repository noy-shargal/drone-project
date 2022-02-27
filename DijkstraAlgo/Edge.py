from DijkstraAlgo import Vertex


class Edge:

    def __init__(self, one: Vertex, two: Vertex):
        self.one = one
        self.two = two
        self.distance = one.point().distance(two.point())

    def get_other_vertex(self, vertex: Vertex):

        if self.one == vertex:
            return self.two
        return self.one

    def get_distance(self):
        return self.distance



