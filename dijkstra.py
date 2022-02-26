from Vertex import Vertex
from Edge import Edge
from queue import PriorityQueue

class Dijkstra:

    def __init__(self, source: Vertex, destination: Vertex):
        self._source = source
        self._destination = destination
        self._path = list()
        self._Q = PriorityQueue()




