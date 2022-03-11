from Vertex import Vertex
from queue import PriorityQueue
from datetime import datetime

class Dijkstra:

    def __init__(self):

        self._path = list()
        self._Q = PriorityQueue()
        self._num_of_vertices_visited = 0

    def _update_path(self, destination):
        curr_vertex = destination
        while curr_vertex is not None:
            self._path.append(curr_vertex)
            curr_vertex = curr_vertex.get_prev_vertex()

    def search(self, source: Vertex, destination: Vertex):
        self._num_of_vertices_visited = 0
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S.%f")
        print("Dijkstra start Time =", current_time)
        source.set_distance(0)
        self._Q.put(source)

        vertices_dict = dict()

        while not self._Q.empty():
            vertex = self._Q.get()
            if vertex.is_settled():
                continue
            vertex.settle()
            self._num_of_vertices_visited += 1
            print(str(vertex.point().xy))
            if str(vertex.point().xy)  in vertices_dict.keys():
                vertices_dict[str(vertex.point().xy)] +=1
            else:
                vertices_dict[str(vertex.point().xy)] = 1

            if vertex == destination:
                break
            edges = vertex.get_edges()
            for edge in edges:
                next_vertex = edge.get_other_vertex(vertex)
                if next_vertex.is_settled():
                    continue
                edge_distance = edge.get_distance()
                new_distance = vertex.get_distance() + edge_distance
                if new_distance < next_vertex.get_distance():
                    next_vertex.set_prev_vertex(vertex)
                    next_vertex.set_distance(new_distance)
                    self._Q.put(next_vertex)

        self._update_path(destination)
        later = datetime.now()
        finish_time = later.strftime("%H:%M:%S.%f")
        print("Dijkstra finish Time =", finish_time)

        return True

    def get_num_of_vertices_visited(self):
        return self._num_of_vertices_visited

    def get_path(self):
        return self._path
