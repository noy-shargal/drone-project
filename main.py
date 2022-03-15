import math
import pickle
import time

from queue import PriorityQueue

from Dijkstra import Dijkstra
from MapDrawer import MapDrawer
from DroneClient import DroneClient
from DroneTypes import Position
from Obstacles import Obstacles, PersistentGraph
from shapely.geometry import Polygon, Point, box, mapping


class MyDroneClient(DroneClient):

    def __init__(self):
        super().__init__()
        self.stopped = False
        self._state = "GO_TOP_POINT"


    def foolowWall(self):
        self._state = "FOLLOW_OBSTACLE_WALL"

    def goToPoint(self):
        self._state = "GO_TO_POINT"

    def getState(self):
        return self._state


    def toCoords(self, pose):

        x = pose.pos.x_m
        y = pose.pos.y_m
        z = pose.pos.z_m
        return x, y, z

    def stop(self):
        if self.stopped == True:
            return

        self.stopped = True
        pose_res = self.getPose()
        x, y, z = self.toCoords(pose_res)
        self.setAtPosition(x, y, z)

    def resume(self):
        super().flyToPosition(self.target_params)

    def flyToPosition(self, x: float, y: float, z: float, v: float):
        self.target_params = x, y, z, v
        super().flyToPosition(x, y, z, v)

    def senseObstacle(self):
        lidar_data = self.getLidarData()
        if lidar_data.points == [0.0]:
            return False, [0.0]
        return True, lidar_data.points

    def getPointInRealWorldCoords(self,rel_x, rel_y):
        pose = self.getPose()
        x_translate  = pose.pos.x_m
        y_translate = pose.pos.y_m
        x_orientation = pose.orientation.x_rad
        y_orientation = pose.orientation.y_rad

        







        point = Point(abs_x + rel_x, abs_y, rel_y)
        return point

from Vertex import Vertex


# def test_pq():
#     Q = PriorityQueue()
#
#     v1 = Vertex(Point(0.0, 0.0))
#     v2 = Vertex(Point(0.0, 0.0))
#     v3 = Vertex(Point(0.0, 0.0))
#     v4 = Vertex(Point(0.0, 0.0))
#     v5 = Vertex(Point(0.0, 0.0))
#
#     v1.set_distance(1.0)
#     v2.set_distance(22.0)
#     v3.set_distance(3.0)
#     v4.set_distance(4.0)
#     v5.set_distance(5.0)
#
#     Q.put(v2)
#     Q.put(v4)
#     Q.put(v5)
#     Q.put(v1)
#     Q.put(v3)
#
#     v2.set_distance(0.5)
#     Q.get()
#     Q.put(v2)
#     vv1 = Q.get()
#     vv2 = Q.get()
#     vv3 = Q.get()
#     vv4 = Q.get()
#     vv5 = Q.get()

    x = 1

def position_to_point(pos: Position):
    x = pos.x_m
    y = pos.y_m
    return Point(x, y)

def reached_goal_2D(curr: Position, goal: Position):
    diff_x = curr.x_m - goal.x_m
    diff_y = curr.y_m - goal.y_m
    dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)

    if dist < 5.0:
        return True
    return False


if __name__ == "__main__":

    obs = Obstacles()
    obs.read_csv()

    loaded_graph = None
    # file_to_read = open("graph.pickle", "rb")
    # loaded_graph = pickle.load(file_to_read)

    obs.build_visibility_graph(loaded_graph)
    md = MapDrawer()
    source = Point(-1250.0, -830.0)
    destination = Point(0.0, -600.0)
    obs.set_source(source)
    obs.set_destination(destination)

    ########################################################################################
    graph = PersistentGraph()
    # graph.set(obs.get_edges(), obs.get_vertices())
    # file_to_store = open("full_graph.pickle", "wb")
    # pickle.dump(graph, file_to_store)
    # file_to_store.close()
    ########################################################################################

    md.set_source(source)
    md.set_destination(destination)
    polygons = obs.get_polygons()
    md.add_polygons(polygons)
    print("Number of polygons is: " + str(len(polygons)))
    edges = obs.get_edges()
    md.add_edges(edges)
    src = obs.get_source_vertex()
    dst = obs.get_destination_vertex()
    dij = Dijkstra()
    found = dij.search(src, dst)
    path = None
    if found:
        path = dij.get_path()
        md.set_path(path)
    print("Total Distance: " + str(dst.get_distance()))
    print("Number Of vertices visited :" + str(dij.get_num_of_vertices_visited()))
    md.show()

    client = MyDroneClient()
    client.connect()
    print(client.isConnected())
    time.sleep(4)
    path.reverse()
    client.setAtPosition(source.x, source.y, -100)
    time.sleep(3)

    prev_point_num = 0
    point_num = 1
    need_fly_command = True
    real_path = list()
    while True:
        lidar_data = client.getLidarData()
        goal = Position()

        p = path[point_num].point()
        goal.x_m, goal.y_m, goal.z_m = p.x, p.y, -100.0
        if need_fly_command:
            client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, 5)
            need_fly_command = False
            print("Flying to point number: " + str(point_num))

        if reached_goal_2D(client.getPose().pos, goal):
            print("Reached goal number : " + str(point_num))
            prev_point_num = point_num
            point_num += 1
            need_fly_command = True
            pos = client.getPose().pos
            real_path.append(position_to_point(pos))
            if point_num == len(path):
                break

        sensing_obstacle, points_list = client.senseObstacle()
        if sensing_obstacle:
            print("Lidar points : " + str(points_list))

        position = client.getPose().pos


        if obs.is_point_in_obstacle()


        md.set_real_path(real_path)
        md.show()



