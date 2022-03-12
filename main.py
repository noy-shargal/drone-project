import math
import pickle
import time

from queue import PriorityQueue

import numpy as np

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

    def getPointInRealWorldCoords(self, x_drone, y_drone):
        pose = self.getPose()
        theta = pose.orientation.z_rad
        x_world = x_drone * np.cos(theta) - y_drone* np.sin(theta) + pose.pos.x_m
        y_world = x_drone * np.sin(theta) + y_drone * np.cos(theta) + pose.pos.y_m
        return Point(x_world, y_world)

    def getPointInPolarCoords(self, rel_x, rel_y):
        r = math.sqrt(rel_x * rel_x + rel_y * rel_y)
        theta = math.atan2(rel_y, rel_x)
        return r, theta


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


class NewObstaclesBuilder:
    def __init__(self, obs: Obstacles):
        self._obstacles = obs
        self._points = list()

    def report_point(self, point: Point, drone_position: Point):
        if point not in self._points:
            self._points.append(point)
        if len(self._points) >= 2:
            point1 = self._points[0]
            point2 = self._points[1]
            point3_x = (point1.x + point2.x) / 2.0
            point3_y = None
            avg_y = (point1.y + point2.y) / 2.0
            if avg_y > drone_position.y:
                point3_y = avg_y + 50.0
            else:
                point3_y = avg_y - 50.0
            point3 = Point(point3_x, point3_y)
            self._obstacles.add_new_obstacle(point1, point2, point3)


if __name__ == "__main__":

    read_from_json = True
    height = -50
    obs = Obstacles()
    source = Point(-1250.0, -830.0)
    destination = Point(0.0, -820.0)

    if read_from_json == True:
        obs.set_destination_point(destination)
        obs.load_from_json()
    else:
        obs.read_csv()
        obs.build_visibility_graph()
        obs.save_to_json()

    md = MapDrawer()

    obs.set_source(source)
    obs.set_destination(destination)

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
    client.setAtPosition(source.x, source.y, height)
    time.sleep(3)

    prev_point_num = 0
    point_num = 1
    need_fly_command = True
    real_path = list()
    new_obs_builder = NewObstaclesBuilder(obs)

    while True:
        lidar_data = client.getLidarData()
        goal = Position()

        p = path[point_num].point()
        goal.x_m, goal.y_m, goal.z_m = p.x, p.y, height
        pose = client.getPose()
        print(str(pose.orientation.x_rad) + " , " + str(pose.orientation.y_rad) + " , " + str(pose.orientation.z_rad))
        time.sleep(2)
        if need_fly_command:
            client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, 6)
            need_fly_command = False
            print("Flying to point number: " + str(point_num))

        if reached_goal_2D(client.getPose().pos, goal):
            print("Reached goal number : " + str(point_num))
            prev_point_num = point_num
            point_num += 1
            need_fly_command = True
            pose = client.getPose()
            real_path.append(position_to_point(pose.pos))
            if point_num == len(path):
                break


    md.set_real_path(real_path)
    md.show()
    y = 4
