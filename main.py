import math
import numpy as np
import time

import DroneTypes
from Dijkstra import Dijkstra
from MapDrawer import MapDrawer
from DroneClient import DroneClient
from DroneTypes import Position
from Obstacles import Obstacles, PersistentGraph
from shapely.geometry import Polygon, Point, box, mapping

from sim_config import sim_config


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
        if self.stopped:
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

    def getLidarData(self):
        point_cloud = DroneTypes.PointCloud()
        lidar_data = self.client.getLidarData()

        point_cloud.points = lidar_data.point_cloud

        return point_cloud, self.getPose()

    def senseObstacle(self):
        lidar_data, pose = self.getLidarData()
        if lidar_data.points == [0.0]:
            return False, [0.0], pose
        return True, lidar_data.points, pose

    def getPointInRealWorldCoords(self, x_drone, y_drone, pose):
        theta = pose.orientation.z_rad
        x_world = x_drone * np.cos(theta) - y_drone * np.sin(theta) + pose.pos.x_m
        y_world = x_drone * np.sin(theta) + y_drone * np.cos(theta) + pose.pos.y_m
        return x_world, y_world

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


if __name__ == "__main__":

    config = sim_config()
    obs = Obstacles()
    obs.set_destination_point(config.destination)

    if config.load_from_json:
        obs.load_from_json()
    else:
        obs.read_csv()
        obs.build_visibility_graph()
    obs.set_source(config.source)
    obs.set_destination(config.destination)
    ################### MAP DRAWER ###########################################
    md = MapDrawer()
    md.set_source(config.source)
    md.set_destination(config.destination)
    polygons = obs.get_polygons()
    md.add_polygons(polygons)
    print("Number of polygons is: " + str(len(polygons)))
    edges = obs.get_edges()
    md.add_edges(edges)
    ###########################################################################

    ################### A STAR ################################################

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

    ###########################################################################

    client = MyDroneClient()
    client.connect()
    print(client.isConnected())
    time.sleep(4)
    path.reverse()
    client.setAtPosition(config.source.x, config.source.y, config.height)
    time.sleep(3)

    prev_point_num = 0
    point_num = 1
    need_fly_command = True
    real_path = list()

    while True:

        lidar_data = client.getLidarData()
        goal = Position()
        p = path[point_num].point()
        goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
        if need_fly_command:
            client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.velocity)
            need_fly_command = False
            print("Flying to point number: " + str(point_num))

        if client.reached_goal_2D(client.getPose().pos, goal):
            print("Reached goal number : " + str(point_num))
            prev_point_num = point_num
            point_num += 1
            need_fly_command = True
            pos = client.getPose().pos
            real_path.append(client.position_to_point(pos))
            if point_num == len(path):
                print("Reached destination at ("+str(client.getPose().pos.x_m)+", "+str(client.getPose().pos.y_m)+") ")
                break

        # sensing_obstacle, points_list = client.senseObstacle()
        # if sensing_obstacle:
        #     print("Lidar points : " + str(points_list))
        #
        # position = client.getPose().pos


        # if obs.is_point_in_obstacle()
        #
        #
        # md.set_real_path(real_path)
        # md.show()



