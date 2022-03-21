import math
from collections import OrderedDict

import numpy as np
import time

import DroneTypes
from Agent import Agent
from Dijkstra import Dijkstra
from MapDrawer import MapDrawer
from DroneClient import DroneClient
from DroneTypes import Position
from Obstacles import Obstacles, PersistentGraph
from shapely.geometry import Polygon, Point, box, mapping

from TangentBug import TangentBug
from Vertex import Vertex
from Config import Config
from utils import get_parallelogram_missing_point
from MyDroneClient import  MyDroneClient
# class MyDroneClient(DroneClient):
#
#     def __init__(self):
#         super().__init__()
#         self.stopped = False
#         self._state = "GO_TOP_POINT"
#
#     def foolowWall(self):
#         self._state = "FOLLOW_OBSTACLE_WALL"
#
#     def goToPoint(self):
#         self._state = "GO_TO_POINT"
#
#     def getState(self):
#         return self._state
#
#     def toCoords(self, pose):
#
#         x = pose.pos.x_m
#         y = pose.pos.y_m
#         z = pose.pos.z_m
#         return x, y, z
#
#     def stop(self):
#         if self.stopped:
#             return
#
#         self.stopped = True
#         pose_res = self.getPose()
#         x, y, z = self.toCoords(pose_res)
#         self.setAtPosition(x, y, z)
#
#     def resume(self):
#         super().flyToPosition(self.target_params)
#
#     def flyToPosition(self, x: float, y: float, z: float, v: float):
#         self.target_params = x, y, z, v
#         super().flyToPosition(x, y, z, v)
#
#     def getLidarData(self):
#         point_cloud = DroneTypes.PointCloud()
#         lidar_data = self.client.getLidarData('Lidar1')
#         # point_cloud.points = lidar_data.point_cloud
#         return lidar_data
#
#
#         # point_cloud = DroneTypes.PointCloud()
#         # lidar_data = self.client.getLidarData()
#         #
#         # point_cloud.points = lidar_data.point_cloud
#         #
#         # return point_cloud, self.getPose()
#
#     def senseObstacle(self):
#         #lidar_data, pose = self.getLidarData()
#         lidar_data  = self.getLidarData()
#         if lidar_data.point_cloud == [0.0]:
#             return False, [0.0], lidar_data.pose
#         return True, lidar_data.point_cloud, lidar_data.pose
#
#     def getPointInRealWorldCoords(self, x_drone, y_drone, pose):
#         theta = pose.orientation.z_rad
#         x_world = x_drone * np.cos(theta) - y_drone * np.sin(theta) + pose.pos.x_m
#         y_world = x_drone * np.sin(theta) + y_drone * np.cos(theta) + pose.pos.y_m
#         return x_world, y_world
#
#     def getPointInPolarCoords(self, rel_x, rel_y):
#         r = math.sqrt(rel_x * rel_x + rel_y * rel_y)
#         theta = math.atan2(rel_y, rel_x)
#         return r, theta
#
#
#     def position_to_point(self, pos: Position):
#         x = pos.x_m
#         y = pos.y_m
#         return Point(x, y)
#
#     def reached_goal_2D(self, curr: Position, goal: Position):
#         diff_x = curr.x_m - goal.x_m
#         diff_y = curr.y_m - goal.y_m
#         dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)
#
#         if dist < 5.0:
#             return True
#         return False


if __name__ == "__main__":

    drone_agent = Agent()
    drone_agent.connect_and_spawn()


    # config = Config()

    # obs = Obstacles()
    # obs.set_destination_point(config.destination)
    #
    # if config.load_from_json:
    #     obs.load_from_json()
    # else:
    #     obs.read_csv()
    #     obs.build_visibility_graph()
    # obs.set_source(config.source)
    # obs.set_destination(config.destination)
    # ################### MAP DRAWER ###########################################
    # #md = MapDrawer()
    # #md.set_source(config.source)
    # #md.set_destination(config.destination)
    # polygons = obs.get_polygons()
    # #md.add_polygons(polygons)
    # print("Number of polygons is: " + str(len(polygons)))
    # edges = obs.get_edges()
    # #md.add_edges(edges)
    # ###########################################################################
    #
    # ################### A STAR ################################################
    #
    # src = obs.get_source_vertex()
    # dst = obs.get_destination_vertex()
    # dij = Dijkstra()
    # found = dij.search(src, dst)
    # path = None
    # if found:
    #     path = dij.get_path()
    #     #md.set_path(path)
    # print("Total Distance: " + str(dst.get_distance()))
    # print("Number Of vertices visited :" + str(dij.get_num_of_vertices_visited()))
    # #md.show()

    ###########################################################################
    agent = Agent()
    try:
        agent.connect_and_spawn()
        agent.fly_to_destination()

        # client = MyDroneClient()
        # client.reset()
        # print ("Connecting.....")
        # client.connect()
        # print(client.isConnected())
        # time.sleep(4)
        # path.reverse()
        # client.setAtPosition(config.source.x, config.source.y, config.height)
        # time.sleep(5)
        # print ("Init position "+ str ([config.source.x, config.source.y, config.height]))
        #
        # prev_point_num = 0
        # point_num = 1
        # need_fly_command = True
        # real_path = list()
        #
        # tb = TangentBug()

        # DEBUG_ltg_count = 0
        # while True:
        #
        #     lidar_data = client.getLidarData()
        #     goal = Position()
        #     p = path[point_num].point()
        #     goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
        #
        #     new_obstacle_points = 0
        #
        #     if need_fly_command:
        #         client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.velocity)
        #         need_fly_command = False
        #         print("Flying to point number: " + str(point_num)+ str ([goal.x_m, goal.y_m, goal.z_m]))
        #
        #     if client.reached_goal_2D(client.getPose().pos, goal):
        #         print("Reached goal number : " + str(point_num))
        #         prev_point_num = point_num
        #         point_num += 1
        #         need_fly_command = True
        #         pos = client.getPose().pos
        #         real_path.append(client.position_to_point(pos))
        #         if point_num == len(path):
        #             print("Reached destination at ("+str(client.getPose().pos.x_m)+", "+str(client.getPose().pos.y_m)+") ")
        #             break
        #
        #     sensing_obstacle, points_list, pose = client.senseObstacle()
        #     if sensing_obstacle:
        #         # print ("sensed obstacle : "+str(points_list), str(pose))
        #         xw, yw = client.getPointInRealWorldCoords(points_list[0], points_list[1], client.getPose())
        #         # print("getPointInRealWorldCoords -> ", "("+str(xw) +", "+ str(yw)+ ")")
        #         # print ("Drone location : (", str(client.getPose().pos.x_m), ", "+str(client.getPose().pos.y_m)+")")
        #
        #         is_known_obs = obs.is_point_in_obstacles_map(Point(xw, yw))
        #         if not is_known_obs:
        #
        #
        #             tb.add_point(Point(points_list[0], points_list[1]), Point(xw, yw))
        #             if tb.get_num_of_points() > 5 and tb.is_way_blocked():
        #                 client.stop()
        #
        #                 curr_poss = Point(client.getPose().pos.x_m, client.getPose().pos.y_m)
        #                 tb.set_current_position(curr_poss)
        #                 next_goal = Point(goal.x_m, goal.y_m)
        #                 tb.set_target(next_goal)
        #                 tb.build_ltg()
        #                 sg = tb.build_sub_graph()
        #                 closest_point = sg.get_closet_point_to_target()
        #                 client.flyToPosition(closest_point.x , closest_point.y, config.height, config.ltf_velocity)
        #                 if sg.is_source_local_minima():
        #                     print("Reached Local Minima")
        #                     point, v_name = tb.get_closest_endpoint_to_target(curr_poss)
        #                     vr = tb.get_vr()
        #                     vl = tb.get_vl()
        #                     par_pnt = get_parallelogram_missing_point(curr_poss, vr, vl, v_name)
        #                     client.flyToPosition(par_pnt.x, par_pnt.y, config.height, config.ltf_velocity)
        #                 #time.sleep(0.24)
        #                 y = 9
        #                 tb = TangentBug()
        #                 DEBUG_ltg_count +=1
        #                 print("tangent bug number : "+ str(DEBUG_ltg_count))
        #
        #             print("sensed obstacle : " + str(points_list), str(pose))
        #             print("getPointInRealWorldCoords -> ", "(" + str(xw) + ", " + str(yw) + ")")
        #             print("Drone location : (", str(client.getPose().pos.x_m), ", " + str(client.getPose().pos.y_m) , ", " + str(client.getPose().pos.z_m)+ ")")
        #             print ("unknown obstacle !")
        #

            # else:
            #     print("didn't sense obstacle : " + str(points_list), str(pose))
            # if sensing_obstacle:
            #     print("Lidar points : " + str(points_list))
            #
            # position = client.getPose().pos


            # if obs.is_point_in_obstacle()
            #
            #
            # md.set_real_path(real_path)
            # md.show()
    finally:
        agent.client.reset()
    exit(1)


