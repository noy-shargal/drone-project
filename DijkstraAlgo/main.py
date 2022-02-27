import math
from Dijkstra import Dijkstra
from MapDrawer import MapDrawer
from DroneClient import DroneClient
from DroneTypes import Position
from Obstacles import Obstacles
from shapely.geometry import Polygon,Point,box, mapping


class MyDroneClient (DroneClient):

    def __init__(self):
        super().__init__()
        self.stopped = False


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
            return False
        return True




def reached_goal_2D(curr: Position, goal: Position):

    diff_x =  curr.x_m - goal.x_m
    diff_y = curr.y_m - goal.y_m
    dist = math.sqrt(diff_x*diff_x + diff_y*diff_y)

    if dist < 11.0:
        return True
    return False


if __name__ == "__main__":

    sourcePoints = [(-1500.0, -1200.0), (-1540.0, -1200.0), (-1540.0, -1240.0), (-1500.0, -1240.0), (-1500.0, -1200.0)]
    source = Polygon(sourcePoints)
    dstPoints = [(0.0, -600.0), (0.0, -640.0), (40.0, -640.0), (40.0, -600.0), (0.0, -600.0)]
    destination = Polygon(dstPoints)
    obs = Obstacles()
    obs.read_csv()

    obs.set_start_polygon(source)
    obs.set_destination_polygon(destination)



    md = MapDrawer()
    polygons = obs.get_polygons()
    md.add_polygons(polygons)
    print("Number of polygons is: "+str(len(polygons)))

    obs.build_visibility_graph()
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
    print ("Total Distance: " + str(dst.get_distance()))
    print ("Number Of vetices visited :" + str(dij.get_num_of_vertices_visited()))
    md.show()
    y = 3*8
    # obs = Obstacles()
    # obs.read()
    # obs.add_obstacle(10, 10, 99)
    # obs.print()


    # client = MyDroneClient()
    # client.connect()
    #
    # print(client.isConnected())
    #
    # time.sleep(4)
    # client.setAtPosition(-346, -700, -100)
    #
    # time.sleep(3)
    # goal = Position()
    # goal.x_m, goal.y_m, goal.z_m = -346, -500, -100
    # client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, 5)
    #
    # while True:
    #     lidar_data = client.getLidarData()
    #     print("Lidar Data: "+str(lidar_data))
    #
    #     print("Position: ",str(client.getPose().pos ))
    #
    #     if reachedGoal2D(client.getPose().pos, goal):
    #         print("Reached goal")
    #         client.stop()
    #
    #     if client.senseObstacle():
    #         print ("Found obstacle.")
    #         client.stop()
    #     time.sleep(1)


        
