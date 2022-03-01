import math
import time

from shapely.geometry import Polygon

from DroneClient import DroneClient
from DroneTypes import Position
from MapDrawer import MapDrawer
from PathPlanner import PathPlanner


class MyDroneClient(DroneClient):

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


if __name__ == "__main__":

    client = MyDroneClient()
    client.connect()
    print(client.isConnected())
    sourcePoints = [(-1500.0, -1200.0), (-1540.0, -1200.0), (-1540.0, -1240.0), (-1500.0, -1240.0), (-1500.0, -1200.0)]
    source = Polygon(sourcePoints)
    dstPoints = [(0.0, -600.0), (0.0, -640.0), (40.0, -640.0), (40.0, -600.0), (0.0, -600.0)]
    destination = Polygon(dstPoints)
    time.sleep(4)
    client.setAtPosition(-346, -700, -100)
    time.sleep(3)
    goal = Position()
    goal.x_m, goal.y_m, goal.z_m = -346, -500, -100
    client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, 5)
    curr_position = (-346, -700)
    path_planner = PathPlanner(curr_position, (-346, -500))
    map_drawer = MapDrawer(*path_planner.get_boundaries())
    map_drawer.add_polygons(path_planner.polygons_map)
    path = [curr_position]
    while not path_planner.reached_goal(curr_position):
        prev_position = curr_position
#        time.sleep(1)
        curr_position = path_planner.next_step(curr_position)
        path.append(curr_position)

    map_drawer.set_path(path)
    map_drawer.show()


