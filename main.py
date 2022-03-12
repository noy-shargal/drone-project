import time
from shapely.geometry import Polygon
from DroneClient import DroneClient
import DroneTypes
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

    def getLidarData(self):
        point_cloud = DroneTypes.PointCloud()
        lidar_data = self.client.getLidarData()

        point_cloud.points = lidar_data.point_cloud

        return point_cloud

    def senseObstacle(self):
        lidar_data = self.getLidarData()
        if lidar_data.points == [0.0]:
            return False
        return True


if __name__ == "__main__":
    sourcePoints = [(-1200.0, -1200.0), (-1240.0, -1200.0), (-1240.0, -1240.0), (-1200.0, -1240.0), (-1200.0, -1200.0)]
    curr_position = (-1200.0, -1200.0)
    source = Polygon(sourcePoints)
    path_planner = PathPlanner(curr_position, (0.0, -600.0))
    map_drawer = MapDrawer(*path_planner.get_boundaries())
    polygons_map = path_planner.polygons_map
    polygons_map['start'] = source

    client = MyDroneClient()
    client.connect()
    print(client.isConnected())
    dstPoints = [(0.0, -600.0), (0.0, -640.0), (40.0, -640.0), (40.0, -600.0), (0.0, -600.0)]
    destination = Polygon(dstPoints)
    time.sleep(4)
    client.setAtPosition(-1200, -1200, -100)
    time.sleep(3)
    goal = DroneTypes.Position()
    goal.x_m, goal.y_m, goal.z_m = 0.0, -600.0, -100
    # client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, 5)
    polygons_map['goal'] = destination
    # map_drawer.paint_attraction_map(path_planner._attraction_map)
    map_drawer.add_polygons(polygons_map)
    path = [curr_position]
    draw_count = 0

    while not path_planner.reached_goal(curr_position):
        next_position = path_planner.next_step(curr_position)
        print(next_position)
        client.flyToPosition(next_position[0], next_position[1], -100, 5)
        while not path_planner.reached_location(curr_position, next_position):
            curr_position = client.getPose().pos.x_m, client.getPose().pos.y_m
        curr_position = next_position
