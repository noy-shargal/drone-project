import math
from DroneClient import DroneClient
from DroneTypes import Position


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


def reached_goal_2D(curr: Position, goal: Position):
    diff_x = curr.x_m - goal.x_m
    diff_y = curr.y_m - goal.y_m
    dist = math.sqrt(diff_x * diff_x + diff_y * diff_y)

    if dist < 11.0:
        return True
    return False


if __name__ == "__main__":

    client = MyDroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)

    time.sleep(3)
    goal = Position()
    goal.x_m, goal.y_m, goal.z_m = -346, -500, -100
    client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, 5)

    while True:
        lidar_data = client.getLidarData()
        print("Lidar Data: " + str(lidar_data))

        print("Position: ", str(client.getPose().pos))

        if reachedGoal2D(client.getPose().pos, goal):
            print("Reached goal")
            client.stop()

        if client.senseObstacle():
            print("Found obstacle.")
            client.stop()
        time.sleep(1)
