from DroneClient import DroneClient
import DroneTypes
import numpy as np
import math
import time
from shapely.geometry import Point
import airsim

class MyDroneClient(DroneClient):
    LIDAR_ANGLE_APERTURE = 180

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

    @staticmethod
    def parse_lidar_data(lidar_data):
        assert len(lidar_data) % 3 == 0
        output = list()
        for i in range(len(lidar_data)//3):
            output.append((lidar_data[i*3], lidar_data[i*3+1]))
        return output

    def full_lidar_scan(self, theta_resolution=10, sleep_between_samples=0.07):
        """
        acquires a full angle aperture scan for the lidar
        :param theta_resolution: the step between different acquisitions
        :param continuously_update: if on will keep updating an acquired cell with new samples
        :return: a vector containing discrete samples for the entire angle range
        """
        num_of_angles = self.LIDAR_ANGLE_APERTURE // theta_resolution
        output = np.ones((num_of_angles,))*np.float(np.inf)
        for i in range(3*int(180/theta_resolution)+2):
            lidar_data = self.client.getLidarData('Lidar1')
            if len(lidar_data.point_cloud) >= 3:
                x, y = lidar_data.point_cloud[0], lidar_data.point_cloud[1]
                r, theta_rad = self.getPointInPolarCoords(x, y)
                theta = theta_rad * 180 / math.pi
                angle_index = self._angle_to_index(theta, theta_resolution)
                output[angle_index] = r
                if output[angle_index] < np.float(np.inf):
                    print(f"LIDAR: {theta}, {angle_index}, {r}")
            time.sleep(sleep_between_samples)
        return output


    @staticmethod
    def _extract_angle(pose):
        print(airsim.to_eularian_angles(pose.orientation))
        theta = math.atan2(pose.orientation.y_val, pose.orientation.x_val) * 180 / math.pi
        return theta

    @staticmethod
    def _prepare_lidar_value(point_cloud):
        if len(point_cloud) == 1 and point_cloud[0] == 0.0:
            return np.float(np.inf)
        x, y = point_cloud[0], point_cloud[1]
        dist = math.sqrt(x**2 + y**2)
        return dist

    @staticmethod
    def _angle_to_index(angle, theta_resolution):
        return int((angle + 90) / theta_resolution)
