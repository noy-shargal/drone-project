from DroneClient import DroneClient
import DroneTypes
import numpy as np
import math
from shapely.geometry import Point


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

    def full_lidar_scan(self, theta_resolution=1, continuously_update=True):
        """
        acquires a full angle aperture scan for the lidar
        :param theta_resolution: the step between different acquisitions
        :param continuously_update: if on will keep updating an acquired cell with new samples
        :return: a vector containing discrete samples for the entire angle range
        """
        num_of_angles = self.LIDAR_ANGLE_APERTURE // theta_resolution
        output = np.zeros((num_of_angles,))
        while np.any(np.zeros_like(output) == output):
            lidar_data = self.client.getLidarData('Lidar1')
            angle = self._extract_angle(lidar_data.pose)
            value = self._prepare_lidar_value(lidar_data.point_cloud)
            angle_index = self._angle_to_index(angle, theta_resolution)
            if continuously_update or not output[angle_index]:
                output[angle_index] = value
        return output

    @staticmethod
    def _extract_angle(pose):
        return pose.orientation.z_rad

    @staticmethod
    def _prepare_lidar_value(point_cloud):
        x, y = point_cloud[0], point_cloud[1]
        dist = math.sqrt(x**2 + y**2)
        if dist == 0.0:
            dist = np.float(np.inf)
        return dist

    @staticmethod
    def _angle_to_index(angle, theta_resolution):
        return int(angle / theta_resolution)