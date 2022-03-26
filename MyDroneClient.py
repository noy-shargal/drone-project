from DroneClient import DroneClient
import DroneTypes
import numpy as np
import math
from shapely.geometry import Point

import time

from Obstacles import Obstacles
from utils import getPointInRealWorldCoords

from Config import config
from Obstacle import ThinWallObstacle


class LidarPointInfo:
    def __init__(self, x, y, r, theta, valid):
        self.x = x
        self.y = y
        self.r = r
        self.theta = theta
        self.valid = valid

#LidarPointInfo = np.dtype([('x', np.float32), ('y', np.float ), ('r', np.float )])
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
        self.flyToPosition(x, y, z, 0.01)

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

    def isNewObstaclefullSenseObstacle(self, obs: Obstacles):
        lidar_data = self.full_lidar_scan(0.0, 0.04, True)
        for pi in lidar_data:
            if pi != np.float(np.inf):
                obs.is_point_in_obstacles_map(Point(pi.x, pi.y))
                return True
        return False

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
        for i in range(len(lidar_data) // 3):
            output.append((lidar_data[i * 3], lidar_data[i * 3 + 1]))
        return output

    def full_lidar_scan(self, full_lidar_scan_time, sleep_between_samples=0.07, verbose=False):
        """
        acquires a full angle aperture scan for the lidar
        :param theta_resolution: the step between different acquisitions
        :param continuously_update: if on will keep updating an acquired cell with new samples
        :return: a vector containing discrete samples for the entire angle range
        """
        num_of_angles = self.LIDAR_ANGLE_APERTURE // config.lidar_theta_resolution
        output = np.ones((num_of_angles,)) * np.float(np.inf)
        scans = int(full_lidar_scan_time / sleep_between_samples)
        for i in range(scans):
            lidar_data = self.client.getLidarData('Lidar1')
            if len(lidar_data.point_cloud) >= 3:
                x, y = lidar_data.point_cloud[0], lidar_data.point_cloud[1]
                r, theta_rad = self.getPointInPolarCoords(x, y)
                r -= config.buffer_size
                r = max(config.buffer_size, r)
                theta = theta_rad * 180 / math.pi
                angle_index = self._angle_to_index(theta, config.lidar_theta_resolution)
                output[angle_index] = r
                if verbose:
                    print(f"LIDAR: {theta}, {angle_index}, {r}")
            time.sleep(sleep_between_samples)
        return output

    def full_lidar_scan_v2(self, full_lidar_scan_time, sleep_between_samples=0.07, verbose=False):
        """
        acquires a full angle aperture scan for the lidar
        :param theta_resolution: the step between different acquisitions
        :param continuously_update: if on will keep updating an acquired cell with new samples
        :return: a vector containing discrete samples for the entire angle range
        """
        num_of_angles = self.LIDAR_ANGLE_APERTURE // config.lidar_theta_resolution
        output = np.ones((num_of_angles,)) * np.float(np.inf)
        world_cords_dict = {}
        scans = int(full_lidar_scan_time / sleep_between_samples)
        world_cords_dict = self._initialize_world_coords_dict(num_of_angles)
        for i in range(scans):
            lidar_data = self.client.getLidarData('Lidar1')
            if len(lidar_data.point_cloud) >= 3:
                x, y = lidar_data.point_cloud[0], lidar_data.point_cloud[1]
                r, theta_rad = self.getPointInPolarCoords(x, y)

                r -= config.buffer_size
                r = max(config.buffer_size, r)
                theta = theta_rad * 180 / math.pi
                angle_index = self._angle_to_index(theta, config.lidar_theta_resolution)
                output[angle_index] = r

                wx, wy = getPointInRealWorldCoords(x, y, self.getPose())

                world_cords_dict[angle_index] = LidarPointInfo( wx, wy, r, theta, True)

                if verbose:
                    print(f"LIDAR: {theta}, {angle_index}, {r}")
            time.sleep(sleep_between_samples)
        return output, world_cords_dict

    @staticmethod
    def _angle_to_index(angle, theta_resolution = config.lidar_theta_resolution):
        return int((angle + 90) / theta_resolution)

    def zero_angle_to_index(angle, theta_resolution):
        return int(90 / theta_resolution)

    def lidar_scan_contains_obstacle(self, full_lidar_scan):
        for lidar_scan_idx in range(len(full_lidar_scan)):
            if full_lidar_scan[lidar_scan_idx] < np.float(np.inf):  # obstacle was detected
                return True
        return False

    def _build_obstacles(self, full_lidar_scan, current_pose, verbose=False):
        output = list()
        first_endpoint_r_index = None
        trend = None
        padded_full_lidar_scan = np.array(np.concatenate(([np.float(np.inf)], full_lidar_scan, [np.float(np.inf)])))

        for i in range(1, len(padded_full_lidar_scan) - 1):
            if padded_full_lidar_scan[i] < np.float(np.inf) and padded_full_lidar_scan[i - 1] == np.float(np.inf):
                first_endpoint_r_index = i
            if padded_full_lidar_scan[i + 1] == np.float(np.inf) and padded_full_lidar_scan[i] < np.float(np.inf):
                second_endpoint_r_index = i
                assert first_endpoint_r_index is not None
                new_obs = self._build_obstacle(first_endpoint_r_index, second_endpoint_r_index, current_pose,
                                               padded_full_lidar_scan)
                output.append(new_obs)
                first_endpoint_r_index = None
            elif first_endpoint_r_index is not None and first_endpoint_r_index < i:
                current_trend = np.sign(padded_full_lidar_scan[i] / padded_full_lidar_scan[i - 1])
                if trend is None:
                    trend = current_trend
                elif current_trend != trend:
                    second_endpoint_r_index = i
                    new_obs = self._build_obstacle(first_endpoint_r_index, second_endpoint_r_index, current_pose,
                                                   padded_full_lidar_scan)
                    output.append(new_obs)
                    first_endpoint_r_index = None
        if verbose:
            print("Number of obstacles created in '_build_obstacles' is : ", len(output))
        return output

    def _build_obstacle(self, first_endpoint_r_index, second_endpoint_r_index, current_pose, full_lidar_scan):
        first_endpoint_r = full_lidar_scan[first_endpoint_r_index]
        first_theta = self._angle_index_to_value(first_endpoint_r_index - 0.5, len(full_lidar_scan))
        second_endpoint_r = full_lidar_scan[second_endpoint_r_index]
        second_theta = self._angle_index_to_value(second_endpoint_r_index + 0.5, len(full_lidar_scan))

        first_endpoint_in_world_coordinates = self._calculate_world_coordinates(first_endpoint_r, first_theta,
                                                                                current_pose)
        second_endpoint_in_world_coordinates = self._calculate_world_coordinates(second_endpoint_r, second_theta,
                                                                                 current_pose)
        obs = ThinWallObstacle(first_endpoint_in_world_coordinates, second_endpoint_in_world_coordinates)
        return obs

    @staticmethod
    def _calculate_world_coordinates(r, theta, current_pose):
        real_theta = theta - 90
        real_theta = real_theta * math.pi / 180

        x_drone = r * np.cos(real_theta)
        y_drone = r * np.sin(real_theta)

        xw, yw = getPointInRealWorldCoords(x_drone, y_drone, current_pose)

        return Point(xw, yw)

    @staticmethod
    def _angle_index_to_value(angle_index, num_of_values):
        return angle_index * 180 / num_of_values

    def _initialize_world_coords_dict(self, num_of_angles):
        output = dict()
        for i in range(num_of_angles):
            output[i] = LidarPointInfo( 0, 0, 0, 0, False)
        return output
