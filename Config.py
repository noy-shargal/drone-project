from shapely.geometry import Point


class Config:

    def __init__(self):

        self.source = Point(-1200.0, -1200)
        self.destination = Point(0.0, -600.0)
        #
        # self.source = Point(-1100.0, -1170)
        # self.destination = Point(-200, -550.0)

        self.height = -50
        self.astar_velocity = 10
        self.apf_velocity = 4
        self.ltf_velocity = 3
        self.load_from_json = True
        self.show_map = False

        # bug properties
        self.lidar_theta_resolution = 10
        self.fly_with_full_lidar = True
        self.lidar_scans_number = 1
        self.buffer_size = 0
        self.wall_detection_angle_fov = 60

        self.obstacles_padding = 4
        self.bug2_velocity = 2


config = Config()
