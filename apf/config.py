import numpy as np


class InfiniteRepulsionConfig:
    def __init__(self,
                 start_position,
                 end_position,
                 height,
                 q_star,
                 s,
                 k,
                 d,
                 grid_size,
                 velocity,
                 data_type,
                 unknown_amplification,
                 use_obstacles_map,
                 reach_dist,
                 window_size):
        self.q_star = q_star
        self.s = s
        self.k = k
        # self.start_position = start_position
        # self.end_position = end_position
        # self.height = height
        # self.velocity = velocity
        self.d = d
        self.grid_size = grid_size
        self.data_type = data_type
        self.unknown_amplification = unknown_amplification
        self.use_obstacles_map = use_obstacles_map
        self.reach_dist = reach_dist
        self.window_size = window_size


class FiniteRepulsionConfig:
    def __init__(self,
                 start_position,
                 end_position,
                 height,
                 q_star,
                 s,
                 k,
                 d,
                 grid_size,
                 velocity,
                 data_type,
                 unknown_amplification,
                 use_obstacles_map,
                 reach_dist,
                 window_size,
                 eta,
                 lidar_padding):
        self.q_star = q_star
        self.s = s
        self.k = k
        self.eta = eta
        # self.start_position = start_position
        # self.end_position = end_position
        # self.height = height
        # self.velocity = velocity
        self.d = d
        self.grid_size = grid_size
        self.data_type = data_type
        self.unknown_amplification = unknown_amplification
        self.use_obstacles_map = use_obstacles_map
        self.reach_dist = reach_dist
        self.window_size = window_size
        self.lidar_padding = lidar_padding


finite_repulsion_config1 = FiniteRepulsionConfig(start_position=(-1200.0, -1200.0),
                                                 end_position=(0.0, -600.0),
                                                 height=-100,
                                                 q_star=30,
                                                 s=25,
                                                 k=0.025,
                                                 d=30.0,
                                                 grid_size=0.25,
                                                 velocity=1.0,
                                                 data_type=np.float32,
                                                 unknown_amplification=50,
                                                 use_obstacles_map=False,
                                                 reach_dist=10,
                                                 window_size=30,
                                                 eta=7,
                                                 lidar_padding=0)
current_config = finite_repulsion_config1
