import numpy as np

from LTG import AugmentedSubGraph
from Obstacle import ThinWallObstacle


class TangentBug:
    MODES = ['Target', 'Wall', 'Transition']

    def __init__(self, start, target, convex_angle_steps=2):
        self._start = start
        self._target = target
        self._mode = self.MODES[0]
        self.current_pose = None

        self._wall_mode_data = dict()
        self._convex_angle_steps = convex_angle_steps

        self._transition_mode_goal = None

    def step(self, current_pose, full_lidar_scan):
        """
        return a valid step or -1 if the target is not reachable
        ToDo: change to exception
        """
        self.current_pose = current_pose
        obstacles = self._build_obstacles(full_lidar_scan)
        sub_graph = AugmentedSubGraph(obstacles, current_pose.location,  self._target)
        if self._mode == self.MODES[0]:
            step = self._target_step(sub_graph)
        elif self._mode == self.MODES[1]:
            step = self._wall_step(sub_graph)
        else:
            step = self._transition_step(sub_graph)
        return step

    def _target_step(self, sub_graph: AugmentedSubGraph):
        if not sub_graph.is_source_local_minima():
            return sub_graph.get_closet_point_to_target()
        self._enter_wall_mode(sub_graph)
        return self._wall_step(sub_graph)

    def _enter_wall_mode(self, sub_graph: AugmentedSubGraph):
        self._mode = self.MODES[1]
        self._wall_mode_data['blocking_obstacle'] = sub_graph.get_blocking_obstacle()
        self._wall_mode_data['d_min'] = sub_graph.calculate_d_min()
        self._wall_mode_data['direction'] = sub_graph.find_following_direction()
        self._start_point = self.current_pose.location

    def _wall_step(self, sub_graph: AugmentedSubGraph):
        blocking_obstacle = sub_graph.get_blocking_obstacle()
        self._wall_mode_data['d_min'] = sub_graph.calculate_d_min()
        if sub_graph.g2_is_empty():
            return self._next_boundary_following_or_exit(blocking_obstacle)
        self._enter_transition_mode()
        return self._transition_step(sub_graph)

    def _transition_step(self, sub_graph: AugmentedSubGraph):
        pass

    def _build_obstacles(self, full_lidar_scan, threshold=0.1):
        output = list()
        second_derivative = np.gradient(full_lidar_scan)
        abs_second_derivative = np.fabs(second_derivative)
        discontinuities = np.where(abs_second_derivative > threshold, 1, 0)
        first_endpoint_r_index = None
        for i in range(len(full_lidar_scan)):
            if discontinuities[i] == 1:
                if first_endpoint_r_index is None:
                    first_endpoint_r_index = full_lidar_scan[i], i
                else:
                    first_endpoint_real_xy = self._calculate_real_world_coordinates(first_endpoint_r_index)
                    second_endpoint_real_xy = self._calculate_real_world_coordinates(full_lidar_scan[i], i)
                    output.append(ThinWallObstacle(first_endpoint_real_xy, second_endpoint_real_xy))
                    first_endpoint_r_index = None
        return output

    def _enter_transition_mode(self, exit_vertex):
        self._mode = self.MODES[2]
        pass

    def _next_boundary_following_or_exit(self, blocking_obstacle):
        raise NotImplementedError("handle convex vertex")
        pass
