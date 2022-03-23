from enum import Enum
from typing import List
import math

import numpy as np
from shapely.geometry import LineString, Point

from LTG import AugmentedSubGraph, TGEdge, TGVertex
from Obstacle import ThinWallObstacle
from utils import getPointInRealWorldCoords


class TangentBugModes(Enum):
    TARGET = 1
    WALL_WALKING = 2
    TRANSITION = 3


class TangentBug:
    MODES = ['Target', 'Wall', 'Transition']

    def __init__(self, target, convex_angle_steps=2):

        self._target = target
        self._mode = TangentBugModes.TARGET
        self._wall_mode_data = {'last_direction': None}
        self._convex_angle_steps = convex_angle_steps
        self._transition_mode_goal = None
        self.obstacles = None
        self._v_leave = None

    def step(self, curr_pose, full_lidar_scan):
        """
        return a valid step (Point) or -1 if the target is not reachable
        ToDo: change to exception
        """
        curr_pos = curr_pose.pos
        sub_graph = self._build_augmented_sub_graph(full_lidar_scan, curr_pose)
        if self._mode == TangentBugModes.TARGET:
            step = self._target_step(curr_pos, sub_graph)
        elif self._mode == TangentBugModes.WALL_WALKING:
            step = self._wall_step(curr_pos, sub_graph)
        else:
            step = self._transition_step(curr_pos, sub_graph)
        return step

    def _target_step(self, curr_pos, sub_graph: AugmentedSubGraph):
        point, min_distance = sub_graph.get_closet_point_to_target()
        local_minima = curr_pos.distance(self._target) <= min_distance
        if not local_minima:
            return point
        self._enter_wall_mode(curr_pos, sub_graph)
        return self._wall_step(curr_pos, sub_graph)

    def _enter_wall_mode(self, curr_pos, sub_graph: AugmentedSubGraph):
        self._mode = TangentBugModes.WALL_WALKING
        blocking_obstacle = sub_graph.get_blocking_obstacle()
        d_min, _ = blocking_obstacle.get_closest_point_to_target(self._target)

        self._wall_mode_data['blocking_obstacle'] = blocking_obstacle
        self._wall_mode_data['d_min'] = d_min
        self._wall_mode_data['start_pos'] = curr_pos

    def _wall_step(self, curr_pos, sub_graph: AugmentedSubGraph):
        sub_graph.update_blocking_obstacle(curr_pos, self.obstacles)
        blocking_obstacle = sub_graph.get_blocking_obstacle()
        d_min, _ = blocking_obstacle.get_closest_point_to_target(self._target)
        self._wall_mode_data['d_min'] = d_min

        if sub_graph.is_g2_empty(d_min):
            step = blocking_obstacle.get_positive_direction(self._wall_mode_data['last_direction'],
                                                            self._target)
            self._wall_mode_data['last_step'] = self._build_last_direction(step, curr_pos)
            return step
        self._enter_transition_mode(sub_graph)
        return self._transition_step(curr_pos, sub_graph)

    def _transition_step(self, curr_pos: Point, sub_graph):
        if curr_pos.distance(self._target) >= self._wall_mode_data['d_min']:
            return self._v_leave
        self._mode = TangentBugModes.TARGET
        self._wall_mode_data = {'last_direction': None}
        self._target_step(curr_pos, sub_graph)

    @staticmethod
    def _build_last_direction(step, curr_pos):
        output = np.array([step.x - curr_pos.x,
                           step.y - curr_pos.y])
        return output

    def _build_obstacles(self, full_lidar_scan, current_pose, threshold=0.1):
        output = list()
        first_endpoint_r_index = None
        for i in range(1, len(full_lidar_scan)-1):
            if full_lidar_scan[i] < np.float(np.inf) and full_lidar_scan[i-1] == np.float(np.inf):
                first_endpoint_r_index = i
            if full_lidar_scan[i+1] == np.float(np.inf) and full_lidar_scan[i] < np.float(np.inf):
                second_endpoint_r_index = i
                if first_endpoint_r_index is not None:
                    new_obs = self._build_obstacle(first_endpoint_r_index, second_endpoint_r_index, current_pose,
                                                full_lidar_scan)
                    output.append(new_obs)
                    first_endpoint_r_index = None

        return output

    def _enter_transition_mode(self, sub_graph: AugmentedSubGraph):
        self._mode = TangentBugModes.TRANSITION
        self._v_leave, _ = sub_graph.get_closet_point_to_target()

    def _add_vertices(self, graph, obstacles, curr_pos):
        start_point = Point(curr_pos.x_m, curr_pos.y_m)
        v_start = TGVertex(start_point, False,"START")
        graph.add_vertex(v_start)

        v_T = TGVertex(self._target, False,"TARGET")
        graph.add_vertex(v_T)

        for obs in obstacles:
            v1 = TGVertex(obs._first_endpoint, True)
            v2 = TGVertex(obs._second_endpoint, True)

            graph.add_vertex(v1)
            graph.add_vertex(v2)

        graph.remove_duplicate_vertices()
        graph.try_to_add_T_node(start_point, self._target, obstacles)

    def _add_start_edges(self, graph: AugmentedSubGraph):
        vertices = graph.get_vertices()
        start_vertex = graph.get_start()
        target_vertex = graph.get_target()
        start_to_target_distance = start_vertex.distance(target_vertex)

        for v in vertices:
            if v.vtype == "INNER" or v.vtype == "T_NODE":
                if v.distance(target_vertex) < start_to_target_distance:  # admisible vertex
                    v.is_admissible = True
                edge = TGEdge(start_vertex, v, False)
                graph.add_edge(edge)
                start_vertex.add_edge(edge)
                v.add_edge(edge)

    def is_line_intersects_with_obstacles(self, line: LineString, obstacles):
        for obs in obstacles:
            if obs.intersects_line(line):
                return True
        return False

    def _try_to_update_status(self, graph: AugmentedSubGraph, vertex: TGVertex, handled_list: List[TGVertex],
                              obstacles):

        min_distance = np.float(np.inf)
        closet_relevant_vertex = None
        for hv in handled_list:
            line = LineString([hv.point(), vertex.point()])
            if not self.is_line_intersects_with_obstacles(line, obstacles):
                distance_to_hv = vertex.distance(hv)
                total_distance = distance_to_hv + hv.get_distance_to_target()
                if total_distance < min_distance:
                    min_distance = total_distance
                    closet_relevant_vertex = hv

        if closet_relevant_vertex is not None:
            edge = TGEdge(vertex, self._target, "VIRTUAL_EDGE", min_distance)
            vertex.set_status(True, min_distance)
            graph.add_edge(edge)
            return True
        return False

    def _add_virtual_edges_to_target(self, graph: AugmentedSubGraph, obstacles):
        vertices = graph.get_vertices()
        target_vertex = graph.get_target()
        inner_vertices = list()

        for v in vertices:
            if v.vtype == "INNER" and v.is_admissible:
                inner_vertices.append(v)

        handled_list = list()

        for v in inner_vertices:
            if not self.is_line_intersects_with_obstacles(LineString([v.point(), target_vertex.point()]), obstacles):
                edge = TGEdge(v, target_vertex, "VIRTUAL_EDGE")  # there is direct line
                v.set_status(True, edge.distance)
                graph.add_edge(edge)
                handled_list.append(v)

        while len(handled_list) < len(inner_vertices):
            for v in inner_vertices:
                if not v.get_status():
                    if self._try_to_update_status(graph, v, handled_list, obstacles):
                        handled_list.append(v)

    def _build_augmented_sub_graph(self, full_lidar_scan, curr_pose):
        curr_pos = curr_pose.pos
        self.obstacles = self._build_obstacles(full_lidar_scan, curr_pose)  # list of thin wall obstacles
        curr_point = Point(curr_pos.x_m, curr_pos.y_m)
        graph = AugmentedSubGraph(curr_point, self._target)
        self._add_vertices(graph, self.obstacles, curr_pos)
        self._add_start_edges(graph)
        self._add_virtual_edges_to_target(graph, self.obstacles)
        return graph

    def _build_obstacle(self, first_endpoint_r_index, second_endpoint_r_index, current_pose, full_lidar_scan):
        first_endpoint_r = full_lidar_scan[first_endpoint_r_index]
        first_theta = self._angle_index_to_value(first_endpoint_r_index, len(full_lidar_scan))
        second_endpoint_r = full_lidar_scan[second_endpoint_r_index]
        second_theta = self._angle_index_to_value(second_endpoint_r_index, len(full_lidar_scan))

        first_endpoint_in_world_coordinates = self._calculate_world_coordinates(first_endpoint_r, first_theta,
                                                                                current_pose)
        second_endpoint_in_world_coordinates = self._calculate_world_coordinates(second_endpoint_r, second_theta,
                                                                                 current_pose)
        obs = ThinWallObstacle(first_endpoint_in_world_coordinates, second_endpoint_in_world_coordinates)
        return obs

    @staticmethod
    def _angle_index_to_value(angle_index, num_of_values):
        return angle_index * 180 / num_of_values

    @staticmethod
    def _calculate_world_coordinates(r, theta, current_pose):
        real_theta = theta - 90
        real_theta = real_theta * math.pi /180

        x_drone = r * np.cos(real_theta)
        y_drone = r * np.sin(real_theta)

        xw, yw = getPointInRealWorldCoords(x_drone, y_drone, current_pose)

        return Point(xw, yw)

