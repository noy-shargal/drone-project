from enum import Enum
from typing import List

import numpy as np
from shapely.geometry import LineString

from LTG import AugmentedSubGraph, TGEdge, TGVertex
from Obstacle import ThinWallObstacle

class TangentBugModes(Enum):
    TARGET= 1
    WALL_WALKING = 2
    TRANSITION = 3


class TangentBug:
    MODES = ['Target', 'Wall', 'Transition']

    def __init__(self, target, convex_angle_steps=2):

        self._target = target
        self._mode = self.MODES[0]
        self.current_pose = None
        self._wall_mode_data = dict()
        self._convex_angle_steps = convex_angle_steps
        self._transition_mode_goal = None


    def step(self, curr_pos, full_lidar_scan):
        """
        return a valid step (Point) or -1 if the target is not reachable
        ToDo: change to exception
        """

        sub_graph = self._build_augmented_sub_graph(full_lidar_scan, curr_pos)
        if self._mode == TangentBugModes.TARGET:
            step = self._target_step(curr_pos, sub_graph)
        elif self._mode == TangentBugModes.WALL_WALKING:
            step = self._wall_step(sub_graph)
        else:
            step = self._transition_step(sub_graph)
        return step

    def _target_step(self,curr_pos, sub_graph: AugmentedSubGraph):
        point, min_distance =  sub_graph.get_closet_point_to_target()
        local_minima =  curr_pos.distance(self._target) <= min_distance
        if not local_minima:
            return point
        self._enter_wall_mode(sub_graph)
        return self._wall_step(sub_graph)

    def _enter_wall_mode(self, sub_graph: AugmentedSubGraph):
        self._mode = TangentBugModes.WALL_WALKING
        blocking_obstacle = sub_graph.get_blocking_obstacle()
        d_min, point = blocking_obstacle.get_closest_point_to_target(self._target)

        self._wall_mode_data['blocking_obstacle'] = blocking_obstacle
        self._wall_mode_data['d_min'] = d_min
        self._start_point = self.current_pose.location

    def _wall_step(self, sub_graph: AugmentedSubGraph):
        blocking_obstacle = sub_graph.get_blocking_obstacle()
        self._wall_mode_data['d_min'] = sub_graph.calculate_d_min()
        if sub_graph.g2_is_empty():
            step = self._next_boundary_following_or_exit(blocking_obstacle)
            self._wall_mode_data['last_step'] = step
            return step
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

    def _add_vertices(self, graph, obstacles, curr_pos):
        v_start = TGVertex(curr_pos, "START")
        graph.add_vertex(v_start)

        v_T = TGVertex(self._target, "TARGET")
        graph.add_vertex(v_T)

        for obs in obstacles:
            v1 = TGVertex(obs._first_endpoint)
            v2 = TGVertex(obs._second_endpoint)

            graph.add_vertex(v1)
            graph.add_vertex(v2)

        graph.remove_duplicate_vertices()
        graph.try_to_add_T_node(curr_pos, self._target)

    def _add_start_edges(self, graph: AugmentedSubGraph):
        vertices = graph.get_vertices()
        start_vertex = graph.get_start()
        target_vertex = graph.get_target()
        start_to_target_distance = start_vertex.distance(target_vertex)

        for v in vertices:
            if v.vtype == "INNER" or v.vtype == "T_NODE":
                if v.distance(target_vertex) < start_to_target_distance: # admisible vertex
                    v.is_admissible = True
                edge = TGEdge(start_vertex, v)
                graph.add_edge(edge)
                start_vertex.add_edge(edge)
                v.add_edge(edge)

    def is_line_intersects_with_obstacles(self, line: LineString, obstacles):

        for obs in obstacles:
            if obs.intersect(line):
                return True
        return  False

    def _try_to_update_status(self, graph : AugmentedSubGraph,vertex: TGVertex, handled_list:List[TGVertex], obstacles):

        min_distance = np.float(np.inf)
        closet_relevant_vertex = None
        for hv in handled_list:
            line = LineString(hv.point(), vertex.point())
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
        inner_vertices = [v for v in vertices if v.vtype == "ÏNNER" and v.is_admissible]
        handled_list = list()

        for v in inner_vertices:
            if not self.is_line_intersects_with_obstacles(LineString[v.point(), target_vertex.point()]):
                edge = TGEdge(v, target_vertex,"VIRTUAL_EDGE") # there is direct line
                v.set_status(True, edge.distance)
                graph.add_edge(edge)
                handled_list.append(v)

        while len(handled_list) < len(inner_vertices):
            for v in inner_vertices:
                if not v.get_status():
                    if self._try_to_update_status(graph, v, handled_list, obstacles):
                        handled_list.append(v)

    def _build_augmented_sub_graph(self, full_lidar_scan, curr_pos):
        obstacles = self._build_obstacles(full_lidar_scan) # list of thin wall obstacles
        graph = AugmentedSubGraph(curr_pos, self._target)
        self._add_vertices(graph, obstacles, curr_pos)
        self._add_start_edges(graph)
        self._add_virtual_edges_to_target(graph, obstacles)
        return graph





