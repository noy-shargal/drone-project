from PathPlanner import PathPlanner
from MyDroneClient import MyDroneClient
from typing import Tuple
import time


class Agent:
    HEIGHT = -50
    VELOCITY = 6

    def __init__(self, start: Tuple, goal: Tuple):
        self._path_planner = PathPlanner(start, goal)
        self._drone_client = MyDroneClient()
        self._start = start

    def connect_and_spawn(self):
        self._drone_client.connect()
        time.sleep(2)
        self._drone_client.setAtPosition(*self._start, self.HEIGHT)
        time.sleep(2)

    def fly_to_destination(self):
        curr_position = self._start
        while not self._path_planner.reached_goal(curr_position):
            next_position = self._path_planner.next_step(curr_position)
            self._drone_client.flyToPosition(next_position[0], next_position[1], self.HEIGHT, self.VELOCITY)
            while not self._path_planner.reached_location(curr_position, next_position):
                curr_position = self._drone_client.getPose().pos.x_m, self._drone_client.getPose().pos.y_m
                self._update_obstacles()
            curr_position = next_position

    def _update_obstacles(self):
        sensed_obstacle, lidar_data = self._drone_client.senseObstacle()
        if sensed_obstacle:
            parsed_lidar_data = self._drone_client.parse_lidar_data(lidar_data)
            for lidar_sample in parsed_lidar_data:
                x, y = self._drone_client.getPointInRealWorldCoords(*lidar_sample)
                self._path_planner.process_lidar_data((x, y))
        return

    def fly_to_destination_debug(self):
        curr_position = self._start
        while not self._path_planner.reached_goal(curr_position):
            next_position = self._path_planner.next_step(curr_position)
            self._drone_client.flyToPosition(next_position[0], next_position[1], self.HEIGHT, self.VELOCITY)
            while not self._path_planner.reached_location(curr_position, next_position):
                time.sleep(0.5)
                semi_next_position = self._drone_client.getPose().pos.x_m, self._drone_client.getPose().pos.y_m
                if semi_next_position == curr_position:
                    print(semi_next_position)
                self._update_obstacles_debug()
            curr_position = next_position

    def _update_obstacles_debug(self):
        sensed_obstacle, lidar_data = self._drone_client.senseObstacle()
        if sensed_obstacle:
            parsed_lidar_data = self._drone_client.parse_lidar_data(lidar_data)
            for lidar_sample in parsed_lidar_data:
                x, y = self._drone_client.getPointInRealWorldCoords(*lidar_sample)
                self._path_planner.process_lidar_data((x, y))
        return

def main(start: Tuple, goal: Tuple):
    agent = Agent(start, goal)
    agent.connect_and_spawn()
    agent.fly_to_destination_debug()


if __name__ == '__main__':
    current_start = (-1200.0, -1200.0)
    current_end = (0.0, -600.0)
    main(current_start, current_end)
