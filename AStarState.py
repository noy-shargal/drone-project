from shapely.geometry import Point

from AlgoStateInterface import AlgoStateEnum, AlgoStateInterface
from APFState import APFState
from Agent import AlgoState
from Config import config
from DroneTypes import Position
from EndState import EndState
from MyDroneClient import MyDroneClient
from Obstacles import Obstacles

from utils import getPointInRealWorldCoords


class AStarState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.ASTAR)
        self._agent = agent

    def enter(self):
        # fly to poistion
        # sense for unknown obsticle - if there is on - exit to APFState
        # if reached goal - exit to EndState
        print("ENTER A-STAR STATE")
        point_num = 1
        need_fly_command = True
        client = self._agent.client
        path = self._agent.path
        obs = self._agent.obs
        goal = Position()

        while True:

            p = path[self._agent.astar_curr_point].point()
            goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
            sensing_obstacle, points_list, pose = client.senseObstacle()
            if sensing_obstacle:
                point = Point(points_list[0], points_list[1])
                world_point = getPointInRealWorldCoords(point.x, point.y, pose)
                if not obs.is_point_in_obstacles_map(Point(*world_point)):  # new obstacle
                    apf_state = APFState()
                    return apf_state

            if self._agent.astar_curr_point >= len(path) + 1:
                return self.exit(EndState())

            if need_fly_command:
                client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.astar_velocity)
                need_fly_command = False
                print("Flying to point number: " + str(point_num) + str([goal.x_m, goal.y_m, goal.z_m]))

            if self._agent.reached_goal_2D(client.getPose().pos, goal):
                print("Reached goal number : " + str(self._agent.astar_curr_point))
                self._agent.astar_curr_point += 1
                need_fly_command = True
                pos = client.getPose().pos

                if self._agent.astar_curr_point == len(path):
                    print("Reached destination at (" + str(pos.x_m) + ", " + str(pos.y_m) + ") ")
                    return self.exit(EndState())

    def exit(self, next_state):
        print("EXIT A-STAR STATE")
        return next_state
