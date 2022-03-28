from shapely.geometry import Point

from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum
from Config import config
from MyDroneClient import MyDroneClient
from SmartAgent_v1 import SmartAgent_v1


class FlyToGoalState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.FLY_TO_GOAL)
        self._agent = agent

    def enter(self):
        print("ENTER FlyToGoalState STATE")
        curr = self._agent.client.flyToPosition(config.x, config.y, config.height, 5)
        while not self._agent.point_reached_goal_2D(Point(*curr), Point(*self._agent.goal)):
            curr = self._agent.client.getPose().pos.x_m, self._agent.client.getPose().pos.y_m
        return AlgoStateEnum.END

    def exit(self):
        print("EXIT FlyToGoalState STATE")
        return

    def _stop(self):
        self._agent.client.stop()
        return

