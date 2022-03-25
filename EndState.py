from AlgoStateInterface import AlgoStateEnum

from AlgoStateInterface import AlgoStateInterface


class EndState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.ASTAR)
        self._agent = agent

    def enter(self):
        print("ENTER END STATE")

    def exit(self):
        return
