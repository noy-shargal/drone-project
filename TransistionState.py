from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum


class TransistionState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.ASTAR)
        self._agent = agent

    def enter(self):
        print("ËNTER Transistion State")
        if self._agent.is_wall_ahead(30):
            return AlgoStateEnum.APF
        return AlgoStateEnum.ASTAR

    def exit(self):
        print("ËXIT Transistion State")
        return