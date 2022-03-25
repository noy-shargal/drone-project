from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum

from AlgoStateInterface import AlgoStateInterface


class EndState(AlgoStateInterface):

    def __init__(self):
        super().__init__(AlgoStateEnum.ASTAR)

    def enter(self):
        print("ENTER END STATE")

    def exit(self, next_state):
        assert next_state is None

