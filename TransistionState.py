from  AlgoState import AlgoStateEnum, AlgoState

from AlgoStateInterface import AlgoStateInterface


class TransistionState(AlgoStateInterface):

    def __init__(self):
        super().__init__(AlgoStateEnum.ASTAR)

    def enter(self):
        # full 360 scan
        # if no unknown obstacled -
            # next_mode = ASTAR
        # else
            # next_mode = APF
        # fly to poistion
        # sense for unknown obsticle - f there is on - on_exit_mode
        pass

    def exit(self):
        # change mode to next_mode
        pass
