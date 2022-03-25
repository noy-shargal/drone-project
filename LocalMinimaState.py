from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum


class LocalMinimaState(AlgoStateInterface):
    def __init__(self, agent):
        super().__init__(AlgoStateEnum.LOCAL_MINIMA)
        self._agent = agent

    def enter(self):
        print("ENTER LOCAL MINIMA STATE")
        return AlgoStateEnum.TRANSISTION

    def exit(self):
        print("EXIT LOCAL MINIMA STATE")
