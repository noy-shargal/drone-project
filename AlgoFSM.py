from AStarState import AStarState
from APFState import APFState
from EndState import EndState
from TransistionState import TransistionState
from AlgoStateInterface import AlgoStateEnum


class AlgoFSM:

    def __init__(self, agent, init_state_enum:AlgoStateEnum, end_state_enum:AlgoStateEnum):
        astar_state = AStarState(agent)
        apf_state = APFState(agent)
        end_state = EndState(agent)
        transition_state = TransistionState(agent)

        self._states = {AlgoStateEnum.ASTAR: astar_state,
                        AlgoStateEnum.APF: apf_state,
                        AlgoStateEnum.END: end_state,
                        AlgoStateEnum.TRANSISTION: transition_state}

        self._init_state_enum = init_state_enum
        self._end_state_enum = end_state_enum

        self._curr_state = None

    def init_state_enum(self):
        return self._init_state_enum

    def change_state(self, next_state_enum:AlgoStateEnum):
        if self._curr_state is not None:
            self._curr_state.exit()
        self._curr_state = self._states[next_state_enum]
        next_state_enum = self._curr_state.enter()
        return next_state_enum

    def get_current_state(self):
        return self._curr_state



