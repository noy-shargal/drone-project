from enum import unique, Enum


@unique
class AlgoStateEnum(Enum):
    ASTAR = 1
    APF = 2
    TRANSISTION = 3
    END = 4

class AlgoStateInterface:

    def __init__(self, state_enum):
        self._state_enum = state_enum

    def enter(self):
        pass

    def exit(self, next_state):
        pass

    def state_enum(self):
        return self._state_enum
