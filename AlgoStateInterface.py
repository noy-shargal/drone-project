from enum import unique, Enum


@unique
class AlgoStateEnum(Enum):
    ASTAR = 1
    APF = 2
    TRANSISTION = 3
    LOCAL_MINIMA = 4
    END = 5

class AlgoStateInterface:

    def __init__(self, state_enum):
        self._state_enum = state_enum

    def enter(self):
        pass

    def exit(self):
        return

    def state_enum(self):
        return self._state_enum
