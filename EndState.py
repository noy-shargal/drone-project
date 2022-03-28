from shapely.geometry import Point

from AlgoStateInterface import AlgoStateEnum

from AlgoStateInterface import AlgoStateInterface
from MapDrawer import MapDrawer


class EndState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.END)
        self._agent = agent

    def enter(self):
        print("ENTER END STATE")
        self._agent.show_real_path()
        map = self._agent._apf_path_planner._obstacles_map.get_map()
        size_x = self._agent._apf_path_planner._obstacles_map._size_x
        size_y = self._agent._apf_path_planner._obstacles_map._size_y
        md = MapDrawer()
        for i in range(size_x):
            for j in range(size_y):
                if map[i, j] == 2:
                    md.set_point(Point(i, j))
        md.show()

        print("THE END !!!")


        return AlgoStateEnum.TERMINAL_STATE

    def exit(self):
        return
