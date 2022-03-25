from AlgoStateInterface import AlgoStateInterface, AlgoStateEnum


class APFState(AlgoStateInterface):

    def __init__(self):
        super().__init__(AlgoStateEnum.APF)


    def enter(self):
        print("ENTER APF STATE")

        client.flyToPosition(cur_pos.pos.x_m, cur_pos.pos.y_m, cur_pos.pos.z_m, 0.1)
        tuple_goal = (goal.x_m, goal.y_m)
        is_new_unkmown_obstacle = True
        while is_new_unkmown_obstacle:
            self.apf_fly_to_destination(tuple_goal)
            point_num += 1
            p = self._path[point_num].point()
            goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
            tuple_goal = (goal.x_m, goal.y_m)
            is_new_unkmown_obstacle = client.isNewObstaclefullSenseObstacle(self._obs)


    def exit(self, next_state):
        print("EXIT APF STATE")