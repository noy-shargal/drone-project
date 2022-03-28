from shapely.geometry import Point
from AlgoStateInterface import AlgoStateEnum, AlgoStateInterface
from Config import config
from DroneTypes import Position
from utils import getPointInRealWorldCoords


class AStarState(AlgoStateInterface):

    def __init__(self, agent):
        super().__init__(AlgoStateEnum.ASTAR)
        self._agent = agent

    def enter(self):
        # fly to poistion
        # sense for unknown obsticle - if there is on - exit to APFState
        # if reached goal - exit to EndState
        print("ENTER A-STAR STATE")
        need_fly_command = True
        client = self._agent.client
        path = self._agent.path
        obs = self._agent.obs
        goal = Position()

        cur_pose = self._agent.client.getPose()
        start = (cur_pose.pos.x_m, cur_pose.pos.y_m)

        while True:
            if self._agent.astar_curr_point >= len(self._agent.path):
                return AlgoStateEnum.END
            p = path[self._agent.astar_curr_point].point()
            goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height
            sensing_obstacle, points_list, pose = client.senseObstacle()
            if sensing_obstacle:
                point = Point(points_list[0], points_list[1])
                world_point = getPointInRealWorldCoords(point.x, point.y, pose)
                if not obs.is_point_in_obstacles_map(Point(*world_point)):  # new obstacle
                    print("PADDING POINT: "+str(world_point))
                    return AlgoStateEnum.APF

            if self._agent.astar_curr_point >= len(path) + 1:
                return AlgoStateEnum.END

            if need_fly_command:
                client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.astar_velocity)
                need_fly_command = False
                #print("Flying to point number: " + str(self._agent.astar_curr_point) + str([goal.x_m, goal.y_m, goal.z_m]))

            point = Point(goal.x_m, goal.y_m)
            self._agent.add_path_point(point)
            if self._agent.reached_goal_2D(client.getPose().pos, goal):
                print("Reached goal number : " + str(self._agent.astar_curr_point))
                self._agent.astar_curr_point += 1
                need_fly_command = True
                pos = client.getPose().pos
                if self._agent.astar_curr_point == len(path):
                    print("Reached destination at (" + str(pos.x_m) + ", " + str(pos.y_m) + ") ")
                    return AlgoStateEnum.END

    def exit(self):
        print("EXIT A-STAR STATE")
