import time
from enum import Enum

from shapely.geometry import Point

from DroneTypes import Position
from MyDroneClient import MyDroneClient
from PathPlanner import PathPlanner
from BasicTangentBug import BasicTangentBug
from Config  import config
from utils import get_parallelogram_missing_point

class AlgoType(Enum):
    ASTAR = 1
    TANGET_BUG_TARGET = 2
    TANGET_WALKING_WALL = 3
    FREE_SPACE_RE_ASTAR = 4

class Events (Enum):
    UNKNOWN_OBSTACLE = 1
    LOCAL_MINIMA = 2
    FREE_SPACE = 3


class Agent:

    def __init__(self):
        self._path_planner = PathPlanner()
        self._client = MyDroneClient()

        self._algo_type = AlgoType.ASTAR

        # self._lidar_points_counter = Countdowner(5.0)
        # self._lidar_points = list()
        self._path = self._path_planner.get_path()

    def change_algo_type(self, event):
        if self._algo_type == AlgoType.ASTAR and event == Events.UNKNOWN_OBSTACLE:
            self._algo_type = AlgoType.TANGET_BUG_TARGET
            return

        if self._algo_type == AlgoType.TANGET_BUG_TARGET and event == Events.LOCAL_MINIMA:
            self._algo_type = AlgoType.TANGET_WALKING_WALL
            return

        if self._algo_type == AlgoType.TANGET_WALKING_WALL and event == Events.FREE_SPACE:
            self._algo_type = AlgoType.ASTAR
            return


    def connect_and_spawn(self):
        print("Connecting.....")
        self._client.connect()
        time.sleep(2)
        self._client.setAtPosition(config.source.x, config.source.y , config.height)
        time.sleep(2)
        print(self._client.isConnected())
        time.sleep(2)


    def fly_to_destination(self):

        print("Init position " + str([config.source.x, config.source.y, config.height]))
        prev_point_num = 0
        point_num = 1
        need_fly_command = True
        real_path = list()
        tb = BasicTangentBug()
        client = self._client

        goal = Position()

        while True:

            lidar_data = client.getLidarData()


            if self._algo_type == AlgoType.ASTAR:
                p = self._path[point_num].point()
                goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height

            new_obstacle_points = 0

            if need_fly_command:
                client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.velocity)
                need_fly_command = False
                print("Flying to point number: " + str(point_num) + str([goal.x_m, goal.y_m, goal.z_m]))

            if client.reached_goal_2D_pos(client.getPose().pos, goal):
                if self._algo_type == AlgoType.ASTAR:
                    point_num += 1
                print("Reached goal number : " + str(point_num))
                prev_point_num = point_num

                need_fly_command = True
                pos = client.getPose().pos
                real_path.append(client.position_to_point(pos))
                if point_num == len(self._path):
                    print("Reached destination at (" + str(client.getPose().pos.x_m) + ", " + str(
                        client.getPose().pos.y_m) + ") ")
                    break

            sensing_obstacle, points_list, pose = client.senseObstacle()
            if sensing_obstacle and self._algo_type != AlgoType.TANGET_WALKING_WALL:

                # print ("sensed obstacle : "+str(points_list), str(pose))
                xw, yw = client.getPointInRealWorldCoords(points_list[0], points_list[1], client.getPose())
                # print("getPointInRealWorldCoords -> ", "("+str(xw) +", "+ str(yw)+ ")")
                # print ("Drone location : (", str(client.getPose().pos.x_m), ", "+str(client.getPose().pos.y_m)+")")
                is_known_obs = self._path_planner.is_point_in_obstacles_map(Point(xw, yw))

                if not is_known_obs:

                    tb.add_point(Point(points_list[0], points_list[1]), Point(xw, yw))
                    if tb.get_num_of_points() > 5 and tb.is_way_blocked():
                        #client.stop()
                        self.change_algo_type(Events.UNKNOWN_OBSTACLE)
                        curr_pos = Point(client.getPose().pos.x_m, client.getPose().pos.y_m)
                        tb.set_current_position(curr_pos)
                        next_goal = Point(goal.x_m, goal.y_m)
                        tb.set_target(next_goal)
                        tb.build_ltg()
                        sg = tb.build_sub_graph()
                        #closest_point = sg.get_closet_point_to_target()
                        closest_point, _= tb.get_closest_endpoint_to_target(curr_pos)
                        # client.flyToPosition(closest_point.x, closest_point.y, config.height, config.ltf_velocity)
                        goal.x_m, goal.y_m, goal.z_m = closest_point.x, closest_point.y, config.height
                        need_fly_command = True
                        if sg.is_source_local_minima():
                            self.change_algo_type(Events.LOCAL_MINIMA)
                            print("Reached Local Minima")
                            point, v_name = tb.get_closest_endpoint_to_target(curr_pos)
                            vr = tb.get_vr()
                            vl = tb.get_vl()
                            par_pnt = get_parallelogram_missing_point(curr_pos, vr, vl, v_name)
                            print("Flying Parallel from :"+str(curr_pos)+" to "+str(par_pnt) )
                            # client.flyToPosition(par_pnt.x, par_pnt.y, config.height, config.ltf_velocity)
                            goal.x_m, goal.y_m, goal.z_m = par_pnt.x, par_pnt.y, config.height
                            need_fly_command = True
                        # time.sleep(0.24)
                        y = 9
                        tb = BasicTangentBug()

                    # print("sensed obstacle : " + str(points_list), str(pose))
                    # print("getPointInRealWorldCoords -> ", "(" + str(xw) + ", " + str(yw) + ")")
                    # print("Drone location : (", str(client.getPose().pos.x_m), ", " + str(client.getPose().pos.y_m),
                    #       ", " + str(client.getPose().pos.z_m) + ")")
                    # print("unknown obstacle !")

    @property
    def client(self):
        return self._client


