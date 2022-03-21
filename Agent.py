import time

from shapely.geometry import Point

from DroneTypes import Position
from MyDroneClient import MyDroneClient
from PathPlanner import PathPlanner
from TangentBug import TangentBug
from Config  import config
from utils import get_parallelogram_missing_point


class Agent:

    def __init__(self):
        self._path_planner = PathPlanner()
        self._client = MyDroneClient()

        # self._lidar_points_counter = Countdowner(5.0)
        # self._lidar_points = list()
        self._path = self._path_planner.get_path()

    def connect_and_spawn(self):
        self._client.reset()
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

        tb = TangentBug()

        client = self._client

        DEBUG_ltg_count = 0
        while True:

            lidar_data = client.getLidarData()
            goal = Position()
            p = self._path[point_num].point()
            goal.x_m, goal.y_m, goal.z_m = p.x, p.y, config.height

            new_obstacle_points = 0

            if need_fly_command:
                client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, config.velocity)
                need_fly_command = False
                print("Flying to point number: " + str(point_num) + str([goal.x_m, goal.y_m, goal.z_m]))

            if client.reached_goal_2D(client.getPose().pos, goal):
                print("Reached goal number : " + str(point_num))
                prev_point_num = point_num
                point_num += 1
                need_fly_command = True
                pos = client.getPose().pos
                real_path.append(client.position_to_point(pos))
                if point_num == len(self._path):
                    print("Reached destination at (" + str(client.getPose().pos.x_m) + ", " + str(
                        client.getPose().pos.y_m) + ") ")
                    break

            sensing_obstacle, points_list, pose = client.senseObstacle()
            if sensing_obstacle:
                # print ("sensed obstacle : "+str(points_list), str(pose))
                xw, yw = client.getPointInRealWorldCoords(points_list[0], points_list[1], client.getPose())
                # print("getPointInRealWorldCoords -> ", "("+str(xw) +", "+ str(yw)+ ")")
                # print ("Drone location : (", str(client.getPose().pos.x_m), ", "+str(client.getPose().pos.y_m)+")")
                is_known_obs = self._path_planner.is_point_in_obstacles_map()

                if not is_known_obs:

                    tb.add_point(Point(points_list[0], points_list[1]), Point(xw, yw))
                    if tb.get_num_of_points() > 5 and tb.is_way_blocked():
                        client.stop()

                        curr_poss = Point(client.getPose().pos.x_m, client.getPose().pos.y_m)
                        tb.set_current_position(curr_poss)
                        next_goal = Point(goal.x_m, goal.y_m)
                        tb.set_target(next_goal)
                        tb.build_ltg()
                        sg = tb.build_sub_graph()
                        closest_point = sg.get_closet_point_to_target()
                        client.flyToPosition(closest_point.x, closest_point.y, config.height, config.ltf_velocity)
                        if sg.is_source_local_minima():
                            print("Reached Local Minima")
                            point, v_name = tb.get_closest_endpoint_to_target(curr_poss)
                            vr = tb.get_vr()
                            vl = tb.get_vl()
                            par_pnt = get_parallelogram_missing_point(curr_poss, vr, vl, v_name)
                            client.flyToPosition(par_pnt.x, par_pnt.y, config.height, config.ltf_velocity)
                        # time.sleep(0.24)
                        y = 9
                        tb = TangentBug()
                        DEBUG_ltg_count += 1
                        print("tangent bug number : " + str(DEBUG_ltg_count))

                    print("sensed obstacle : " + str(points_list), str(pose))
                    print("getPointInRealWorldCoords -> ", "(" + str(xw) + ", " + str(yw) + ")")
                    print("Drone location : (", str(client.getPose().pos.x_m), ", " + str(client.getPose().pos.y_m),
                          ", " + str(client.getPose().pos.z_m) + ")")
                    print("unknown obstacle !")

    @property
    def client(self):
        return self._client


