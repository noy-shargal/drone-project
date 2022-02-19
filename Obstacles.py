import csv



class Obstacle:

    def __init__(self,  num_of_points, x_meter, y_meter, z_meter, obs_type):

        self.num_of_points = num_of_points
        self.x_meter = x_meter
        self.y_meter = y_meter
        self.z_meter = z_meter
        self.obs_type = obs_type

    def get(self):
        return self.num_of_points, self.x_meter, self.y_meter, self.z_meter, self.obs_type


class Obstacles:

    def __init__(self):
        self._points_map = dict()

    def get_obstacle(self, id):
        if self._points_map[id] is None:
            self._points_map[id] = list()
        return  self._points_map[id]
    def read(self):
        with open('obstacles_100m_above_sea_level.csv', newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                obs = self.get_obstacle(row[0])

                obs = Obstacle(*row)
                key = row[1], row[2], row[3]
                self._points_map[key] = obs

    def print(self):

        for item in self._points_map.keys():
            print(item)

    def is_position_blocked(self, x, y, z):

        key = x, y, z
        return  self._points_map[key] is not None

    def add_obstacle(self, x, y, z):

        self._points_map[x, y, z] = Obstacle(None, x, y, z, None)