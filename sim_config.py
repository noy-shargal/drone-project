from shapely.geometry import Point


class sim_config:

    def __init__(self):

        self.source = Point(-1250.0, -830.0)
        self.destination = Point(0.0, -600.0)
        self.height = -100
        self.velocity = 7
        self.load_from_json = True



