from shapely.geometry import Point


class sim_config:

    def __init__(self):

        self.source = Point(-1200.0, -1200)
        self.destination = Point(0.0, -600.0)
        self.height = -30
        self.velocity = 7
        self.ltf_velocity = 3
        self.load_from_json = True



