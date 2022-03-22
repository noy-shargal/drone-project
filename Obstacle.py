from shapely.geometry import LineString


class ThinWallObstacle:
    def __init__(self, first_endpoint, second_endpoint):
        self._first_endpoint = first_endpoint
        self._second_endpoint = second_endpoint
        self._line = LineString([self._first_endpoint, self._second_endpoint])

    def intersects_line(self, line :LineString)
        return line.intersects(self._line)


