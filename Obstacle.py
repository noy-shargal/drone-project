from shapely.geometry import LineString, Point


class ThinWallObstacle:
    def __init__(self, first_endpoint, second_endpoint):
        self._first_endpoint = first_endpoint
        self._second_endpoint = second_endpoint
        self._line = LineString([self._first_endpoint, self._second_endpoint])

    def intersects_line(self, line :LineString):
        return line.intersects(self._line)

    def get_closest_point_to_target(self, target: Point):
        d_min1 = target.distance(self._first_endpoint)
        d_min2 = target.distance(self._second_endpoint)
        if d_min1 < d_min2:
            return d_min1, self._first_endpoint
        return d_min2, self._second_endpoint



