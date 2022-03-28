import math

from shapely.geometry import Point


class Vector:
    def __init__(self, p1: Point, p2: Point):
        self._x = p2.x - p1.x
        self._y = p2.y - p1.y

    def normalize(self):
        norm = self.get_norm()
        self._x /= norm
        self._y /= norm

    def get_norm(self):
        return math.sqrt(self._x ** 2 + self._y ** 2)

    def multiple_by_scalar(self, scalar):
        self._x *= scalar
        self._y *= scalar

    def get_direction(self):
        return self._x, self._y

    def inner_product(self, other):
        return self._x * other._x + self._y * other._y

    def remove_vector(self, other):
        self._x -= other._x
        self._y -= other._y

    def opposite(self):
        return Vector(Point(self._x, self._y), Point(0, 0))

    def remove_projection(self, other):
        projection_scalar = self.inner_product(other) / self.get_norm()
        projection_vector = Vector(Point(0, 0), Point(self._x, self._y))
        projection_vector.multiple_by_scalar(projection_scalar)

        output = Vector(Point(0, 0), Point(other._x, other._y))
        output.remove_vector(projection_vector)
        output.normalize()
        return output

    def get_as_point(self):
        return Point(self._x, self._y)

    def get_angle(self):
        return math.atan2(self._y, self._x) * 180 / math.pi

    def __str__(self):
        return str(self._x) + '_' + str(self._y)