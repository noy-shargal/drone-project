import math


def get_point_in_polar_coords(rel_x, rel_y):
    r = math.sqrt(rel_x * rel_x + rel_y * rel_y)
    theta = math.atan2(rel_y, rel_x)
    return r, theta