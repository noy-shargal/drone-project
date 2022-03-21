import math

from shapely.geometry import Point


def get_point_in_polar_coords(rel_x, rel_y):
    r = math.sqrt(rel_x * rel_x + rel_y * rel_y)
    theta = math.atan2(rel_y, rel_x)
    return r, theta


def get_point_in_polar_degrees_coords(rel_x, rel_y):
    r = math.sqrt(rel_x * rel_x + rel_y * rel_y)
    theta = math.atan2(rel_y, rel_x)
    degrees = theta * 180 / math.pi
    return r, degrees


def get_parallelogram_missing_point(curr_position: Point, vr: Point, vl: Point, parallel_to_point=None):
    assert parallel_to_point is not None

    diff_x = None
    diff_y = None

    if parallel_to_point == "VL":
        diff_x = vl.x - vr.x
        diff_y = vl.y - vr.y

    elif parallel_to_point == "VR":
        diff_x = vr.x - vl.x
        diff_y = vr.y - vl.y

    assert diff_x is not None
    new_x = curr_position.x + diff_x
    new_y = curr_position.y + diff_y
    return Point(new_x, new_y)
