import math
import numpy as np
from math import fmod, pi, fabs, sqrt

class LineSegment:
  def __init__(self, x0, y0, x1, y1):
    self.pA = np.array([x0, y0])
    self.pB = np.array([x1, y1])
    self.length = np.linalg.norm(self.pA - self.pB)

def normalize_angle(angle):
  a = fmod(angle + pi, 2.0 * pi)
  if a < 0.0:
    a += 2.0 * pi
  return a - pi

def is_in_bound(u, a, b):
   return u >= min(a, b) and u <= max(a, b)

def is_double_equal(a, b, eps = 1e-8):
    return fabs(a - b) < eps

def norm_square_of_np_array(array):
    if not isinstance(array, np.ndarray):
        raise ValueError("Input must be a NumPy array!")
    return np.dot(array, array)

def get_unit_vector(vector):
    norm = np.linalg.norm(vector)
    if norm <= 1e-9:
        raise ValueError("Cannot normalize a zero vector")
    return vector / norm


def cal_two_points_dist_square(x0, y0, x1, y1):
  return (x0 - x1) ** 2 + (y0 - y1) ** 2

def cal_two_points_dist(x0, y0, x1, y1):
  return sqrt(cal_two_points_dist_square(x0, y0, x1, y1))

def cal_point_to_line_dist_square(point, line):
    if not isinstance(point, np.ndarray) or not isinstance(line.pA, np.ndarray) or not isinstance(line.pB, np.ndarray):
        raise ValueError("Input must be NumPy arrays!")

    if point.shape != line.pA.shape or point.shape != line.pB.shape:
        raise ValueError("Shapes of inputs must match!")

    v_AB = line.pB - line.pA
    v_AO = point - line.pA
    v_AO_dot_v_AB = np.dot(v_AO, v_AB)
    v_AB_norm_square = norm_square_of_np_array(v_AB)

    if v_AB_norm_square < 1e-8:
       return 0.0

    return norm_square_of_np_array(v_AO) - (v_AO_dot_v_AB **2) / v_AB_norm_square

def cal_point_to_line_dist(point, line):
   return sqrt(cal_point_to_line_dist_square(point, line))

def cal_point_to_line_seg_dist(point, line):
    if not isinstance(point, np.ndarray) or not isinstance(line.pA, np.ndarray) or not isinstance(line.pB, np.ndarray):
        raise ValueError("type error: not np.array type!")

    if point.shape != line.pA.shape or point.shape != line.pB.shape:
        raise ValueError("shape must be equal!")

    if is_double_equal(line.length, 1e-6):
        raise ValueError("length is too short")

    unit_AB = get_unit_vector(line.pB - line.pA)
    unit_BA = -unit_AB

    unit_AO = get_unit_vector(point - line.pA)
    unit_BO = get_unit_vector(point - line.pB)

    cos_OAB = np.dot(unit_AB, unit_AO)
    cos_OBA = np.dot(unit_BA, unit_BO)

    dist = 0.0
    if is_in_bound(cos_OAB, 1e-6, 1 - 1e-6) and is_in_bound(cos_OBA, 1e-6, 1 - 1e-6):
        dist = cal_point_to_line_dist(point, line)
    elif is_double_equal(cos_OAB, 1.0) and is_double_equal(cos_OBA, 1.0):
        dist = 0.0
    else:
        dist = min(np.linalg.norm(line.pB - point), np.linalg.norm(line.pA - point))

    return dist


def circle_from_points(p1, p2, p3, tol=1e-9):

    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3

    d = 2 * (x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2))

    if abs(d) < tol:
        raise ValueError("三点共线或近似共线，无法构成一个唯一的圆")

    ux = ((x1**2 + y1**2)*(y2 - y3) +
          (x2**2 + y2**2)*(y3 - y1) +
          (x3**2 + y3**2)*(y1 - y2)) / d
    uy = ((x1**2 + y1**2)*(x3 - x2) +
          (x2**2 + y2**2)*(x1 - x3) +
          (x3**2 + y3**2)*(x2 - x1)) / d


    radius = math.sqrt((ux - x1)**2 + (uy - y1)**2)

    return (ux, uy), radius

if __name__ == '__main__':
    # 示例：定义三个点
    p1 = (0, 0)
    p2 = (1, 0)
    p3 = (0, 1)

    try:
        center, radius = circle_from_points(p1, p2, p3)
        print("圆心坐标为:", center)
        print("圆的半径为:", radius)
    except ValueError as e:
        print(e)
# # for test
# if __name__ == "__main__":
#     point = np.array([0.5,1])
#     line = LineSegment(0, 0, 1, 1)
#     print("line = ", line.length)
#     print("line = ", line.pA)
#     print("line = ", line.pB)
#     print("dist = ", cal_point_to_line_seg_dist(point, line))