import math
import numpy as np

class coord_transformer:
    def __init__(self, cur_pos_xn = 0.0, cur_pos_yn = 0.0, cur_yaw = 0.0):
        self.set_info(cur_pos_xn, cur_pos_yn, cur_yaw)

    def set_info(self, cur_pos_xn, cur_pos_yn, cur_yaw):
        self.cur_pos_xn = cur_pos_xn
        self.cur_pos_yn = cur_pos_yn
        self.cur_yaw = cur_yaw
        self.cur_yaw_cos = math.cos(cur_yaw)
        self.cur_yaw_sin = math.sin(cur_yaw)

    ## avoid calc sin or cos of original pose repeatedly
    def local_to_global(self, local_x_vec, local_y_vec):
        global_x_vec = []
        global_y_vec = []
        tmp_x = 0.0
        tmp_y = 0.0
        for i in (range(len(local_x_vec))):
            x_rotated = local_x_vec[i] * self.cur_yaw_cos - local_y_vec[i] * self.cur_yaw_sin
            y_rotated = local_x_vec[i] * self.cur_yaw_sin + local_y_vec[i] * self.cur_yaw_cos
            x_rotated += self.cur_pos_xn
            y_rotated += self.cur_pos_yn
            global_x_vec.append(x_rotated)
            global_y_vec.append(y_rotated)
        return global_x_vec, global_y_vec

    def global_to_local(self, global_x_vec, global_y_vec):
        local_x_vec = []
        local_y_vec = []
        for i in (range(len(global_x_vec))):
            x1 = global_x_vec[i] - self.cur_pos_xn
            y1 = global_y_vec[i] - self.cur_pos_yn
            tmp_x = x1 * self.cur_yaw_cos - y1 * self.cur_yaw_sin
            tmp_y = x1 * self.cur_yaw_sin + y1 * self.cur_yaw_cos
            local_x_vec.append(tmp_x)
            local_y_vec.append(tmp_y)
        return local_x_vec, local_y_vec

def rotate(x, y, theta):
    x_rotated = x * math.cos(theta) - y * math.sin(theta)
    y_rotated = x * math.sin(theta) + y * math.cos(theta)
    return x_rotated, y_rotated

def local2global(x, y, ox, oy, otheta):
    tx, ty = rotate(x, y, otheta)
    return (tx+ox, ty+oy)

def global2local(x, y, ox, oy, otheta):
    x1 = x-ox
    y1 = y-oy
    tx, ty = rotate(x1, y1, -otheta)
    return (tx, ty)


def load_car_box(path_x_vec, path_y_vec, path_theta_vec, car_xb, car_yb):
  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(len(path_x_vec)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], path_x_vec[k], path_y_vec[k], path_theta_vec[k])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)
  return car_box_x_vec, car_box_y_vec
