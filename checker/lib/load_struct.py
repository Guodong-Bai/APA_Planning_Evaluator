import sys
import numpy as np
import math
from scipy.interpolate import interp1d
from scipy.misc import derivative
from lib.load_rotate import *
import ipywidgets



JAC_S811 = 'JAC_S811'
CHERY_T26 = 'CHERY_T26'
CHERY_E0X = 'CHERY_E0X'

def load_car_params_patch_parking(vehicle_type = CHERY_E0X, car_lat_inflation = 0.0):
  if vehicle_type == JAC_S811:
    # for JAC_S811
    car_x = [3.424, 3.624, 3.624, 3.424, 2.177, 2.177, 1.916, 1.916, -0.747, -0.947, -0.947, -0.747, 1.916, 1.916, 2.177, 2.177]
    car_y = [0.945, 0.745, -0.745, -0.945, -0.945, -1.055, -1.055, -0.945, -0.945, -0.745, 0.745, 0.945, 0.945, 1.055, 1.055, 0.945]
    car_x = [3.187, 3.424, 3.624,  3.624,  3.424,  3.187,  2.177,  2.177,  1.977,  1.977, -0.476, -0.798, -0.947, -0.947, -0.798, -0.476, 1.977, 1.977, 2.177, 2.177]
    car_y = [0.945, 0.795, 0.645, -0.645, -0.795, -0.945, -0.945, -1.055, -1.055, -0.945, -0.945, -0.795, -0.645,  0.645,  0.795,  0.945, 0.945, 1.055, 1.055, 0.945]
    wheel_base = 2.7
  elif vehicle_type == CHERY_T26:
    # for CHERY_T26
    car_x = [3.518, 3.718, 3.718, 3.518, 2.092, 2.092, 1.906, 1.906, -0.885, -1.085, -1.085, -0.885, 1.906, 1.906, 2.092, 2.092]
    car_y = [0.9595, 0.7595, -0.7595, -0.9595, -0.9595, -1.092, -1.092, -0.9595, -0.9595, -0.7595, 0.7595, 0.9595, 0.9595, 1.092, 1.092, 0.9595]
    car_x = [3.2980, 3.5800, 3.7180,  3.7180,  3.5800,  3.2980,  2.0920,  2.092,  1.892,  1.8920, -0.6020, -0.9970, -1.0850, -1.085, -0.9970, -0.6020, 1.892,  1.892, 2.092, 2.092]
    car_y = [0.9595, 0.8095, 0.6595, -0.6595, -0.8095, -0.9595, -0.9595, -1.092, -1.092, -0.9595, -0.9595, -0.8095, -0.6595, 0.6595,  0.8095,  0.9595, 0.9595, 1.092, 1.092, 0.9595]
    wheel_base = 2.796
  elif vehicle_type == CHERY_E0X:
    # for CHERY_E0X
    # car_x = [3.725, 3.925, 3.925, 3.725, 2.235, 2.235, 2.035, 2.035, -0.825, -1.025, -1.025, -0.825, 2.035, 2.035, 2.235, 2.235]
    # car_y = [0.9875, 0.7875, -0.7875, -0.9875, -0.9875, -1.1145, -1.1145, -0.9875, -0.9875, -0.7875, 0.7875, 0.9875, 0.9875, 1.1145, 1.1145, 0.9875]
    # car_x = [3.4655, 3.7711, 3.9250,  3.9250,  3.7711,  3.4655,  2.235,   2.235,   2.035,   2.035,  -0.5150, -0.8653, -1.0250, -1.0250, -0.8653, -0.5150, 2.0350, 2.0350, 2.2350, 2.2350]
    # car_y = [0.9875, 0.8375, 0.6875, -0.6875, -0.8375, -0.9875, -0.9875, -1.1145, -1.1145, -0.9875, -0.9875, -0.8375, -0.6875,  0.6875,  0.8375,  0.9875, 0.9875, 1.1145, 1.1145, 0.9875]
    car_x = [3.5815, 3.8330, 3.9250,  3.9250,  3.8330,  3.5815,  2.2350,  2.2350,  2.0350,  2.0350, -0.4690, -0.8960, -1.0250, -1.0250, -0.8960, -0.4690, 2.0350, 2.0350, 2.2350, 2.2350]
    car_y = [0.9875, 0.6755, 0.2545, -0.2545, -0.6755, -0.9875, -0.9875, -1.1145, -1.1145, -0.9875, -0.9875, -0.8617, -0.4696,  0.4696,  0.8617,  0.9875, 0.9875, 1.1145, 1.1145, 0.9875]
    wheel_base = 3.0

  for i in range(len(car_x)):
    if car_y[i] > 0.0:
      car_y[i] = car_y[i] + car_lat_inflation
    else:
      car_y[i] = car_y[i] - car_lat_inflation

  return car_x, car_y, wheel_base

def load_car_circle_coord():
  # circle_x = [3.3, 3.3, 2.0, -0.6, -0.6, 2.0, 2.7, 1.8, 0.9, 0.0]
  # circle_y = [-0.55, 0.55, 0.85, 0.55, -0.55, -0.85, 0.0, 0.0, 0.0, 0.0]
  # circle_r = [0.35, 0.35, 0.25, 0.35, 0.35, 0.25, 0.95, 0.95, 0.95, 0.95]
  # circle_x = [1.35, 3.2, 3.2, 2.02, -0.55, -0.55, 2.02, 2.7, 1.8, 0.9, 0.0]
  # circle_y = [0.0, -0.5, 0.5, 0.95, 0.5, -0.5, -0.95, 0.0, 0.0, 0.0, 0.0]
  # circle_r = [2.4, 0.35, 0.35, 0.18, 0.35, 0.35, 0.18, 0.95, 0.95, 0.95, 0.95]

  circle_x = [1.45, 3.4650, 3.4650, 2.12, -0.5650, -0.5650, 2.12, 2.9375, 1.8, 0.9, -0.0375]
  circle_y = [0.0, 0.5275, -0.5275, -0.9345, -0.5275, 0.5275, 0.9345, 0.0, 0.0, 0.0, 0.0]
  circle_r = [2.58, 0.46, 0.46, 0.18, 0.46, 0.46, 0.18, 0.9875, 0.9875, 0.9875, 0.9875]

  return circle_x, circle_y, circle_r

def load_car_uss_patch(vehicle_type = JAC_S811):
  if vehicle_type == JAC_S811:
    # for JAC_S811
    apa_x = [3.187342, 3.424531, 3.593071, 3.593071, 3.424531, 3.187342,
            -0.476357, -0.798324, -0.879389, -0.879389, -0.798324, -0.476357]
    apa_y = [0.887956, 0.681712, 0.334651, -0.334651, -0.681712, -0.887956,
            -0.887956, -0.706505, -0.334845, 0.334845, 0.706505, 0.887956]
  elif vehicle_type == CHERY_T26:
   # for CHERY_T26
    apa_x = [3.298241, 3.580141, 3.667435, 3.667435, 3.580141, 3.298241,
            -0.602483, -0.997449, -1.06219, -1.06219, -0.997449, -0.602483]
    apa_y = [0.935328, 0.680863, 0.334976, -0.334976, -0.680863, -0.935328,
            -0.935328, -0.669815, -0.299949, 0.299949, 0.699815, 0.935328]
  elif vehicle_type == CHERY_E0X:
    apa_x = [3.4655,  3.7711,  3.8742, 3.8742, 3.7711,  3.4655,
            -0.5150, -0.8653, -1.0115, -1.0115, -0.8653, -0.5150]
    apa_y = [0.97620,  0.69918,  0.32000,  -0.32000, -0.66918, -0.97620,
            -0.9583, -0.8314, -0.3250, 0.3250,  0.8314,  0.9583]

  return apa_x, apa_y

def load_uss_angle_patch(vehicle_type = JAC_S811):
  if vehicle_type == JAC_S811:
    # for JAC_S811
    uss_angle = [170, 130, 92, 88, 50, 8, 352, 298, 275, 264, 242, 187]
  elif vehicle_type == CHERY_T26:
    # for CHERY_T26
    uss_angle = [169.998, 125.019, 97.046, 82.954, 54.981, 10.002,354.78, 298.086, 277.369, 262.631, 241.9914, 185.22]
  elif vehicle_type == CHERY_E0X:
    # for CHERY_E0X
    uss_angle = [169.998, 125.019, 97.046, 82.954, 54.981, 10.002,354.78, 298.086, 277.369, 262.631, 241.9914, 185.22]

  return uss_angle



def load_car_box(path_x_vec, path_y_vec, path_theta_vec, car_xb, car_yb):
  car_box_x_vec = []
  car_box_y_vec = []
  for k in range(0, len(path_x_vec)):
    car_xn = []
    car_yn = []
    for i in range(len(car_xb)):
      tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], path_x_vec[k], path_y_vec[k], path_theta_vec[k])
      car_xn.append(tmp_x)
      car_yn.append(tmp_y)
    car_box_x_vec.append(car_xn)
    car_box_y_vec.append(car_yn)
  return car_box_x_vec, car_box_y_vec

def load_ego_car_corner_path(path_x_vec, path_y_vec, path_heading_vec, car_local_x, car_local_y):
  corner_path_x_vec = []
  corner_path_y_vec = []
  for i in range(len(path_x_vec)):
    tmp_x, tmp_y = local2global( car_local_x, car_local_y, path_x_vec[i], path_y_vec[i], path_heading_vec[i])
    corner_path_x_vec.append(tmp_x)
    corner_path_y_vec.append(tmp_y)
  return corner_path_x_vec, corner_path_y_vec

def load_ego_car_given_corner_path(path_x_vec, path_y_vec, path_heading_vec, car_local_x, car_local_y, idx_vec):
  corner_path_x_vec = []
  corner_path_y_vec = []
  for idx in idx_vec:
    tmp_x_vec, tmp_y_vec = load_ego_car_corner_path(path_x_vec, path_y_vec, path_heading_vec, car_local_x[idx], car_local_y[idx])
    corner_path_x_vec.append(tmp_x_vec)
    corner_path_y_vec.append(tmp_y_vec)
  return corner_path_x_vec, corner_path_y_vec

def load_ego_car_box(ego_x, ego_y, ego_heading, car_xb, car_yb):
  car_xn = []
  car_yn = []
  for i in range(len(car_xb)):
    tmp_x, tmp_y = local2global(car_xb[i], car_yb[i], ego_x, ego_y, ego_heading)
    car_xn.append(tmp_x)
    car_yn.append(tmp_y)
  return car_xn, car_yn