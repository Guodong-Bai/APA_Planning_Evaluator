import sys
import numpy as np
import math
from scipy.interpolate import interp1d
from scipy.misc import derivative
from lib.load_rotate import *
import ipywidgets

def isINJupyter():
    try:
        __file__
    except NameError:
        return True
    else:
        return False

def binary_search(in_list, target):
  low = 0
  high = len(in_list) - 1
  while low <= high:
      mid = (low + high) // 2
      if in_list[mid] == target:
          return mid
      elif in_list[mid] > target:
          high = mid - 1
      else:
          low = mid + 1
  return -1

def find_nearest(msg, bag_time, find_json = False):
  if msg['enable']  == True:
    msg_idx = 0
    while msg['t'][msg_idx] <= bag_time and msg_idx < (len(msg['t'])-2):
      msg_idx = msg_idx + 1
    if find_json:
      return msg['json'][msg_idx]
    else:
      return msg['data'][msg_idx]
  else:
    return None

def load_car_params_patch():
  car_x = [3.518, 3.718, 3.718, 3.518, 2.092, 2.092, 1.906, 1.906, -0.885, -1.085, -1.085, -0.885, 1.906, 1.906, 2.092, 2.092]
  car_y = [0.9595, 0.7595, -0.7595, -0.9595, -0.9595, -1.1095, -1.1095, -0.9595, -0.9595, -0.7595, 0.7595, 0.9595, 0.9595, 1.1095, 1.1095, 0.9595]
  return car_x, car_y

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