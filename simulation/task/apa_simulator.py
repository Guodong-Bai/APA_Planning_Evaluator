import __main__
import sys
import numpy as np
import copy
import json
import os
import shutil

sys.path.append(".")
sys.path.append("..")
sys.path.append("../..")
sys.path.append("../../jupyter/")

sys.path.append("../lib")


import importlib

# def LoadInput():


def UpdatePlanningPybind(path_x_vec, path_y_vec, path_heading_vec,
    planning_module, ego_x, ego_y, ego_heading, obs_x_vec, obs_y_vec, slot_points, ds):
    path_x_vec = []
    path_y_vec = []
    path_heading_vec = []
    module = importlib.import_module("lib." + planning_module)
    PlanningClass = getattr(module, "Planning")  # 确认类名为 'Planning'
    planning_instance = PlanningClass()  # 实例化类
    planning_instance.Init()

    if not planning_instance.UpdateByJson(obs_x_vec, obs_y_vec, 2.4, 6.0, ego_x, ego_y, ego_heading, ds):
        return 0

    path_x_vec = planning_instance.GetPathEle(0)
    path_y_vec = planning_instance.GetPathEle(1)
    path_heading_vec = planning_instance.GetPathEle(2)
    return 1