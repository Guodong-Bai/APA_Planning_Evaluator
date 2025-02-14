import __main__
import sys
import time
import numpy as np
import copy
import json
import os
import shutil
import importlib

sys.path.append(".")
sys.path.append("..")
sys.path.append("../..")
sys.path.append("../lib")
sys.path.append("../../jupyter/")

import time

class PlanningUpdater:
    def __init__(self, planning_module):
        # 动态加载模块
        self.planning_module = importlib.import_module(planning_module)
        self.path_x_vec = []
        self.path_y_vec = []
        self.path_heading_vec = []

        self.success = False
        self.computation_time = 0.0
        self.path_length = 0.0
        self.escape_heading = 0.0
        self.gear_shift_cnt_slot = 0
        self.total_gear_shift_cnt = 0


    def update_planning(self, ego_x, ego_y, ego_heading_rad, obs_x_vec, obs_y_vec, slot_points, ds):
        self.success = False
        self.planning_module.Init()

        start_time = time.perf_counter()
        if self.planning_module.Preprocess(obs_x_vec, obs_y_vec, 2.2, 6.0, ego_x, ego_y, ego_heading_rad, ds):
            if self.planning_module.Update():
                self.success = True
                end_time = time.perf_counter()
                self.computation_time = (end_time - start_time) * 1000

                self.path_x_vec = self.planning_module.GetPathEle(0)
                self.path_y_vec = self.planning_module.GetPathEle(1)
                self.path_heading_vec = self.planning_module.GetPathEle(2)
        return self.success

ego_x = 1.0
ego_y = 2.0
ego_heading = 0.0
obs_x_vec = []
obs_y_vec = []
slot_points =[]

planner = PlanningUpdater("lib.parallel_planning_py")

success = planner.update_planning(ego_x, ego_y, ego_heading, obs_x_vec, obs_y_vec, slot_points, 0.02)
print("success = ", success)
print("path_x_vec = ", planner.path_x_vec)
print("path_y_vec = ", planner.path_y_vec)
print("computation_time_ms = ", planner.computation_time)
