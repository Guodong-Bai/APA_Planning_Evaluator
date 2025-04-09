import __main__
import math
import sys
import time
import numpy as np
import copy
import json
import os
import shutil
import importlib

from checker.lib.load_geometry import *
from checker.lib.load_struct import load_car_params_patch_parking

sys.path.append(".")
sys.path.append("..")
sys.path.append("../..")
sys.path.append("../lib")
sys.path.append("../../jupyter/")

import time

kRad2Deg = 180.0 / math.pi
kDeg2Rad = math.pi / 180.0


class PlanningResult:
    def __init__(self):
        self.Reset()

    def Reset(self):
        self.success = False
        self.initial_pose = []
        self.path_x_vec = []
        self.path_y_vec = []
        self.path_heading_vec = []
        self.gear_vec = []
        self.gear_shift_idx_vec = []

        self.path_length = 0.0
        self.escape_heading = 0.0
        self.corner_obstacle_avoidance_dist = 0.0
        self.computation_time = 0.0
        self.gear_shift_cnt_slot = 0
        self.total_gear_shift_cnt = 0
        self.data = {}

    def TransferToJson(self):
        self.data["success"] = self.success
        self.data["initial_pose"] = self.initial_pose
        self.data["path_x_vec"] = self.path_x_vec
        self.data["path_y_vec"] = self.path_y_vec
        self.data["path_heading_vec"] = self.path_heading_vec

        self.data["gear_vec"] = self.gear_vec
        self.data["gear_shift_idx_vec"] = self.gear_shift_idx_vec

        self.data["path_length"] = self.path_length
        self.data["escape_heading"] = self.escape_heading
        self.data["computation_time"] = self.computation_time
        self.data["gear_shift_cnt_slot"] = self.gear_shift_cnt_slot
        self.data["total_gear_shift_cnt"] = self.total_gear_shift_cnt
        self.data["corner_obstacle_avoidance_dist"] = self.corner_obstacle_avoidance_dist
        return self.data

    def PrintInfo(self):
        print("success = ", self.success)
        print("initial_pose = ", self.initial_pose)

        if self.success:
            print("path_length = ", self.path_length)
            print("total_gear_shift_cnt = ", self.total_gear_shift_cnt)

            print("computation_time = ", self.computation_time)

            print("escape_heading deg= ", self.escape_heading * kRad2Deg)
            print(
                "corner_obstacle_avoidance_dist= ",
                self.corner_obstacle_avoidance_dist,
            )

            print("gear_shift_cnt_slot = ", self.gear_shift_cnt_slot)

            print("gear_vec = ", self.gear_vec)
            print("gear_shift_idx_vec = ", self.gear_shift_idx_vec)


class PlanningUpdater:
    def __init__(self, planning_module="parallel_planning_py"):
        self.result = PlanningResult()

        cur_file = os.path.abspath(__file__)
        cur_path = os.path.dirname(cur_file)
        parent_path = os.path.dirname(cur_path)
        lib_path = os.path.join(parent_path, "lib")
        if lib_path not in sys.path:
            sys.path.append(lib_path)
        self.planning_module = importlib.import_module(planning_module)
        print(f"Successfully imported {planning_module}")

    def update_planning(
        self, ego_x, ego_y, ego_heading_rad, obs_x_vec, obs_y_vec, slot_points, ds
    ):
        self.result.Reset()
        self.result.initial_pose = [ego_x, ego_y, ego_heading_rad]

        x_vec = slot_points[0]
        y_vec = slot_points[1]

        slot_width = math.sqrt((x_vec[0] - x_vec[3]) ** 2 + (y_vec[0] - y_vec[3]) ** 2)
        slot_length = math.sqrt((x_vec[0] - x_vec[1]) ** 2 + (y_vec[0] - y_vec[1]) ** 2)

        print("slot_width = ", slot_width)
        print("slot_length = ", slot_length)

        start_time = time.perf_counter()
        self.planning_module.Init()
        if self.planning_module.Preprocess(
            obs_x_vec, obs_y_vec, slot_width, slot_length, ego_x, ego_y, ego_heading_rad, ds):
            print("ego pt in hybrid = ", ego_x, ego_y)
            if self.planning_module.Update():
                self.result.success = True

                end_time = time.perf_counter()
                # computation time
                self.result.computation_time = (end_time - start_time) * 1000
                # path points
                self.result.path_x_vec = self.planning_module.GetPathEle(0)
                self.result.path_y_vec = self.planning_module.GetPathEle(1)
                self.result.path_heading_vec = self.planning_module.GetPathEle(2)
                print("first pt of path = ", self.result.path_x_vec[0], self.result.path_y_vec[0])

                path_points_size = len(self.result.path_x_vec)

                for i in range(path_points_size - 1):
                    pos1 = np.array(
                        [self.result.path_x_vec[i], self.result.path_y_vec[i]]
                    )
                    pos2 = np.array(
                        [self.result.path_x_vec[i + 1], self.result.path_y_vec[i + 1]]
                    )
                    v_12 = pos2 - pos1
                    v_12_norm = np.linalg.norm(v_12)
                    if v_12_norm < 0.01:
                        continue
                    # path length
                    self.result.path_length += v_12_norm
                    if i != 0:
                        pos0 = np.array(
                            [
                                self.result.path_x_vec[i - 1],
                                self.result.path_y_vec[i - 1],
                            ]
                        )
                        v_01 = pos1 - pos0

                        u_12 = v_12 /  np.linalg.norm(v_12)
                        u_01 = v_01 /  np.linalg.norm(v_01)


                        dot = np.dot(u_12, u_01)
                        if dot < 1e-12:
                            self.result.total_gear_shift_cnt += 1
                            self.result.gear_shift_idx_vec.append(i)


                escape_idx = path_points_size - 1
                for i in range(len(self.result.gear_shift_idx_vec)):
                    idx = self.result.gear_shift_idx_vec[i]
                    if self.result.path_y_vec[idx] < 0.5 * slot_width and self.result.path_x_vec[idx] > -0.8 and self.result.path_x_vec[idx] < slot_length + 0.8:
                        escape_idx = idx
                        self.result.escape_heading = self.result.path_heading_vec[idx]
                        self.result.gear_shift_cnt_slot = self.result.total_gear_shift_cnt - i
                        break
                p0 = (self.result.path_x_vec[escape_idx], self.result.path_y_vec[escape_idx])
                p1 = (self.result.path_x_vec[escape_idx - 2], self.result.path_y_vec[escape_idx - 2])
                p2 = (self.result.path_x_vec[escape_idx - 4], self.result.path_y_vec[escape_idx - 4])
                center, radius = circle_from_points(p0, p1, p2)
                print("center = ", center)
                print("radius = ", radius)

                car_vertex_local_x_vec, car_vertex_local_y_vec, _ = load_car_params_patch_parking()
                corner_radius = sqrt(car_vertex_local_x_vec[0] ** 2 + (car_vertex_local_y_vec[0] + radius)** 2)
                print("corner radius = ", corner_radius)

                corner_oa_dist = 10.0
                for obs_x, obs_y in zip(obs_x_vec, obs_y_vec):
                    if is_in_bound(obs_x, slot_length - 0.3, slot_length + 2.0) and is_in_bound(obs_y, 0.5 * slot_width - 0.45 , 0.5 * slot_width + 1.0):
                        dist_obs_to_center = sqrt((obs_x - center[0])**2 + (obs_y - center[1])**2)
                        corner_oa_dist = min(corner_oa_dist, dist_obs_to_center - corner_radius)
                self.result.corner_obstacle_avoidance_dist = corner_oa_dist

                self.result.PrintInfo()


        return self.result.success


if __name__ == "__main__":
    ego_x = 1.0
    ego_y = 2.0
    ego_heading = 0.0
    obs_x_vec = []
    obs_y_vec = []
    slot_points = []

    planner = PlanningUpdater("lib.parallel_planning_py")

    success = planner.update_planning(
        ego_x, ego_y, ego_heading, obs_x_vec, obs_y_vec, slot_points, 0.02
    )
    print("success = ", success)
    print("path_x_vec = ", planner.path_x_vec)
    print("path_y_vec = ", planner.path_y_vec)
    print("computation_time_ms = ", planner.computation_time)
