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

sys.path.append('../lib')


import importlib

# bag path
with open(sys.argv[1]) as file:
    data = json.load(file)

output_path = data['output_path']

try:
  os.mkdir(output_path)
except:
  pass

def UpdatePlanningPybind(ego_x, ego_y, ego_heading, obs_x_vec, obx_y_vec, slot_points, planning_module):
  module = importlib.import_module('lib.' + planning_module)
  PlanningClass = getattr(module, 'Planning')  # 确认类名为 'Planning'
  planning_instance = PlanningClass()         # 实例化类
  planning_instance.Init()
  planning_instance.UpdateByJson(ego_x, ego_y, ego_heading, obs_x_vec, obx_y_vec, slot_points)