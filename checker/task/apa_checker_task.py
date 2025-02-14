import os
import time
from datetime import datetime
import sys
sys.path.append("..")
sys.path.append("../lib")

from lib.checker_base import *
from lib.collision_detection import CollisionDetection



# planning_module_list: load different apa path planning modules: module_list, record path planning result.
planning_module_list, x_bounds, y_bounds, heading_deg_bounds, max_initial_pose_cnt, \
  max_process, output_path = read_variables_from_json(sys.argv[1])

with open('checker_task.json') as f:
    data = json.load(f)
max_process = data['max_process']
output_path = data['output_path']
max_initial_pose_cnt = data['max_initial_pose_cnt']
# planning_module_list = data['planning_module_list']
x_bounds = [data['x_lower_bound'], data['x_upper_bound']]
y_bounds = [data['y_lower_bound'], data['y_upper_bound']]
heading_deg_bounds = [data['heading_deg_lower_bound'], data['heading_deg_upper_bound']]

print(" max_process = ", max_process)
print(" output_path = ", output_path)
print(" x_bounds = ", x_bounds)
print(" y_bounds = ", y_bounds)
print(" heading_deg_bounds = ", heading_deg_bounds)
print(" max_initial_pose_cnt = ", max_initial_pose_cnt)
# print("planning_module_list = ", planning_module_list)

# scenario_list: load obstacles in different scenarios
obstacle_x_vec = []
obstacle_y_vec = []
# slot_points =load_slot()
# obstacle_x_vec, obstacle_y_vec = load_obstacles()

t_tag1 = time.time()
now = datetime.now()
formatted_str = now.strftime('%Y.%m%d.%H%M.%S')

try:
  os.mkdir(output_path)
except:
  pass

output_path = output_path + '/' + "checker." + formatted_str













# # start_pose_list: construct different start poses for parallel parking that are at least 0.3 m far away from obstacles
# col_det = CollisionDetection(obstacle_x_vec, obstacle_y_vec)
# initial_pose_cnt = 0

# while initial_pose_cnt <






