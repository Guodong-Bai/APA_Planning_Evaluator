
import os
import sys
import time
import random
import multiprocessing
from datetime import datetime

from bokeh.plotting import figure, show

sys.path.append("..")
sys.path.append("../..")
sys.path.append("../lib")
sys.path.append("../../simulation/task")

from lib.load_json import *
from lib.load_rotate import *
from lib.load_struct import *
from lib.checker_base import *
from lib.collision_detection import CollisionDetection
from simulation.task.contruct_scenario import construct_all_scenarios, construct_all_scenarios_multi_process
from simulation.task.apa_simulator import PlanningUpdater

kRad2Deg = 180.0 / math.pi
kDeg2Rad = math.pi / 180.0


def generate_initial_poses_for_scenario(scenario_data, poses_to_generate):
    obs_x_vec = scenario_data["obs_x"]
    obs_y_vec = scenario_data["obs_y"]

    available_poses = []
    initial_pose_count = 0
    collision_detector = CollisionDetection(obs_x_vec, obs_y_vec)

    while initial_pose_count < poses_to_generate:
        ego_x = random.uniform(x_bounds[0], x_bounds[1])
        ego_y = random.uniform(y_bounds[0], y_bounds[1])
        ego_heading = random.uniform(heading_deg_bounds[0]* kDeg2Rad,
                                     heading_deg_bounds[1]* kDeg2Rad)

        if collision_detector.check_pose_collided(ego_x, ego_y, ego_heading):
            continue

        initial_pose_count += 1
        available_poses.append([ego_x, ego_y, ego_heading])

        print(f"Generated {initial_pose_count} poses for scenario "
              f"- ego pose(deg) = {ego_x}, {ego_y}, {ego_heading * kRad2Deg}")

    return available_poses


def process_initial_poses_for_scenario(scenario_data, max_initial_pose_count, max_process):
    poses_per_process = max_initial_pose_count // max_process
    remaining_poses = max_initial_pose_count % max_process

    tasks = []
    for i in range(max_process):
        poses_to_generate = poses_per_process + (1 if i < remaining_poses else 0)
        tasks.append((scenario_data, poses_to_generate))

    with multiprocessing.Pool(processes=max_process) as pool:
        available_poses = pool.starmap(generate_initial_poses_for_scenario, tasks)

    # Flatten the list of results and return
    return [pose for sublist in available_poses for pose in sublist]


def sample_initial_pose(output_file_path):
    if not os.path.exists(output_file_path):
        print(f"File does not exist: {output_file_path}")

        # Load obstacles in different scenarios
        # scenario_files = construct_all_scenarios()
        scenario_files = construct_all_scenarios_multi_process()

        param_dict = {}
        start_time = time.time()
        # Process all scenarios and generate initial poses in parallel
        for i, file in enumerate(scenario_files):
            scenario_data = load_json(file)
            result = process_initial_poses_for_scenario(scenario_data,
                                                        max_initial_pose_count,
                                                        max_process)
            param_dict[i] = {"initial_pose": result, "scenario_data": scenario_data}

        end_time = time.time()

        time_taken = end_time - start_time
        print(f"Time taken for computation: {time_taken:.2f} seconds")

        save_json(param_dict, output_file_path)
    else:
        print(f"File already exists: {output_file_path}, no need to generate new file")


if __name__ == "__main__":
    x_bounds, y_bounds, heading_deg_bounds, max_initial_pose_count, max_process, output_path = \
        read_variables_from_json("checker_task.json")

    try:
        os.mkdir(output_path)
    except FileExistsError:
        pass

    output_file_path = os.path.join(output_path, "initial_pose_obs_slot.json")

    sample_initial_pose(output_file_path)

    # For debugging
    data = load_json(output_file_path)
    initial_pose_size = len(data["0"]["initial_pose"])
    print("Initial pose size = ", initial_pose_size)
#     # ego_local_x_vec, ego_local_y_vec, _ = load_car_params_patch_parking()
#     # car_box_x_vec, car_box_y_vec = load_car_box(path_x_vec, path_y_vec, path_heading_vec, ego_local_x_vec, ego_local_y_vec)
#     # fig1 = figure(width=1200, height=800, match_aspect=True, aspect_scale=1)
#     # fig1.line(path_x_vec, path_y_vec, line_width = 3.0, line_color = 'green', line_dash = 'solid',legend_label = 'Car Path', visible = True)
#     # fig1.patches(car_box_x_vec, car_box_y_vec, fill_color = None, fill_alpha = 0.0, line_color = "black", line_width = 0.3, visible = True)
#     # fig1.patch(target_slot_x_vec, target_slot_y_vec, fill_color='blue', line_color=None, fill_alpha=0.4, line_width = 0.0, legend_label = 'Target slot')
#     # fig1.patch(front_slot_x_vec, target_slot_y_vec, fill_color='grey', line_color='grey', fill_alpha=0.2, line_width = 0.3)
#     # fig1.patch(rear_slot_x_vec, target_slot_y_vec, fill_color='grey', line_color='grey', fill_alpha=0.2, line_width = 0.3, legend_label = 'Nearby slots')
#     # fig1.scatter(obs_x_vec, obs_y_vec, size=3, color='grey',legend_label = 'External obstacles')
#     # show(fig1)