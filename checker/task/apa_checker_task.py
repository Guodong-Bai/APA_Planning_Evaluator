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
from simulation.task.contruct_scenario import construct_all_scenarios
from simulation.task.apa_simulator import PlanningUpdater

kRad2Deg = 180.0 / math.pi
kDeg2Rad = math.pi / 180.0


# Function to generate a set of initial poses for a specific scenario
def generate_initial_poses_for_scenario(scenario_data, poses_to_generate):
    obs_x_vec = scenario_data["obs_x"]
    obs_y_vec = scenario_data["obs_y"]

    ava_pose_vec = []
    initial_pose_cnt = 0
    col_det = CollisionDetection(obs_x_vec, obs_y_vec)
    # Generate poses until the specified number of poses is reached
    while initial_pose_cnt < poses_to_generate:
        ego_x = random.uniform(x_bounds[0], x_bounds[1])
        ego_y = random.uniform(y_bounds[0], y_bounds[1])
        ego_heading = random.uniform(heading_deg_bounds[0],
                                     heading_deg_bounds[1]) * kDeg2Rad
        if col_det.check_pose_collided(ego_x, ego_y, ego_heading):
            continue
        initial_pose_cnt += 1
        ava_pose_vec.append([ego_x, ego_y, ego_heading])
        print(
            f"Generated {initial_pose_cnt} poses for scenario - ego pose(deg) = {ego_x}, {ego_y}, {ego_heading * kRad2Deg}"
        )

    return ava_pose_vec


# Function to manage multiprocessing across each scenario's pose generation
def process_initial_poses_for_scenario(scenario_data, max_initial_pose_cnt,
                                       max_process):
    # Calculate how many poses each process should generate
    poses_per_process = max_initial_pose_cnt // max_process
    remaining_poses = max_initial_pose_cnt % max_process

    # Tasks list for each process
    tasks = []
    for i in range(max_process):
        poses_to_generate = poses_per_process + (1
                                                 if i < remaining_poses else 0)
        tasks.append((scenario_data, poses_to_generate))

    # Use multiprocessing to parallelize the pose generation
    with multiprocessing.Pool(processes=max_process) as pool:
        ava_pose_vec = pool.starmap(generate_initial_poses_for_scenario, tasks)

    # Flatten the list of results and return
    flattened_result = [pose for sublist in ava_pose_vec for pose in sublist]
    return flattened_result


def sample_initial_pose(output_file_path):
    if not os.path.exists(output_file_path):
        print(f"File do not exists: {output_file_path}")
        # scenario_list: load obstacles in different scenarios
        scenario_files = construct_all_scenarios()

        # Start time for performance calculation
        start_time = time.time()

        param_dict = {}
        # Call the function to process all scenarios and generate initial poses in parallel
        for i in range(len(scenario_files)):
            file = scenario_files[i]
            scenario_data = load_json(file)
            result = process_initial_poses_for_scenario(scenario_data,
                                                        max_initial_pose_cnt,
                                                        max_process)
            param_dict[i] = {"initial_pose": result,
                            "scenario_data": scenario_data}

        # End time for performance calculation
        end_time = time.time()

        # Calculate and print the time taken for computation
        time_taken = end_time - start_time
        print(f"Time taken for computation: {time_taken:.2f} seconds")

        save_json(param_dict, output_file_path)
        # print("param_dict = ", param_dict)
    else:
        print(f"File already exists: {output_file_path}")


# if __name__ == "__main__":

#     x_bounds, y_bounds, heading_deg_bounds, max_initial_pose_cnt, max_process, output_path = \
#         read_variables_from_json("checker_task.json")

#     try:
#         os.mkdir(output_path)
#     except:
#         pass

#     output_file_path = output_path + "/" + "initial_pose_obs_slot.json"

#     sample_initial_pose(output_file_path)

#     param_dict = load_json(output_file_path)
#     planner_res = {}

#     for i in range(len(param_dict)):

#         scenario_key = str(i)
#         scenario = param_dict[scenario_key]
#         scenario_data = scenario["scenario_data"]
#         obs_x_vec = scenario_data["obs_x"]
#         obs_y_vec = scenario_data["obs_y"]
#         initial_pose_vec = scenario["initial_pose"]

#         target_slot_x_vec = scenario_data["target_corner_x"]
#         target_slot_y_vec = scenario_data["target_corner_y"]
#         slot_points = [target_slot_x_vec, target_slot_y_vec]

#         result_vec = []
#         for j in range(len(initial_pose_vec)):
#             initial_pose = initial_pose_vec[j]
#             planner = PlanningUpdater("parallel_planning_py")
#             success = planner.update_planning(initial_pose[0], initial_pose[1], initial_pose[2], obs_x_vec, obs_y_vec, slot_points, 0.025)
#             result_vec.append(planner.result.TransferToJson())
#         planner_res[scenario_key] = result_vec
#     output_file_path = output_path + "/" + "proposed_geometric_method.json"
#     save_json(planner_res, output_file_path)


    # ego_local_x_vec, ego_local_y_vec, _ = load_car_params_patch_parking()
    # car_box_x_vec, car_box_y_vec = load_car_box(path_x_vec, path_y_vec, path_heading_vec, ego_local_x_vec, ego_local_y_vec)
    # fig1 = figure(width=1200, height=800, match_aspect=True, aspect_scale=1)
    # fig1.line(path_x_vec, path_y_vec, line_width = 3.0, line_color = 'green', line_dash = 'solid',legend_label = 'Car Path', visible = True)
    # fig1.patches(car_box_x_vec, car_box_y_vec, fill_color = None, fill_alpha = 0.0, line_color = "black", line_width = 0.3, visible = True)
    # fig1.patch(target_slot_x_vec, target_slot_y_vec, fill_color='blue', line_color=None, fill_alpha=0.4, line_width = 0.0, legend_label = 'Target slot')
    # fig1.patch(front_slot_x_vec, target_slot_y_vec, fill_color='grey', line_color='grey', fill_alpha=0.2, line_width = 0.3)
    # fig1.patch(rear_slot_x_vec, target_slot_y_vec, fill_color='grey', line_color='grey', fill_alpha=0.2, line_width = 0.3, legend_label = 'Nearby slots')
    # fig1.scatter(obs_x_vec, obs_y_vec, size=3, color='grey',legend_label = 'External obstacles')
    # show(fig1)




    # Function to update planning for a single initial pose
def update_planning_for_pose(initial_pose, obs_x_vec, obs_y_vec, slot_points):
    planner = PlanningUpdater("parallel_planning_py")
    success = planner.update_planning(initial_pose[0], initial_pose[1], initial_pose[2], obs_x_vec, obs_y_vec, slot_points, 0.025)
    if success:
        return planner.result.TransferToJson()
    else:
        return None  # or some kind of failure indicator

def process_scenario(scenario_key, scenario_data):
    # Extract scenario data
    initial_pose_vec = scenario_data["initial_pose"]
    obs_x_vec = scenario_data["scenario_data"]["obs_x"]
    obs_y_vec = scenario_data["scenario_data"]["obs_y"]

    target_slot_x_vec = scenario_data["scenario_data"]["target_corner_x"]
    target_slot_y_vec = scenario_data["scenario_data"]["target_corner_y"]
    slot_points = [target_slot_x_vec, target_slot_y_vec]

    # Use multiprocessing to parallelize planning for each initial pose
    with multiprocessing.Pool() as pool:
        result_vec = pool.starmap(update_planning_for_pose, [(initial_pose, obs_x_vec, obs_y_vec, slot_points) for initial_pose in initial_pose_vec])

    # Filter out None results (in case of failure)
    result_vec = [result for result in result_vec if result is not None]

    return result_vec

# if __name__ == "__main__":

#     x_bounds, y_bounds, heading_deg_bounds, max_initial_pose_cnt, max_process, output_path = \
#         read_variables_from_json("checker_task.json")
#     try:
#         os.mkdir(output_path)
#     except:
#         pass

#     output_file_path = output_path + "/" + "initial_pose_obs_slot.json"
#     sample_initial_pose(output_file_path)

#     param_dict = load_json(output_file_path)
#     planner_res = {}

#     # Start time for performance calculation
#     start_time = time.time()

#     # Use multiprocessing to process all scenarios
#     with multiprocessing.Pool() as pool:
#         # Corrected line: now passing (scenario_key, param_dict[scenario_key])
#         results = pool.starmap(process_scenario, [(scenario_key, param_dict[scenario_key]) for scenario_key in param_dict])

#     # Store the results
#     for i, result in enumerate(results):
#         scenario_key = str(i)
#         planner_res[scenario_key] = result

#     # End time for performance calculation
#     end_time = time.time()
#     time_taken = end_time - start_time
#     print(f"Time taken for computation: {time_taken:.2f} seconds")

#     # Save the result to a file
#     output_file_path = os.path.join(output_path, "proposed_geometric_method.json")
#     save_json(planner_res, output_file_path)
