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

K_RAD2DEG = 180.0 / math.pi
K_DEG2RAD = math.pi / 180.0


def update_planning_for_pose(initial_pose, obs_x_vec, obs_y_vec, slot_points):
    planner = PlanningUpdater("parallel_planning_py")
    success = planner.update_planning(
        initial_pose[0], initial_pose[1], initial_pose[2], obs_x_vec, obs_y_vec, slot_points, 0.025
    )
    return planner.result.TransferToJson()


def process_scenario(scenario_data):
    """Process a single scenario and return the results."""
    initial_pose_vec = scenario_data["initial_pose"]
    obs_x_vec = scenario_data["scenario_data"]["obs_x"]
    obs_y_vec = scenario_data["scenario_data"]["obs_y"]

    target_slot_x_vec = scenario_data["scenario_data"]["target_corner_x"]
    target_slot_y_vec = scenario_data["scenario_data"]["target_corner_y"]
    slot_points = [target_slot_x_vec, target_slot_y_vec]

    # Process all initial poses in parallel
    result_vec = [
        update_planning_for_pose(initial_pose, obs_x_vec, obs_y_vec, slot_points)
        for initial_pose in initial_pose_vec
    ]

    # Filter out None results (in case of failure)
    return [result for result in result_vec if result is not None]

if __name__ == "__main__":
    # Read the configuration from JSON and process the data
    x_bounds, y_bounds, heading_deg_bounds, max_initial_pose_cnt, max_process, output_path = read_variables_from_json("checker_task.json")

    # Ensure the output path exists
    os.makedirs(output_path, exist_ok=True)

    output_file_path = os.path.join(output_path, "initial_pose_obs_slot.json")

    planner_res = {}
    start_time = time.time()
    param_dict = load_json(output_file_path)

    # Use multiprocessing to process all scenarios
    with multiprocessing.Pool(processes=max_process) as pool:
        # Passing each scenario to process_scenario function
        results = pool.map(process_scenario, [param_dict[scenario_key] for scenario_key in param_dict])

    # Store the results in planner_res
    for i, result in enumerate(results):
        scenario_key = str(i)
        planner_res[scenario_key] = result

    # End time for performance calculation
    end_time = time.time()
    time_taken = end_time - start_time
    print(f"Time taken for computation: {time_taken:.2f} seconds")

    # Save the results to a file
    output_file_path = os.path.join(output_path, "traditional_geometric_method.json")

    # output_file_path = os.path.join(output_path, "proposed_geometric_method.json")


    save_json(planner_res, output_file_path)

    for i, scenario_res_vec in planner_res.items():
        success_cnt = 0
        total_cnt = len(scenario_res_vec)

        for result in scenario_res_vec:
            if result["success"]:
                success_cnt += 1
            else:
                print("failed initial pose = ", result["initial_pose"])

        success_rate = success_cnt / total_cnt if total_cnt > 0 else 0

        print(f"Scenario {i} success rate = {success_rate:.2%}")
