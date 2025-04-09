import sys, os, json, time
from math import cos, fabs, pi, sin

# import random
# import multiprocessing
from datetime import datetime
from bokeh.plotting import figure, show

sys.path.append("..")
sys.path.append("../..")

sys.path.append("../../simulation/")



from checker.lib.checker_base import read_variables_from_json
from simulation.code.APA_Planning.jupyter_pybind.notebooks_apa.lib.load_json import write_json
from simulation.code.APA_Planning.jupyter_pybind.notebooks_apa.lib.load_rotate import ego_vertex_l2g, ego_vertex_path_l2g
from simulation.lib import parallel_planning_py

import ipywidgets
from IPython.core.display import display, HTML

import bokeh.plotting as bkp
from bokeh.events import Tap
from bokeh.layouts import row
from bokeh.models import Range1d, ColumnDataSource, SingleIntervalTicker
from bokeh.models import WheelZoomTool, HoverTool, TapTool, CustomJS
from bokeh.io import output_notebook, push_notebook, export_svgs

from lib.load_json import load_json
from lib.load_rotate import *
from lib.load_struct import *

from simulation.task.apa_simulator import PlanningUpdater

from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import cairosvg

kRad2Deg = 180.0 / pi
kDeg2Rad = pi / 180.0


def update_planning_for_pose(initial_pose, obs_x_vec, obs_y_vec, slot_points):
    planner = PlanningUpdater("parallel_planning_py")
    success = planner.update_planning(
        initial_pose[0], initial_pose[1], initial_pose[2], obs_x_vec, obs_y_vec, slot_points, 0.02
    )
    return planner.result.TransferToJson()


# def process_scenario(scenario_data):
#     """Process a single scenario and return the results."""
#     initial_pose_vec = scenario_data["initial_pose"]
#     obs_x_vec = scenario_data["scenario_data"]["obs_x"]
#     obs_y_vec = scenario_data["scenario_data"]["obs_y"]

#     target_slot_x_vec = scenario_data["scenario_data"]["target_corner_x"]
#     target_slot_y_vec = scenario_data["scenario_data"]["target_corner_y"]
#     slot_points = [target_slot_x_vec, target_slot_y_vec]

#     # Process all initial poses in parallel
#     result_vec = [
#         update_planning_for_pose(initial_pose, obs_x_vec, obs_y_vec, slot_points)
#         for initial_pose in initial_pose_vec
#     ]

#     # Filter out None results (in case of failure)
#     return [result for result in result_vec if result is not None]

if __name__ == "__main__":
    # Read the configuration from JSON and process the data
    _, _, _, _, max_process, output_path = read_variables_from_json("checker_task.json")

    # Ensure the output path exists
    os.makedirs(output_path, exist_ok=True)

     # Save the results to a file
    output_file_path = os.path.join(output_path, "hybrid_a_star.json")
    # output_file_path = os.path.join(output_path, "proposed_geometric_method.json")
    # output_file_path = os.path.join(output_path, "vor_ppm.json")
    # output_file_path = os.path.join(output_path, "vor.json")

    # planner_res = {}
    # start_time = time.time()

    # # Use multiprocessing to process all scenarios
    # with multiprocessing.Pool(processes=max_process) as pool:
    #     # Passing each scenario to process_scenario function
    #     results = pool.map(process_scenario, [input_data[scenario_key] for scenario_key in input_data])

    # # Store the results in planner_res
    # for i, result in enumerate(results):
    #     scenario_key = str(i)
    #     planner_res[scenario_key] = result

    # # End time for performance calculation
    # end_time = time.time()
    # time_taken = end_time - start_time
    # print(f"Time taken for computation: {time_taken:.2f} seconds")



    # save_json(planner_res, output_file_path)

    # for i, scenario_res_vec in planner_res.items():
    #     success_cnt = 0
    #     total_cnt = len(scenario_res_vec)

    #     for result in scenario_res_vec:
    #         if result["success"]:
    #             success_cnt += 1
    #             # print("initial_pose = ", result["initial_pose"])
    #             # for i in range(len(result["path_x_vec"])):
    #             #     print(i, ", ", result["path_x_vec"][i], ", ", result["path_y_vec"][i], ", ", result["path_heading_vec"][i] * K_RAD2DEG )

    #         else:
    #             print("failed initial pose = ", result["initial_pose"])

    #     success_rate = success_cnt / total_cnt if total_cnt > 0 else 0

    #     print(f"Scenario {i} success rate = {success_rate:.2%}")

    input_data = load_json("../out/initial_pose_obs_slot.json")

    output_dict = {}
    for key in input_data:
        certain_scenario_data = input_data[key]
        scenario_data = certain_scenario_data["scenario_data"]

        obs_x_vec = scenario_data["obs_x"]
        obs_y_vec = scenario_data["obs_y"]

        target_corner_x_vec = scenario_data["target_corner_x"]
        target_corner_y_vec = scenario_data["target_corner_y"]

        front_corner_x_vec = scenario_data["front_corner_x"]
        rear_corner_x_vec = scenario_data["rear_corner_x"]


        ego_pose = certain_scenario_data["initial_pose"][0]
        print("ego_pose = ", ego_pose)

        slot_points = [ target_corner_x_vec, target_corner_y_vec]

        print("---------------------")
        print("obs_x_vec size = ", len(obs_x_vec))
        print("obs_y_vec size = ", len(obs_y_vec))

        pout_x = -100
        pout_y = -100
        pin_x = 100
        pin_y = -100
        for i in range(len(obs_x_vec)):
            obs_x = obs_x_vec[i]
            obs_y = obs_y_vec[i]

            if obs_y > -0.5 and obs_y < 1.5:
                if obs_x < 3:
                    pout_x = max(pout_x, obs_x)
                    pout_y = max(pout_y, obs_y)
                else:
                    pin_x = min(pin_x, obs_x)
                    pin_y = max(pin_y, obs_y)
        print("pin = ", pin_x, ", ", pin_y)
        print("pout = ", pout_x, ", ", pout_y)

        print("tlane length = ", pin_x - pout_x)

        print("slot_points = ", slot_points)


        res = update_planning_for_pose(ego_pose, obs_x_vec, obs_y_vec, slot_points)
        print("res = ", res)
    #     one_scenario_data = [res]
    #     output_dict[key] = one_scenario_data
    # write_json(output_file_path, output_dict)



