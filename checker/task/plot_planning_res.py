import math
import sys
import ipywidgets

import bokeh.plotting as bkp
from bokeh.events import Tap
from bokeh.layouts import row
from bokeh.io import output_notebook, push_notebook
from bokeh.models import ColumnDataSource, WheelZoomTool, HoverTool, TapTool, CustomJS, Arrow, VeeHead

sys.path.append("..")
sys.path.append("../..")
sys.path.append("../lib")
sys.path.append("../out")

from IPython.core.display import display, HTML
from lib.load_json import load_json
from lib.load_rotate import load_car_box, local2global
from lib.load_struct import *
from lib.region_sample import grid_sample

arrow_len = 0.3
display_resolution = 1.2

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()


data_obs = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_slot = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_other_slot = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_path = ColumnDataSource(data={"x_vec": [], "y_vec": [], "theta_vec": []})
data_path_box = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_path_circle = ColumnDataSource(data={"x_vec": [], "y_vec": [], "radius_vec": []})

data_initial_pose = ColumnDataSource(data={"x": [], "y": [], "theta": []})
data_initial_car_vertex = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_target_pose = ColumnDataSource(data={"x": [], "y": [], "theta": []})
data_target_car_vertex = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_initial_pos =  ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_initial_pose_arrow =  ColumnDataSource(data={"xs_vec": [], "ys_vec": [], "xe_vec":[], "ye_vec":[]})

data_initial_pose_region = ColumnDataSource(data={"cx": [], "cy": [], "w":[], "h":[]})

ego_local_x_vec, ego_local_y_vec, _ = load_car_params_patch_parking()

ego_circle_local_x_vec, ego_circle_local_y_vec, ego_circle_local_r_vec = [
    lst[1:] for lst in load_car_circle_coord()
]


def reset_data():
    data_path.data.update({"x_vec": [], "y_vec": [], "theta_vec": []})
    data_path_box.data.update({"x_vec": [], "y_vec": []})
    data_path_circle.data.update({"x_vec": [], "y_vec": [], "radius_vec": []})


scenario = load_json("../out/initial_pose_obs_slot.json")
checker_task = load_json("checker_task.json")


fig1 = bkp.figure(width=1200, height=800, match_aspect=True, aspect_scale=1)
fig1.x_range.flipped = False
fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0

# measure tool
source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.circle("x", "y", size=10, source=source, color="red", legend_label="measure tool")
line_source = ColumnDataSource(data=dict(x=[], y=[]))
fig1.line(
    "x",
    "y",
    source=source,
    line_width=3,
    line_color="pink",
    line_dash="solid",
    legend_label="measure tool",
)
text_source = ColumnDataSource(data=dict(x=[], y=[], text=[]))
fig1.text(
    "x",
    "y",
    "text",
    source=text_source,
    text_color="red",
    text_align="center",
    text_font_size="15pt",
    legend_label="measure tool",
)
# Define the JavaScript callback code
callback_code = """
    var x = cb_obj.x;
    var y = cb_obj.y;

    source.data['x'].push(x);
    source.data['y'].push(y);

    if (source.data['x'].length > 2) {
        source.data['x'].shift();
        source.data['y'].shift();
        source.data['x'].shift();
        source.data['y'].shift();
    }
    source.change.emit();

    if (source.data['x'].length >= 2) {
        var x1 = source.data['x'][source.data['x'].length - 2];
        var y1 = source.data['y'][source.data['y'].length - 2];
        var x2 = x;
        var y2 = y;
        var x3 = (x1 + x2) / 2;
        var y3 = (y1 + y2) / 2;

        var distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

        console.log("Distance between the last two points: " + distance);

        distance = distance.toFixed(4);
        text_source.data = {'x': [x3], 'y': [y3], 'text': [distance]};
        text_source.change.emit();

        line_source.data = {'x': [x1, x2], 'y': [y1, y2]};
        line_source.change.emit();
    }

    if (source.data['x'].length == 1) {
        text_source.data['x'].shift();
        text_source.data['y'].shift();
        text_source.data['text'].shift();
    }
    text_source.change.emit();
"""
# Create a CustomJS callback with the defined code
callback = CustomJS(
    args=dict(source=source, line_source=line_source, text_source=text_source),
    code=callback_code,
)
# Attach the callback to the Tap event on the plot
fig1.js_on_event(Tap, callback)

fig1.line(
    "x_vec",
    "y_vec",
    source=data_path,
    line_width=3.0,
    line_color="green",
    line_dash="solid",
    legend_label="Car Path",
    visible=True,
)

fig1.patch(
    "x_vec",
    "y_vec",
    source=data_slot,
    line_width=1.0,
    line_color="black",
    fill_alpha=0.0,
    # legend_label="Target slots",
    visible=True,
)

fig1.patches(
    "x_vec",
    "y_vec",
    source=data_other_slot,
    line_width=1.5,
    line_color="black",
    fill_alpha=0.0,
    # legend_label="Nearby slots",
    visible=True,
)
fig1.scatter(
    "x_vec",
    "y_vec",
    source=data_obs,
    size=3,
    color="grey",
    legend_label="External obstacles",
)

fig1.scatter(
    "x", "y", source=data_initial_pose, size=3, color="red", legend_label="Initial pose"
)
fig1.patch(
    "x_vec",
    "y_vec",
    source=data_initial_car_vertex,
    fill_color="blue",
    line_color="blue",
    fill_alpha=0.3,
    line_width=0.3,
    legend_label="Ego car",
)
fig1.patches(
    "x_vec",
    "y_vec",
    source=data_path_box,
    fill_color="black",
    line_color="black",
    fill_alpha=0.01,
    line_width=0.3,
    legend_label="Envelope",
)

fig1.circle(
    x="x_vec",
    y="y_vec",
    radius="radius_vec",
    source=data_path_circle,
    fill_color=None,
    line_color="black",
    line_alpha=1.0,  # 设置边线完全不透明
    line_width=0.5,
    legend_label="Circle Envelope",
    visible=False,
)

# fig1.segment(
#     x0="xs_vec", y0="ys_vec", x1="xe_vec", y1="ye_vec",
#     source=data_initial_pose_arrow,
#     line_width=1, line_color="black", legend_label="initial pose"
# )

fig1.circle("x_vec", "y_vec", source=data_initial_pos, size=2, color="green", alpha=1, legend_label="initial pose")



arrow = Arrow(
    end=VeeHead(
        size=8,
        fill_color="blue",
        fill_alpha=0.6,
        line_color="blue",   # ← 这里把头部轮廓也设为绿色
        line_width=1          # ← 如果想让轮廓线更粗／更细，可以调这个参数
    ),
    x_start="xs_vec",
    y_start="ys_vec",
    x_end="xe_vec",
    y_end="ye_vec",
    source=data_initial_pose_arrow,
    line_color="blue",      # ← 这是箭杆（shaft）的颜色
    line_width=1,
    line_alpha=1
)

fig1.add_layout(arrow)

fig1.rect(
    x="cx", y="cy",
    width="w", height="h",
    angle=0,                # 不旋转
    source=data_initial_pose_region,
    fill_color=None,        # 只绘制边框
    line_color="green",
    line_width=2,
    legend_label="Rectangles"
)




fig1.legend.label_text_font = "Times New Roman"  # 设置图例字体类型
fig1.legend.label_text_font_size = "14pt"  # 设置图例字体大小
fig1.legend.location = "top_left"

fig1.legend.click_policy = "hide"
fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)


class LocalViewSlider:
    def __init__(self, slider_callback):
        self.method_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width="20%"),
            description="planning method",
            min=0,
            max=1,
            value=1,
            step=1,
        )

        self.scenario_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width="25%"),
            description="scenario key",
            min=0,
            max=50,
            value=0,
            step=1,
        )

        self.case_idx_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width="60%"),
            description="intitial pose",
            min=0,
            max=999,
            value=0,
            step=1,
        )

        ipywidgets.interact(
            slider_callback,
            method=self.method_slider,
            scenario_key=self.scenario_slider,
            case_idx=self.case_idx_slider,
        )


def slider_callback(method, scenario_key, case_idx):
    reset_data()

    x_min = checker_task["x_lower_bound"]
    x_max = checker_task["x_upper_bound"]
    y_min = checker_task["y_lower_bound"]
    y_max = checker_task["y_upper_bound"]

    data_initial_pose_region.data.update({\
        "cx": [0.5 * (x_min + x_max)],
        "cy": [0.5 * (y_min + y_max)],
        "w": [x_max - x_min],
        "h": [y_max - y_min]
    })

    key = str(scenario_key)
    initial_pose_vec = scenario[key]["initial_pose"]

    tmp_x_vec = []
    tmp_y_vec = []
    for x, y, theta in initial_pose_vec:
        tmp_x_vec.append(x)
        tmp_y_vec.append(y)
    data_initial_pos.data.update({
        "x_vec": tmp_x_vec,
        "y_vec": tmp_y_vec
    })

    xs_vec = []
    ys_vec = []
    xe_vec = []
    ye_vec = []
    simp_pose_vec = grid_sample(initial_pose_vec, display_resolution)
    for x, y, theta in simp_pose_vec:
        dx = arrow_len * math.cos(theta)
        dy = arrow_len * math.sin(theta)
        xs_vec.append(x)
        ys_vec.append(y)
        xe_vec.append(x + dx)
        ye_vec.append(y + dy)


    data_initial_pose_arrow.data.update({
        "xs_vec": xs_vec,
        "ys_vec": ys_vec,
        "xe_vec": xe_vec,
        "ye_vec": ye_vec
    })

    if method == 0:
        planning_res = load_json("../out/hybrid_a_star.json")
    elif method == 1:
        planning_res = load_json("../out/proposed_geometric_method.json")
    elif method == 2:
        planning_res = load_json("../out/vor_ppm.json")
    else:
        planning_res = load_json("../out/vor.json")

    if key in planning_res:
        scenario_data = scenario[key]["scenario_data"]
        obs_x_vec = scenario_data["obs_x"]
        obs_y_vec = scenario_data["obs_y"]
        data_obs.data.update({"x_vec": obs_x_vec, "y_vec": obs_y_vec})

        target_slot_x_vec = scenario_data["target_corner_x"]
        target_slot_y_vec = scenario_data["target_corner_y"]
        front_slot_x_vec = scenario_data["front_corner_x"]
        rear_slot_x_vec = scenario_data["rear_corner_x"]
        data_slot.data.update({"x_vec": target_slot_x_vec, "y_vec": target_slot_y_vec})
        data_other_slot.data.update(
            {
                "x_vec": [front_slot_x_vec, rear_slot_x_vec],
                "y_vec": [target_slot_y_vec, target_slot_y_vec],
            }
        )

        one_case_res = planning_res[key][case_idx]

        initial_pose = one_case_res["initial_pose"]
        data_initial_pose.data.update(
            {
                "x": [initial_pose[0]],
                "y": [initial_pose[1]],
                "theta": [initial_pose[2]],
            }
        )
        print(
            "initial pose = \n",
            initial_pose[0],
            "\n",
            initial_pose[1],
            "\n",
            initial_pose[2] * 180.0 / math.pi,
        )
        car_global_vertex_x_vec, car_global_vertex_y_vec = load_ego_car_box(
            initial_pose[0],
            initial_pose[1],
            initial_pose[2],
            ego_local_x_vec,
            ego_local_y_vec,
        )

        data_initial_car_vertex.data.update(
            {"x_vec": car_global_vertex_x_vec, "y_vec": car_global_vertex_y_vec}
        )

        if one_case_res["success"] == True:
            path_x_vec = one_case_res["path_x_vec"]
            path_y_vec = one_case_res["path_y_vec"]
            path_heading_vec = one_case_res["path_heading_vec"]

            data_target_pose.data.update(
                {
                    "x": [path_x_vec[-1]],
                    "y": [path_y_vec[-1]],
                    "theta": [path_heading_vec[-1]],
                }
            )

            (
                target_car_global_vertex_x_vec,
                target_car_global_vertex_y_vec,
            ) = load_ego_car_box(
                path_x_vec[-1],
                path_y_vec[-1],
                path_heading_vec[-1],
                ego_local_x_vec,
                ego_local_y_vec,
            )
            data_target_car_vertex.data.update(
                {
                    "x_vec": target_car_global_vertex_x_vec,
                    "y_vec": target_car_global_vertex_y_vec,
                }
            )

            data_path.data.update(
                {
                    "x_vec": path_x_vec,
                    "y_vec": path_y_vec,
                    "theta_vec": path_heading_vec,
                }
            )

            box_x_vec, box_y_vec = load_car_box(
                path_x_vec,
                path_y_vec,
                path_heading_vec,
                ego_local_x_vec,
                ego_local_y_vec,
            )
            data_path_box.data.update(
                {
                    "x_vec": box_x_vec,
                    "y_vec": box_y_vec,
                }
            )

            car_circle_xn = []
            car_circle_yn = []
            car_circle_rn = []
            for k in range(len(path_x_vec)):
                for i in range(len(ego_circle_local_x_vec)):
                    tmp_x, tmp_y = local2global(
                        ego_circle_local_x_vec[i],
                        ego_circle_local_y_vec[i],
                        path_x_vec[k],
                        path_y_vec[k],
                        path_heading_vec[k],
                    )
                    car_circle_xn.append(tmp_x)
                    car_circle_yn.append(tmp_y)
                    car_circle_rn.append(ego_circle_local_r_vec[i])

            data_path_circle.data.update(
                {
                    "x_vec": car_circle_xn,
                    "y_vec": car_circle_yn,
                    "radius_vec": car_circle_rn,
                }
            )

            print("gear shift cnt = ", one_case_res["gear_shift_cnt_slot"])
            print("computation_time = ", one_case_res["computation_time"])
            print("escape_heading = ", one_case_res["escape_heading"] * 180.0 / math.pi)
            print("path points size = ", len(path_x_vec))
        else:
            print("path plan failed!")

    push_notebook()


bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)


