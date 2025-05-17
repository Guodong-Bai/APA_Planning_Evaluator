import sys, os, json
from math import cos, fabs, pi, sin



sys.path.append("..")
sys.path.append("../..")

sys.path.append("../../simulation/")

from simulation.code.APA_Planning.jupyter_pybind.notebooks_apa.lib.load_json import write_json
from task.generate_planning_result import update_planning_for_pose
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

# sys.path.append('/root/miniconda3/lib/python3.7/site-packages')

kRad2Deg = 180.0 / pi
kDeg2Rad = pi / 180.0

display(HTML("<style>.container { width:95% !important;  }</style>"))
output_notebook()

param_data = load_json("/asw/planning/res/conf/apa_params.json")
is_use_box = param_data["is_col_oa_use_box"]

if is_use_box:
    car_xb, car_yb = load_car_box()
else:
    car_xb, car_yb, _ = load_car_params_patch_parking(
        vehicle_type=CHERY_E0X, car_lat_inflation=0.0
    )
car_circle_xb, car_circle_yb, car_circle_r = load_car_circle_coord()


input_data = load_json("../out/initial_pose_obs_slot.json")
output_file_path = "../out/hybrid_a_star.json"


coord_tf = coord_transformer()

data_start_car = ColumnDataSource(data={"car_yn": [], "car_xn": []})
data_start_car_circle = ColumnDataSource(
    data={"car_yn": [], "car_xn": [], "radius": []}
)
data_target_car = ColumnDataSource(data={"car_yn": [], "car_xn": []})
data_PA = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_AB = ColumnDataSource(data=dict(x=[0], y=[0], radius=[1]))
data_BT = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_start_pos = ColumnDataSource(data={"x": [], "y": []})
data_target_pos = ColumnDataSource(data={"x": [], "y": []})

data_path = ColumnDataSource(data={"x_vec": [], "y_vec": [], "theta_vec": []})
data_preparing_step_path = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_preparing_line_path = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_parking_out_path = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_in_slot_path = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_all_debug_path = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_car_box = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_slot = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_other_slot = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_fus_obs = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_obs_pt = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_virtual_obs_pt = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_tra_tb_pt = ColumnDataSource(data={"x_vec": [], "y_vec": []})

data_debug_arc = ColumnDataSource(
    data={
        "cx_vec": [],
        "cy_vec": [],
        "radius_vec": [],
        "pBx_vec": [],
        "pBy_vec": [],
        "pCx_vec": [],
        "pCy_vec": [],
    }
)
data_extra_region = ColumnDataSource(data={"x_vec": [], "y_vec": []})
data_obs_car_polygon = ColumnDataSource(data={"x_vec": [], "y_vec": []})

def reset_data():
    data_path.data.update(
        {
            "x_vec": [],
            "y_vec": [],
            "theta_vec": [],
        }
    )

    data_all_debug_path.data.update(
        {
            "x_vec": [],
            "y_vec": [],
        }
    )

    data_car_box.data.update(
        {
            "x_vec": [],
            "y_vec": [],
        }
    )

    # data_virtual_obs_pt.data.update({"x_vec": [], "y_vec": []})



fig1 = bkp.figure(width=1200, height=800, match_aspect=True, aspect_scale=1)
# fig1.background_fill_color = "#E0E0E0"  # 你也可以用 '#D3D3D3'
fig1.x_range.flipped = False

fig1.outline_line_color = "black"
fig1.outline_line_width = 1.0  # 可以调整边框线条的宽度

fig1.x_range = Range1d(start=-6.0, end=14.0)
fig1.y_range = Range1d(start=-3.0, end=9.0)

fig1.xaxis.axis_label_text_font_size = "18pt"
fig1.xaxis.axis_label_text_font = "Times New Roman"

fig1.yaxis.axis_label_text_font_size = "18pt"
fig1.yaxis.axis_label_text_font = "Times New Roman"

fig1.xaxis.major_label_text_font_size = "18pt"  # 设置x轴字体大小
fig1.xaxis.major_label_text_font = "Times New Roman"  # 设置字体类型

fig1.yaxis.major_label_text_font_size = "18pt"
fig1.yaxis.major_label_text_font = "Times New Roman"

fig1.xaxis.ticker = SingleIntervalTicker(interval=4, num_minor_ticks=0)
fig1.yaxis.ticker = SingleIntervalTicker(interval=4, num_minor_ticks=0)

fig1.xgrid.grid_line_color = None
fig1.ygrid.grid_line_color = None


# # 去除图形四周边框
# fig1.outline_line_color = None
# fig1.xaxis.visible = False
# fig1.yaxis.visible = False
# fig1.xaxis.major_label_text_font_size = '0pt'  # 设置字体大小
# fig1.yaxis.major_label_text_font_size = '0pt'


# 尝试确保图表内容比例一致
aspect_ratio = (fig1.x_range.end - fig1.x_range.start) / (
    fig1.y_range.end - fig1.y_range.start
)
fig1.plot_height = int(fig1.plot_width / aspect_ratio)

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

# target slot
fig1.line(
    "x_vec",
    "y_vec",
    source=data_slot,
    line_width=2.0,
    line_color="black",
    line_dash="solid",
    legend_label="slot",
    visible=True,
)

# nearby slots
fig1.multi_line(
    "x_vec",
    "y_vec",
    source=data_other_slot,
    line_width=2.0,
    line_color="black",
    line_dash="solid",
    legend_label="slot",
    visible=True,
)

# obstacles
fig1.scatter(
    "x_vec",
    "y_vec",
    source=data_fus_obs,
    size=3,
    color="grey",
    legend_label="obstacles",
)

fig1.patches(
    "x_vec",
    "y_vec",
    source=data_car_box,
    fill_color="#98FB98",
    fill_alpha=0.0,
    line_color="black",
    line_width=0.2,
    legend_label="Envelope",
)

# car box at start pose
fig1.circle(
    "x", "y", source=data_start_pos, size=8, color="black", legend_label="Start pose"
)
fig1.patch(
    "car_xn",
    "car_yn",
    source=data_start_car,
    fill_color="red",
    fill_alpha=0.2,
    line_color="black",
    line_width=0.5,
    legend_label="Start pose",
)

fig1.circle(
    x="car_xn",
    y="car_yn",
    radius="radius",
    source=data_start_car_circle,
    fill_color=None,
    fill_alpha=0.0,
    line_color="black",
    line_width=0.5,
)

# target pose
fig1.circle(
    "x", "y", source=data_target_pos, size=8, color="black", legend_label="Target pose"
)
fig1.patch(
    "car_xn",
    "car_yn",
    source=data_target_car,
    fill_color="red",
    fill_alpha=0.3,
    line_color="black",
    line_width=1,
    legend_label="Target pose",
)

fig1.legend.label_text_font = "Times New Roman"  # 设置图例字体类型
fig1.legend.label_text_font_size = "14pt"  # 设置图例字体大小
fig1.legend.location = "top_left"

# fig1.legend.visible = False

fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
fig1.legend.click_policy = "hide"


### sliders config
class LocalViewSlider:
    def __init__(self, slider_callback):
        self.select_pose_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width="15%"),
            description="is select pose",
            min=0,
            max=1,
            value=0,
            step=1,
        )

        self.scenario_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width="55%"),
            description="scenario key",
            min=0,
            max=50,
            value=0,
            step=1,
        )

        self.case_idx_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width="60%"),
            description="pose case idx",
            min=0,
            max=200,
            value=0,
            step=1,
        )

        # ego pose
        self.ego_x_slider = ipywidgets.FloatSlider(
            layout=ipywidgets.Layout(width="75%"),
            description="ego_x",
            min=-15,
            max=15,
            value=6.27,
            step=0.01,
        )
        self.ego_y_slider = ipywidgets.FloatSlider(
            layout=ipywidgets.Layout(width="75%"),
            description="ego_y",
            min=-10,
            max=10,
            value=2.8960951861241053,
            step=0.01,
        )
        self.ego_heading_slider = ipywidgets.FloatSlider(
            layout=ipywidgets.Layout(width="75%"),
            description="ego_heading",
            min=-180,
            max=180,
            value=-0.5133240215115417,
            step=0.1,
        )

        self.ds_slider = ipywidgets.FloatSlider(
            layout=ipywidgets.Layout(width="75%"),
            description="path ds",
            min=0.025,
            max=1.0,
            value=0.3,
            step=0.025,
        )

        ipywidgets.interact(
            slider_callback,
            select_pose=self.select_pose_slider,
            scenario = self.scenario_slider,
            case_idx=self.case_idx_slider,
            ego_x=self.ego_x_slider,
            ego_y=self.ego_y_slider,
            ego_heading=self.ego_heading_slider,
            ds=self.ds_slider
        )


### sliders callback
def slider_callback(select_pose, scenario, case_idx, ego_x, ego_y, ego_heading, ds):

    scenario_key = str(scenario)
    certain_scenario_data = input_data[scenario_key]
    scenario_data = certain_scenario_data["scenario_data"]

    obs_x_vec = scenario_data["obs_x"]
    obs_y_vec = scenario_data["obs_y"]

    target_corner_x_vec = scenario_data["target_corner_x"]
    target_corner_y_vec = scenario_data["target_corner_y"]

    front_corner_x_vec = scenario_data["front_corner_x"]
    rear_corner_x_vec = scenario_data["rear_corner_x"]

    ego_pose = [ego_x, ego_y, ego_heading * kDeg2Rad]
    if not select_pose:
        ego_pose = certain_scenario_data["initial_pose"][case_idx]
        print("ego_pose = ", ego_pose)

    slot_points = [ target_corner_x_vec, target_corner_y_vec]

    # obstacles
    data_fus_obs.data.update(
        {
            "x_vec": obs_x_vec,
            "y_vec": obs_y_vec,
        }
    )

    data_slot.data.update({"x_vec": target_corner_x_vec, "y_vec": target_corner_y_vec})
    data_other_slot.data.update(
        {
            "x_vec": [front_corner_x_vec, rear_corner_x_vec],
            "y_vec": [target_corner_y_vec, target_corner_y_vec],
        }
    )

    # ego start position
    data_start_pos.data.update({"x": [ego_pose[0]], "y": [ego_pose[1]]})

    coord_tf = coord_transformer(ego_pose[0], ego_pose[1], ego_pose[2])
    car_xn, car_yn = coord_tf.local_to_global(car_xb, car_yb)
    data_start_car.data.update(
        {
            "car_xn": car_xn,
            "car_yn": car_yn,
        }
    )

    car_circle_xn, car_circle_yn = coord_tf.local_to_global(
        car_circle_xb, car_circle_yb
    )

    data_start_car_circle.data.update(
        {"car_xn": car_circle_xn, "car_yn": car_circle_yn, "radius": car_circle_r}
    )

    reset_data()

    print("ego_pose = ", ego_pose)
    res = update_planning_for_pose(ego_pose, obs_x_vec, obs_y_vec, slot_points)

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


    if res["success"]:
        path_x_vec = res["path_x_vec"]
        path_y_vec = res["path_y_vec"]
        path_heading_vec = res["path_heading_vec"]

        # update path
        data_path.data.update(
            {
                "x_vec": path_x_vec,
                "y_vec": path_y_vec,
                "theta_vec": path_heading_vec,
            }
        )

        # target pos
        target_pos_x = path_x_vec[-1]
        target_pos_y = path_y_vec[-1]
        target_heading = path_heading_vec[-1]
        data_target_pos.data.update(
            {
                "x": [target_pos_x],
                "y": [target_pos_y],
            }
        )
        # target car
        target_car_xn, target_car_yn = ego_vertex_l2g(target_pos_x, target_pos_y, target_heading, car_xb, car_yb)
        data_target_car.data.update(
            {
                "car_xn": target_car_xn,
                "car_yn": target_car_yn,
            }
        )

        # path ego car
        car_box_x_vec, car_box_y_vec = ego_vertex_path_l2g(path_x_vec, path_y_vec, path_heading_vec, car_xb, car_yb)
        data_car_box.data.update(
            {
                "x_vec": car_box_x_vec,
                "y_vec": car_box_y_vec,
            }
        )


    # output_dict = {}
    # for key in input_data:
    #     certain_scenario_data = input_data[key]
    #     scenario_data = certain_scenario_data["scenario_data"]

    #     obs_x_vec = scenario_data["obs_x"]
    #     obs_y_vec = scenario_data["obs_y"]

    #     target_corner_x_vec = scenario_data["target_corner_x"]
    #     target_corner_y_vec = scenario_data["target_corner_y"]

    #     front_corner_x_vec = scenario_data["front_corner_x"]
    #     rear_corner_x_vec = scenario_data["rear_corner_x"]


    #     ego_pose = certain_scenario_data["initial_pose"][0]
    #     print("ego_pose = ", ego_pose)

    #     slot_points = [ target_corner_x_vec, target_corner_y_vec]
    #     res = update_planning_for_pose(ego_pose, obs_x_vec, obs_y_vec, slot_points)
    #     print("res after = ", res)
    #     one_scenario_data = [res]
    #     output_dict[key] = one_scenario_data
    # write_json(output_file_path, output_dict)



    push_notebook()


bkp.show(row(fig1), notebook_handle=True)
slider_class = LocalViewSlider(slider_callback)

# os.environ['WEB_BROWSER'] = 'chrome'
# fig1.output_backend = "svg"
# export_svgs(fig1, filename="illustration.svg")

# svg_file_path = 'illustration.svg'
# eps_file_path = 'illustration.eps'

# # 使用 CairoSVG 直接转换
# cairosvg.svg2eps(url=svg_file_path, write_to=eps_file_path)

# print(f"SVG 文件已直接转换为 EPS：'{eps_file_path}'")

# cairosvg.svg2pdf(url="illustration.svg", write_to="illustration.pdf")
