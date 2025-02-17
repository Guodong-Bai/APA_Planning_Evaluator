import glob
import json
import bokeh
import sys, os, copy
import bokeh.plotting as bkp

from math import pi
from io import BytesIO
from pathlib import Path
from bokeh.events import Tap
from bokeh.io import push_notebook, output_notebook
from bokeh.plotting import figure, show, output_file
from bokeh.models import CustomJS, ColumnDataSource, WheelZoomTool

sys.path.append("..")
sys.path.append("../..")
from checker.lib.load_json import *
from checker.lib.load_rotate import *


def transform_obs_car(obs_car, heading, y_offset, x_offset_adjustment):
    transformed_x_vec, transformed_y_vec = [], []
    for x, y in zip(obs_car["obs_x"], obs_car["obs_y"]):
        rotated_x, rotated_y = rotate(x, y, heading)
        rotated_y += y_offset
        transformed_x_vec.append(rotated_x)
        transformed_y_vec.append(rotated_y)

    min_x = min(transformed_x_vec)
    transformed_x_vec = [x - min_x + x_offset_adjustment for x in transformed_x_vec]

    return transformed_x_vec, transformed_y_vec


def generate_car_obs_vec(original_car_pt_vec, car_heading, car_y_offset,
                         obs_to_slot_line, is_front, slot_length):
    dx = slot_length + obs_to_slot_line if is_front else -obs_to_slot_line

    rear_x_vec, rear_y_vec = transform_obs_car(original_car_pt_vec, car_heading,
                                               car_y_offset, dx)
    if is_front:
        min_x = min(rear_x_vec)
        deta_x = -min_x + obs_to_slot_line + slot_length
        rear_x_vec = [x + deta_x for x in rear_x_vec]
    else:
        max_x = max(rear_x_vec)
        deta_x = - max_x - obs_to_slot_line
        rear_x_vec = [x + deta_x for x in rear_x_vec]

    return rear_x_vec, rear_y_vec


def construct_all_scenarios():
    cur_file_path = os.path.abspath(__file__)

    cur_path = os.path.dirname(cur_file_path)

    param_path = os.path.join(cur_path, 'source_config.json')

    data = load_json(param_path)
    scenario_param = data["scenario"]

    slot_width = scenario_param["slot_width"]
    slot_length = scenario_param["slot_length"]
    channel_width_vec = scenario_param["channel_width_vec"]
    lon_available_space_vec = scenario_param["lon_available_space_vec"]

    for channel_width in channel_width_vec:
        for lon_available_space in lon_available_space_vec:
            construct_scenario(slot_width=slot_width,
                               slot_length=slot_length,
                               dx=lon_available_space - slot_length,
                               channel_width=channel_width)
    parent_dir = os.path.dirname(cur_path)
    data_dir = os.path.abspath(os.path.join(parent_dir, 'data'))

    front_files = glob.glob(os.path.join(data_dir, 'front*'))

    return front_files


def construct_scenario(slot_width=2.2,
                       slot_length=6.0,
                       curb_offset=0.4,
                       dx=0.4,
                       channel_width=5.5,
                       front_car_y_offset=0.3,
                       front_car_heading=1.0 / 57.3,
                       rear_car_y_offset=0.0,
                       rear_car_heading=-2.0 / 57.3,
                       is_front_occupied=True,
                       is_rear_occupied=True):
    # channel obs car
    x_offset_vec = [1.2, 3.6, 6.0, 8.9, 12.3]
    y_offset_vec = [-0.3, 0.3, 0.6, 0.2, 0.4]
    heading_offset_vec = [0.0, 0.0, 0.0, 6.0 / 57.3, 70 / 57.3]

    curb_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             '../data/curb.json')
    obs_car_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../data/obs_car_pt.json')

    curb_data = load_json(curb_path)
    obs_car = load_json(obs_car_path)

    # Initialization
    obs_x_vec, obs_y_vec = [], []
    obs_to_slot_line = dx * 0.5
    half_slot_width = 0.5 * slot_width

    # Process front obstacle car
    front_obs_car_matrix = []
    if is_front_occupied:
        front_x_vec, front_y_vec = generate_car_obs_vec(obs_car,
                                                        front_car_heading,
                                                        front_car_y_offset,
                                                        obs_to_slot_line, True,
                                                        slot_length)
        obs_x_vec.extend(front_x_vec)
        obs_y_vec.extend(front_y_vec)
        front_obs_car_matrix = [front_x_vec, front_y_vec]

    # Process rear obstacle car
    rear_obs_car_matrix = []
    if is_rear_occupied:
        rear_x_vec, rear_y_vec = generate_car_obs_vec(obs_car, rear_car_heading,
                                                      rear_car_y_offset,
                                                      obs_to_slot_line, False,
                                                      slot_length)
        obs_x_vec.extend(rear_x_vec)
        obs_y_vec.extend(rear_y_vec)
        rear_obs_car_matrix = [rear_x_vec, rear_y_vec]

    # Target corners
    target_corner_x_vec = [slot_length, 0.0, 0.0, slot_length]
    target_corner_y_vec = [
        half_slot_width, half_slot_width, -half_slot_width, -half_slot_width
    ]

    front_corner_x_vec = [x + slot_length for x in target_corner_x_vec]
    rear_corner_x_vec = [x - slot_length for x in target_corner_x_vec]

    # Process channel cars
    channel_matrix = []
    x_offset_vec = [1.2, 3.6, 6.0, 8.9, 12.3]
    y_offset_vec = [-0.3, 0.3, 0.6, 0.2, 0.4]
    heading_offset_vec = [0.0, 0.0, 0.0, 6.0 / 57.3, 70 / 57.3]

    for x_offset, y_offset, heading_offset in zip(x_offset_vec, y_offset_vec,
                                                  heading_offset_vec):
        channel_x_vec, channel_y_vec = transform_obs_car(
            obs_car,
            -pi / 2 + heading_offset,
            y_offset + half_slot_width + channel_width,
            x_offset,
        )
        min_channel_car_y = min(channel_y_vec)
        channel_y_vec = [
            y - min_channel_car_y + half_slot_width + channel_width
            for y in channel_y_vec
        ]
        obs_x_vec.extend(channel_x_vec)
        obs_y_vec.extend(channel_y_vec)
        channel_matrix.append(channel_x_vec)
        channel_matrix.append(channel_y_vec)

    # Process curb
    curb_x_vec = [x - 4.0 for x in curb_data["obs_x"]]
    curb_y_vec = [y - half_slot_width - curb_offset for y in curb_data["obs_y"]]
    obs_x_vec.extend(curb_x_vec)
    obs_y_vec.extend(curb_y_vec)

    data = {
        "obs_x": obs_x_vec,
        "obs_y": obs_y_vec,
        "target_corner_x": target_corner_x_vec,
        "target_corner_y": target_corner_y_vec,
        "front_corner_x": front_corner_x_vec,
        "rear_corner_x": rear_corner_x_vec,
        "channel_matrix": channel_matrix,
        "front_obs_car_matrix": front_obs_car_matrix,
        "rear_obs_car_matrix": rear_obs_car_matrix,
    }

    current_dir = os.path.dirname(os.path.abspath(__file__))
    output_folder = os.path.join(current_dir, '../data')

    output_file_name = os.path.join(
        output_folder,
        f"{'front_occupied' if is_front_occupied else 'front_vacant'}_"
        f"{'rear_occupied' if is_rear_occupied else 'rear_vacant'}_"
        f"lon_{slot_length+dx}_channel_width_{channel_width}.json")
    save_json(data, output_file_name)


if __name__ == "__main__":
    construct_scenario()
    name = "../data/front_occupied_rear_occupied_lon_6.4_channel_width_5.5.json"

    data = load_json(name)

    output_notebook()
    # ## for debug json data
    fig1 = bkp.figure(
        x_axis_label="x",
        y_axis_label="y",
        width=600,
        height=600,
        match_aspect=True,
        aspect_scale=1,
    )
    source = ColumnDataSource(data=dict(x=[], y=[]))
    fig1.circle("x",
                "y",
                size=10,
                source=source,
                color="red",
                legend_label="measure tool")
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
        args=dict(source=source,
                  line_source=line_source,
                  text_source=text_source),
        code=callback_code,
    )

    # Attach the callback to the Tap event on the plot
    fig1.js_on_event(Tap, callback)

    fig1.circle(data["obs_x"], data["obs_y"], size=3, color="green", alpha=0.5)

    fig1.patches(
        xs=[data["target_corner_x"]],
        ys=[data["target_corner_y"]],
        fill_color=["blue"],
        line_color="black",
        alpha=0.3,
    )

    # toolbar
    fig1.toolbar.active_scroll = fig1.select_one(WheelZoomTool)
    # legend
    # fig1.legend.click_policy = 'hide'

    handle = show(fig1, notebook_handle=True)
    push_notebook(handle=handle)
