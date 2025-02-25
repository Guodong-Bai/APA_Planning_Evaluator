import cairosvg
import sys, json
import numpy as np
from bokeh.plotting import figure, show, output_file
from bokeh.layouts import column
from bokeh.models import ColumnDataSource, SingleIntervalTicker
from bokeh.io.export import export_svgs

# 添加搜索路径（根据需要修改）
sys.path.append("..")
sys.path.append("../out/")
sys.path.append("../lib/")

from lib.load_json import load_json, load_json_keys

line_colors = ["blue", "red"]
line_widths = [1.5, 1]
line_styles = ["solid", "solid"]
markers = ["triangle", "circle"]
legends = ["Vor + PPM", "Proposed"]

def add_markers(p, multi_x_vec, multi_shift_cnt_vec, size=8, is_first_fig = False):

    for i in range(len(multi_x_vec)):
        if is_first_fig:
            if markers[i] == "circle":
                p.circle(multi_x_vec[i], multi_shift_cnt_vec[i], size=size, color=line_colors[i], legend_label=legends[i])
            elif markers[i] == "triangle":
                p.triangle(multi_x_vec[i], multi_shift_cnt_vec[i],
                        size=size, color=line_colors[i], legend_label=legends[i])
            elif markers[i] == "square":
                p.square(multi_x_vec[i], multi_shift_cnt_vec[i],
                        size=size, color=line_colors[i], legend_label=legends[i])
        else:
            if markers[i] == "circle":
                p.circle(multi_x_vec[i], multi_shift_cnt_vec[i], size=size, color=line_colors[i])
            elif markers[i] == "triangle":
                p.triangle(multi_x_vec[i], multi_shift_cnt_vec[i],
                        size=size, color=line_colors[i])
            elif markers[i] == "square":
                p.square(multi_x_vec[i], multi_shift_cnt_vec[i],
                        size=size, color=line_colors[i])


def set_fig_text(p1):
    p1.xaxis.axis_label_text_font_size = font_size
    p1.yaxis.axis_label_text_font_size = font_size
    p1.xaxis.major_label_text_font_size = font_size
    p1.yaxis.major_label_text_font_size = font_size

    p1.xaxis.axis_label_text_font = font_family
    p1.yaxis.axis_label_text_font = font_family
    p1.xaxis.major_label_text_font = font_family
    p1.yaxis.major_label_text_font = font_family

    p1.xaxis.axis_label_text_color = "black"
    p1.yaxis.axis_label_text_color = "black"
    p1.xaxis.major_label_text_color = "black"
    p1.yaxis.major_label_text_color = "black"

    p1.xaxis.major_label_standoff = 15
    p1.yaxis.major_label_standoff = 15
    p1.min_border_left = 40
    p1.min_border_bottom = 40

# 统一设置字体大小参数
font_size = "28pt"
font_family = "Times New Roman"
fig_length = 1200
fig_height = 500
x_start = 5.8
x_end = 7.2

# JSON 文件路径
tra_ppm_geo_file_path = "../out/traditional_geometric_method.json"
proposed_file_path = "../out/proposed_geometric_method.json"

file_path_vec = [tra_ppm_geo_file_path, proposed_file_path]

data_vec = []

keys = load_json_keys(proposed_file_path)
print("keys = ", keys[0], " -- ", keys[-1])

for file in file_path_vec:
    data = load_json(file)
    data_vec.append(data)


multi_x_vec = []
multi_shift_cnt_vec = []
multi_escape_heading_vec = []
multi_corner_oa_dist_vec = []

# 生成横坐标数据，以0.05间隔，从5.8开始
for one_method_data in data_vec:
    x_vec = []
    gear_shift_vec = []
    escape_headng_vec = []
    corner_oa_dist_vec = []
    for i in range(len(keys)):
        key = keys[i]
        single_planning_res = one_method_data[key][0]

        if single_planning_res["success"]:
            x_vec.append(5.8 + i * 0.05)

            gear_shift_vec.append(single_planning_res["gear_shift_cnt_slot"])
            escape_headng_vec.append(single_planning_res["escape_heading"] * 57.3)
            corner_oa_dist_vec.append(
                single_planning_res["corner_obstacle_avoidance_dist"]
            )
    multi_x_vec.append(x_vec)
    multi_shift_cnt_vec.append(gear_shift_vec)
    multi_escape_heading_vec.append(escape_headng_vec)
    multi_corner_oa_dist_vec.append(corner_oa_dist_vec)

# 使用 ColumnDataSource 封装数据
source_shift = ColumnDataSource(data=dict(
    x=multi_x_vec,
    y=multi_shift_cnt_vec,
    color=line_colors,
    width=line_widths,
    dash=line_styles
))

source_escape = ColumnDataSource(data=dict(
    x=multi_x_vec,
    y=multi_escape_heading_vec,
    color=line_colors,
    width=line_widths,
    dash=line_styles
))

source_oa_dist = ColumnDataSource(data=dict(
    x=multi_x_vec,
    y=multi_corner_oa_dist_vec,
    color=line_colors,
    width=line_widths,
    dash=line_styles
))


# 绘制“Gear shift count in slot”图
p1 = figure(
    x_axis_label="Parking space length (m)",
    y_axis_label="Gear shift count",
    width=fig_length,
    height=fig_height,
)
p1.multi_line('x', 'y', source=source_shift,
             line_color='color',
             line_width='width',
             line_dash='dash')
add_markers(p1, multi_x_vec, multi_shift_cnt_vec, size=8, is_first_fig = True)

p1.x_range.start = x_start
p1.x_range.end = x_end
p1.xaxis.ticker = SingleIntervalTicker(interval=0.5, num_minor_ticks=0)
p1.y_range.start = 0
p1.y_range.end = 10
p1.yaxis.ticker = SingleIntervalTicker(interval=2, num_minor_ticks=0)
p1.legend.location = "top_right"
set_fig_text(p1)
# 设置图例的字体及大小
p1.legend.label_text_font = font_family
p1.legend.label_text_font_size = font_size


# 绘制“Escape Heading (deg)”图
p2 = figure(x_axis_label="Parking space length (m)",
            y_axis_label="Escape heading (deg)",
            width=fig_length, height=fig_height)
p2.multi_line('x', 'y', source=source_escape,
             line_color='color',
             line_width='width',
             line_dash='dash')
add_markers(p2, multi_x_vec, multi_escape_heading_vec, size=8)
p2.x_range.start = x_start
p2.x_range.end = x_end
p2.xaxis.ticker = SingleIntervalTicker(interval=0.5, num_minor_ticks=0)
p2.y_range.start = 0
p2.y_range.end = 50
p2.yaxis.ticker = SingleIntervalTicker(interval=10, num_minor_ticks=0)
set_fig_text(p2)


p3 = figure(x_axis_label="Parking space length (m)",
            y_axis_label="Distance (m)",
            width=fig_length, height=fig_height)
p3.multi_line('x', 'y', source=source_oa_dist,
             line_color='color',
             line_width='width',
             line_dash='dash')
add_markers(p3, multi_x_vec, multi_corner_oa_dist_vec, size=8)
p3.x_range.start = x_start
p3.x_range.end = x_end
p3.xaxis.ticker = SingleIntervalTicker(interval=0.5, num_minor_ticks=0)
p3.y_range.start = 0
p3.y_range.end = 1.5
p3.yaxis.ticker = SingleIntervalTicker(interval=0.3, num_minor_ticks=0)
set_fig_text(p3)
# 设置输出后端为SVG
p1.output_backend = "svg"
p2.output_backend = "svg"
p3.output_backend = "svg"


# # 输出到 HTML 文件（供浏览器查看图表）
output_file("bokeh_plots.html", title="Proposed Geometric Method Plots")

# 分别导出为SVG文件
export_svgs(p1, filename="gear_shift_cnt.svg")
export_svgs(p2, filename="escape_heading.svg")
export_svgs(p3, filename="corner_oa.svg")


# 将 SVG 转换为 PDF
cairosvg.svg2pdf(url="gear_shift_cnt.svg", write_to="gear_shift_cnt.pdf")
cairosvg.svg2pdf(url="escape_heading.svg", write_to="escape_heading.pdf")
cairosvg.svg2pdf(url="corner_oa.svg", write_to="corner_oa.pdf")


# # 显示两个图表
show(column(p1, p2, p3))
