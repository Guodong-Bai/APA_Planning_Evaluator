import sys, json, cairosvg, numpy as np
from bokeh.plotting import figure, show, output_file
from bokeh.layouts import column
from bokeh.models import SingleIntervalTicker, Legend, LegendItem

# 添加搜索路径（根据需要修改）
sys.path.append("..")
sys.path.append("../out/")
sys.path.append("../lib/")
from lib.load_json import load_json, load_json_keys

# 全局样式参数
line_colors  = ["blue", "red"]
line_widths  = [1.0, 1.5]
line_styles  = ["solid", "dashed"]
markers      = ["triangle", "circle"]
legends      = ["Vor + PPM", "Proposed"]

font_size    = "28pt"
font_family  = "Times New Roman"
fig_length   = 1200
fig_height   = 500
x_start      = 5.8
x_end        = 7.4

# JSON 文件路径
tra_ppm_geo_file_path = "../out/traditional_geometric_method.json"
proposed_file_path    = "../out/proposed_geometric_method.json"
file_paths            = [tra_ppm_geo_file_path, proposed_file_path]

def set_fig_text(p):
    p.xaxis.axis_label_text_font_size = font_size
    p.yaxis.axis_label_text_font_size = font_size
    p.xaxis.major_label_text_font_size = font_size
    p.yaxis.major_label_text_font_size = font_size

    p.xaxis.axis_label_text_font = font_family
    p.yaxis.axis_label_text_font = font_family
    p.xaxis.major_label_text_font = font_family
    p.yaxis.major_label_text_font = font_family

    p.xaxis.axis_label_text_color = "black"
    p.yaxis.axis_label_text_color = "black"
    p.xaxis.major_label_text_color = "black"
    p.yaxis.major_label_text_color = "black"

    p.xaxis.major_label_standoff = 15
    p.yaxis.major_label_standoff = 15
    p.min_border_left = 40
    p.min_border_bottom = 40
    p.outline_line_color = "black"
    p.outline_line_width = 1.0

def create_figure(x_label, y_label, x_interval, y_range, y_interval):
    p = figure(x_axis_label=x_label, y_axis_label=y_label,
               width=fig_length, height=fig_height)
    p.x_range.start = x_start
    p.x_range.end = x_end
    p.xaxis.ticker = SingleIntervalTicker(interval=x_interval, num_minor_ticks=0)
    p.y_range.start = y_range[0]
    p.y_range.end = y_range[1]
    p.yaxis.ticker = SingleIntervalTicker(interval=y_interval, num_minor_ticks=0)
    set_fig_text(p)
    # 关闭竖直方向的网格线
    p.xgrid.visible = False
    return p

def load_methods_data(file_paths):
    """
    加载各方法数据，返回列表，每个元素为字典，包含：
      - "x": 横坐标列表
      - "gear_shift": 换挡次数列表
      - "escape_heading": 逃逸航向（度）列表
      - "oa_dist": 转角避障距离列表
    """
    keys = load_json_keys(proposed_file_path)
    print("keys =", keys[0], "--", keys[-1])
    methods_data = []
    for fp in file_paths:
        data = load_json(fp)
        xs, gear_shifts, escapes, oa_dists = [], [], [], []
        for i, key in enumerate(keys):
            res = data[key][0]
            if res["success"]:
                xs.append(x_start + i * 0.05)
                gear_shifts.append(res["gear_shift_cnt_slot"])
                escapes.append(res["escape_heading"] * 57.3)  # 转换为角度
                oa_dists.append(res["corner_obstacle_avoidance_dist"])
        methods_data.append({
            "x": xs,
            "gear_shift": gear_shifts,
            "escape_heading": escapes,
            "oa_dist": oa_dists,
        })
    return methods_data

def plot_line_and_marker(p, x, y, method_index):
    """绘制线型与标记，并返回对应的两个 glyph renderer"""
    color = line_colors[method_index]
    lw    = line_widths[method_index]
    dash  = line_styles[method_index]
    marker_type = markers[method_index]
    r_line = p.line(x, y, line_color=color, line_width=lw, line_dash=dash)
    if marker_type == "circle":
        r_marker = p.circle(x, y, size=8, color=color)
    elif marker_type == "triangle":
        r_marker = p.triangle(x, y, size=8, color=color)
    elif marker_type == "square":
        r_marker = p.square(x, y, size=8, color=color)
    else:
        r_marker = None
    return r_line, r_marker

def plot_metric(p, methods_data, metric_key, show_legend=False):
    """
    在图 p 上为两种方法绘制 metric_key 对应的数据，
    当 show_legend 为 True 时，将线型与标记合并到 legend 中，
    否则不显示 legend。
    """
    legend_items = []
    for i in range(len(methods_data)):
        x = methods_data[i]["x"]
        y = methods_data[i][metric_key]
        r_line, r_marker = plot_line_and_marker(p, x, y, i)
        if show_legend:
            legend_items.append(LegendItem(label=legends[i], renderers=[r_line, r_marker]))
    if show_legend:
        legend = Legend(
            items=legend_items,
            location="top_right",
            label_text_font=font_family,
            label_text_font_size=font_size,
            label_standoff=15,           # 线和文字之间的间距
            glyph_width=70,              # 线的水平长度
            glyph_height=28,             # 线的高度，与字体高度相近
            label_text_baseline="middle" # 文字垂直居中
        )
        p.add_layout(legend)

methods_data = load_methods_data(file_paths)

# 绘制三个图：换挡次数、逃逸航向和转角避障距离（仅第一个图显示 legend）
p1 = create_figure("Parking space length (m)", "Gear shift count", x_interval=0.4, y_range=(0, 10), y_interval=2)
plot_metric(p1, methods_data, "gear_shift", show_legend=True)

p2 = create_figure("Parking space length (m)", "Escape heading (deg)", x_interval=0.4, y_range=(0, 50), y_interval=10)
plot_metric(p2, methods_data, "escape_heading", show_legend=False)

p3 = create_figure("Parking space length (m)", "Distance (m)", x_interval=0.4, y_range=(0, 1.5), y_interval=0.3)
plot_metric(p3, methods_data, "oa_dist", show_legend=False)

# 设置输出后端为 SVG
for p in (p1, p2, p3):
    p.output_backend = "svg"

# 输出到 HTML 文件，便于浏览器查看
output_file("bokeh_plots.html", title="Proposed Geometric Method Plots")

# 分别导出为 SVG 文件
from bokeh.io.export import export_svgs
export_svgs(p1, filename="gear_shift_cnt.svg")
export_svgs(p2, filename="escape_heading.svg")
export_svgs(p3, filename="corner_oa.svg")

# 将 SVG 转换为 PDF 文件
cairosvg.svg2pdf(url="gear_shift_cnt.svg", write_to="gear_shift_cnt.pdf")
cairosvg.svg2pdf(url="escape_heading.svg", write_to="escape_heading.pdf")
cairosvg.svg2pdf(url="corner_oa.svg", write_to="corner_oa.pdf")

# 显示三个图表
show(column(p1, p2, p3))
