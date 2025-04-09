# import math
# import sys, json, cairosvg, numpy as np
# from bokeh.plotting import figure, show, output_file
# from bokeh.layouts import column
# from bokeh.models import SingleIntervalTicker, Legend, LegendItem, FixedTicker, BoxAnnotation, NormalHead, HoverTool
# from bokeh.models import Label, Arrow, OpenHead
# # 添加搜索路径（根据需要修改）
# sys.path.append("..")
# sys.path.append("../out/")
# sys.path.append("../lib/")
# from lib.load_json import load_json, load_json_keys

# # 全局样式参数
# line_colors  = ["green", "blue", "red", "black"]
# line_widths  = [ 2.0, 1.5, 1.5, 3.0]
# line_styles  = [ "solid", "dotted", "dashed", "solid"]
# markers      = [ "square", "triangle", "asterisk", "no_fill_circle"]
# legends      = [ "Vor", "Vor + PPM", "Hybrid a star", "Proposed"]

# font_size    = "28pt"
# font_family  = "Times New Roman"
# fig_length   = 1200
# fig_height   = 500
# x_start      = 5.8
# x_end        = 7.4

# # JSON 文件路径

# tra_file_path = "../out/vor.json"
# tra_ppm_geo_file_path = "../out/vor_ppm.json"
# hybrid_a_star_file_path = "../out/hybrid_a_star.json"
# proposed_file_path    = "../out/proposed_geometric_method.json"
# file_paths            = [tra_file_path, tra_ppm_geo_file_path, hybrid_a_star_file_path, proposed_file_path]

# def set_fig_text(p):
#     p.xaxis.axis_label_text_font_size = font_size
#     p.yaxis.axis_label_text_font_size = font_size
#     p.xaxis.major_label_text_font_size = font_size
#     p.yaxis.major_label_text_font_size = font_size

#     p.xaxis.axis_label_text_font_style = "normal"   # 设置为正常字体
#     p.yaxis.axis_label_text_font_style = "normal"
#     p.xaxis.major_label_text_font_style = "normal"
#     p.yaxis.major_label_text_font_style = "normal"

#     p.xaxis.axis_label_text_font = font_family
#     p.yaxis.axis_label_text_font = font_family
#     p.xaxis.major_label_text_font = font_family
#     p.yaxis.major_label_text_font = font_family

#     p.xaxis.axis_label_text_color = "black"
#     p.yaxis.axis_label_text_color = "black"
#     p.xaxis.major_label_text_color = "black"
#     p.yaxis.major_label_text_color = "black"


#     p.xaxis.major_label_standoff = 15
#     p.yaxis.major_label_standoff = 15
#     p.min_border_left = 40
#     p.min_border_bottom = 40
#     p.outline_line_color = "black"
#     p.outline_line_width = 1.0

# def create_figure(x_label, y_label, x_interval, y_range, y_interval):
#     p = figure(x_axis_label=x_label, y_axis_label=y_label,
#                width=fig_length, height=fig_height)
#     p.x_range.start = x_start
#     p.x_range.end = x_end

#     num_ticks = int(round((x_end - x_start) / x_interval)) + 1
#     x_ticks = [round(x_start + i * x_interval, 2) for i in range(num_ticks)]
#     p.xaxis.ticker = FixedTicker(ticks=x_ticks)

#     p.y_range.start = y_range[0]
#     p.y_range.end = y_range[1]

#     # 计算 y 轴的刻度，使得最大刻度不超过 y_range[1]（例如：0 到 11 范围内，最大刻度为 10）
#     max_tick = math.floor((y_range[1] - y_range[0]) / y_interval) * y_interval
#     num_y_ticks = int((max_tick - y_range[0]) / y_interval) + 1
#     y_ticks = [round(y_range[0] + i * y_interval, 2) for i in range(num_y_ticks)]
#     p.yaxis.ticker = FixedTicker(ticks=y_ticks)

#     set_fig_text(p)
#     p.xgrid.visible = False
#     return p

# def load_methods_data(file_paths):
#     """
#     加载各方法数据，返回列表，每个元素为字典，包含：
#       - "x": 横坐标列表
#       - "gear_shift": 换挡次数列表
#       - "escape_heading": 逃逸航向（度）列表
#       - "oa_dist": 转角避障距离列表
#     """
#     keys = load_json_keys(proposed_file_path)
#     print("keys =", keys[0], "--", keys[-1])
#     methods_data = []
#     for fp in file_paths:
#         data = load_json(fp)
#         xs, gear_shifts, escapes, oa_dists = [], [], [], []
#         for i, key in enumerate(keys):
#             res = data[key][0]
#             if res["success"]:
#                 xs.append(x_start + i * 0.05)
#                 gear_shifts.append(res["gear_shift_cnt_slot"])
#                 escapes.append(res["escape_heading"] * 57.3)  # 转换为角度
#                 oa_dists.append(res["corner_obstacle_avoidance_dist"])
#         methods_data.append({
#             "x": xs,
#             "gear_shift": gear_shifts,
#             "escape_heading": escapes,
#             "oa_dist": oa_dists,
#         })
#     return methods_data

# def plot_line_and_marker(p, x, y, method_index):
#     """绘制线型与标记，并返回对应的两个 glyph renderer"""
#     color = line_colors[method_index]
#     lw    = line_widths[method_index]
#     dash  = line_styles[method_index]
#     marker_type = markers[method_index]
#     r_line = p.line(x, y, line_color=color, line_width=lw, line_dash=dash)
#     if marker_type == "circle":
#         r_marker = p.circle(x, y, size=10, color=color)
#     elif marker_type == "no_fill_circle":
#         r_marker = p.circle(x, y, size=12, fill_color=None, line_color=color, line_width=lw)
#     elif marker_type == "triangle":
#         r_marker = p.triangle(x, y, size=10, color=color)
#     elif marker_type == "square":
#         r_marker = p.square(x, y, size=10, color=color)
#     elif marker_type == "cross":
#         r_marker = p.cross(x, y, size=12, color=color)
#     elif marker_type == "asterisk":
#         r_marker = p.asterisk(x, y, size=14, line_color=color, line_width=lw)
#     return r_line, r_marker


# def plot_metric(p, methods_data, metric_key, show_legend=False):
#     """
#     在图 p 上为两种方法绘制 metric_key 对应的数据，
#     当 show_legend 为 True 时，将线型与标记合并到 legend 中，
#     否则不显示 legend。
#     """
#     legend_items = []
#     for i in range(len(methods_data)):
#         x = methods_data[i]["x"]
#         y = methods_data[i][metric_key]
#         r_line, r_marker = plot_line_and_marker(p, x, y, i)
#         if show_legend:
#             legend_items.append(LegendItem(label=legends[i], renderers=[r_line, r_marker]))
#     if show_legend:
#         legend = Legend(
#             items=legend_items,
#             location="top_right",
#             label_text_font=font_family,
#             label_text_font_size=font_size,
#             label_standoff=15,           # 线和文字之间的间距
#             glyph_width=70,              # 线的水平长度
#             glyph_height=28,             # 线的高度，与字体高度相近
#             label_text_baseline="middle" # 文字垂直居中
#         )
#         p.add_layout(legend)

# methods_data = load_methods_data(file_paths)

# # 绘制三个图：换挡次数、逃逸航向和转角避障距离（仅第一个图显示 legend）
# p1 = create_figure("Parking space length (m)", "Gear shift count", x_interval=0.4, y_range=(0, 11), y_interval=2)
# plot_metric(p1, methods_data, "gear_shift", show_legend=True)

# p2 = create_figure("Parking space length (m)", "Escape heading (deg)", x_interval=0.4, y_range=(-10, 40), y_interval=10)
# plot_metric(p2, methods_data, "escape_heading", show_legend=False)


# p3 = create_figure("Parking space length (m)", "Distance (m)", x_interval=0.4, y_range=(0, 1.2), y_interval=0.3)
# box = BoxAnnotation(left=5.8, right=7.4, bottom=0.28, top=0.4, fill_color="lightgrey", fill_alpha=0.3, level="underlay")# 让着色矩形在数据图形之下
# p3.add_layout(box)

# label = Label(x=7.18, y=0.08, text="Proper region",
#               text_font="Times New Roman", text_font_style="normal",
#               text_font_size="28pt", text_color="black",
#               text_align="center")
# p3.add_layout(label)

# arrow = Arrow(end=NormalHead(fill_color="black", line_color="black", size=8),
#               x_start=6.97, y_start=0.14,  # 箭头起点
#               x_end=6.9, y_end=0.28)       # 箭头终点
# p3.add_layout(arrow)

# plot_metric(p3, methods_data, "oa_dist", show_legend=False)


# # 设置输出后端为 SVG
# for p in (p1, p2, p3):
#     p.output_backend = "svg"

# # 输出到 HTML 文件，便于浏览器查看
# output_file("bokeh_plots.html", title="Proposed Geometric Method Plots")

# # 分别导出为 SVG 文件
# from bokeh.io.export import export_svgs
# export_svgs(p1, filename="gear_shift_cnt.svg")
# export_svgs(p2, filename="escape_heading.svg")
# export_svgs(p3, filename="corner_oa.svg")


# # 将 SVG 转换为 PDF 文件
# cairosvg.svg2pdf(url="gear_shift_cnt.svg", write_to="gear_shift_cnt.pdf")
# cairosvg.svg2pdf(url="escape_heading.svg", write_to="escape_heading.pdf")
# cairosvg.svg2pdf(url="corner_oa.svg", write_to="corner_oa.pdf")

# # 显示三个图表
# show(column(p1, p2, p3))


import math
import sys, json, cairosvg, numpy as np
from bokeh.plotting import figure, show, output_file
from bokeh.layouts import column
from bokeh.models import SingleIntervalTicker, Legend, LegendItem, FixedTicker, BoxAnnotation, NormalHead, HoverTool, Label, Arrow, OpenHead, ColumnDataSource
# 添加搜索路径（根据需要修改）
sys.path.append("..")
sys.path.append("../out/")
sys.path.append("../lib/")
from lib.load_json import load_json, load_json_keys

# 全局样式参数
line_colors  = ["green", "blue", "red", "black"]
line_widths  = [2.0, 1.5, 1.5, 3.0]
line_styles  = ["solid", "dotted", "dashed", "solid"]
markers      = ["square", "triangle", "asterisk", "no_fill_circle"]
legends      = ["Vor", "Vor + PPM", "Hybrid a star", "Proposed"]

font_size    = "28pt"
font_family  = "Times New Roman"
fig_length   = 1200
fig_height   = 500
x_start      = 5.8
x_end        = 7.4

# JSON 文件路径
tra_file_path = "../out/vor.json"
tra_ppm_geo_file_path = "../out/vor_ppm.json"
hybrid_a_star_file_path = "../out/hybrid_a_star.json"
proposed_file_path    = "../out/proposed_geometric_method.json"
file_paths            = [tra_file_path, tra_ppm_geo_file_path, hybrid_a_star_file_path, proposed_file_path]

def set_fig_text(p):
    p.xaxis.axis_label_text_font_size = font_size
    p.yaxis.axis_label_text_font_size = font_size
    p.xaxis.major_label_text_font_size = font_size
    p.yaxis.major_label_text_font_size = font_size

    p.xaxis.axis_label_text_font_style = "normal"
    p.yaxis.axis_label_text_font_style = "normal"
    p.xaxis.major_label_text_font_style = "normal"
    p.yaxis.major_label_text_font_style = "normal"

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

    num_ticks = int(round((x_end - x_start) / x_interval)) + 1
    x_ticks = [round(x_start + i * x_interval, 2) for i in range(num_ticks)]
    p.xaxis.ticker = FixedTicker(ticks=x_ticks)

    p.y_range.start = y_range[0]
    p.y_range.end = y_range[1]

    # 计算 y 轴刻度，保证不超过 y_range[1]
    max_tick = math.floor((y_range[1] - y_range[0]) / y_interval) * y_interval
    num_y_ticks = int((max_tick - y_range[0]) / y_interval) + 1
    y_ticks = [round(y_range[0] + i * y_interval, 2) for i in range(num_y_ticks)]
    p.yaxis.ticker = FixedTicker(ticks=y_ticks)

    set_fig_text(p)
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
        r_marker = p.circle(x, y, size=10, color=color)
    elif marker_type == "no_fill_circle":
        r_marker = p.circle(x, y, size=12, fill_color=None, line_color=color, line_width=lw)
    elif marker_type == "triangle":
        r_marker = p.triangle(x, y, size=10, color=color)
    elif marker_type == "square":
        r_marker = p.square(x, y, size=10, color=color)
    elif marker_type == "cross":
        r_marker = p.cross(x, y, size=12, color=color)
    elif marker_type == "asterisk":
        r_marker = p.asterisk(x, y, size=14, line_color=color, line_width=lw)
    return r_line, r_marker

def plot_metric(p, methods_data, metric_key, show_legend=False):
    """
    在图 p 上为各方法绘制 metric_key 对应的数据，
    当 show_legend 为 True 时将线型与标记合并到 legend 中，
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
            label_standoff=15,  # 线和文字之间的间距
            glyph_width=70,     # 线的水平长度
            glyph_height=28,    # 线的高度
            label_text_baseline="middle"
        )
        p.add_layout(legend)

def add_aggregated_hover_tool(p, methods_data, metric_key):
    """
    为图 p 添加一个 HoverTool，显示整个横坐标范围内的数据，
    即使某条曲线在某些 x 值上没有数据，也会显示其他曲线的值。

    利用所有方法的 x 坐标取并集，并构造聚合数据源。如果某方法没有该 x 的数据，用 None 填充。
    然后计算每个 x 下现有数据的平均值，用于放置透明触发器。
    """
    # 假定方法顺序与 legends 中一致
    method_names = legends  # ["Vor", "Vor + PPM", "Hybrid a star", "Proposed"]

    # 建立每个方法的 x->y 映射（统一用 round(x, 2) 作为 key）
    method_data_dict = {}
    for i, name in enumerate(method_names):
        xs = methods_data[i]["x"]
        ys = methods_data[i][metric_key]
        mapping = {}
        for x_val, y_val in zip(xs, ys):
            key = round(x_val, 2)
            mapping[key] = y_val
        method_data_dict[name] = mapping

    # 取所有方法 x 坐标的并集
    union_keys = set()
    for name in method_names:
        union_keys = union_keys.union(set(method_data_dict[name].keys()))
    union_x = sorted(union_keys)

    # 构造聚合数据源
    aggregated_data = {"x": union_x}
    for name in method_names:
        aggregated_data[name] = [method_data_dict[name].get(x, None) for x in union_x]

    # 计算每个 x 下可用数据的平均值，作为透明触发器的 y 坐标
    aggregated_data["avg_y"] = []
    for x in union_x:
        values = []
        for name in method_names:
            v = method_data_dict[name].get(x, None)
            if v is not None:
                values.append(v)
        if values:
            avg = sum(values) / len(values)
        else:
            avg = 0
        aggregated_data["avg_y"].append(avg)

    source = ColumnDataSource(data=aggregated_data)
    # 绘制一个透明圆点作为 HoverTool 触发器
    renderer = p.circle('x', 'avg_y', source=source, size=20, fill_alpha=0, line_alpha=0)
    hover = HoverTool(renderers=[renderer],
                      mode='vline',
                      tooltips=[
                          ("x", "@x"),
                          ("Vor", "@{Vor}"),
                          ("Vor + PPM", "@{Vor + PPM}"),
                          ("Hybrid a star", "@{Hybrid a star}"),
                          ("Proposed", "@{Proposed}")
                      ])
    p.add_tools(hover)

# 加载数据
methods_data = load_methods_data(file_paths)

# 绘制三个图：换挡次数、逃逸航向和转角避障距离（第一个图显示图例）
p1 = create_figure("Parking space length (m)", "Gear shift count", x_interval=0.4, y_range=(0, 11), y_interval=2)
plot_metric(p1, methods_data, "gear_shift", show_legend=True)
add_aggregated_hover_tool(p1, methods_data, "gear_shift")

p2 = create_figure("Parking space length (m)", "Escape heading (deg)", x_interval=0.4, y_range=(-10, 40), y_interval=10)
plot_metric(p2, methods_data, "escape_heading", show_legend=False)
add_aggregated_hover_tool(p2, methods_data, "escape_heading")

p3 = create_figure("Parking space length (m)", "Distance (m)", x_interval=0.4, y_range=(0, 1.2), y_interval=0.3)
box = BoxAnnotation(left=5.8, right=7.4, bottom=0.28, top=0.4, fill_color="lightgrey", fill_alpha=0.3, level="underlay")
p3.add_layout(box)

label = Label(x=7.18, y=0.08, text="Proper region",
              text_font="Times New Roman", text_font_style="normal",
              text_font_size="28pt", text_color="black", text_align="center")
p3.add_layout(label)

arrow = Arrow(end=NormalHead(fill_color="black", line_color="black", size=8),
              x_start=6.97, y_start=0.14, x_end=6.9, y_end=0.28)
p3.add_layout(arrow)
plot_metric(p3, methods_data, "oa_dist", show_legend=False)
add_aggregated_hover_tool(p3, methods_data, "oa_dist")

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
