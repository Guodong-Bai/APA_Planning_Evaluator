#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_and_save_boxplots_bokeh_combined.py

使用 Bokeh 生成六个字段的箱线图，每个字段用不同颜色，箱体透明，仅绘制边线，
并将这六个图合并到同一个 HTML 文件中。

生成的 HTML 文件：
    - all_boxplots.html

用法：
    python analyze_and_save_boxplots_bokeh_combined.py [file1.json file2.json ...]
如果未提供文件名，则使用默认的 ../out/proposed_geometric_method.json。
"""

import json
import os
import sys
import numpy as np
from collections import Counter
from bokeh.plotting import figure, output_file, save
from bokeh.layouts import gridplot

def load_records(filenames):
    """从给定文件列表加载所有记录，返回嵌套列表：每个文件对应一个记录列表。"""
    all_runs = []
    for fname in filenames:
        with open(fname, "r", encoding="utf-8") as f:
            data = json.load(f)
        records = []
        for rec_list in data.values():
            records.extend(rec_list)
        all_runs.append(records)
    return all_runs

def compute_success_ratios(all_runs, filenames):
    """打印每个文件中 success==True 的比例。"""
    for fname, records in zip(filenames, all_runs):
        total = len(records)
        succ = sum(1 for r in records if r.get("success", False))
        ratio = succ / total if total else 0
        print(f"{os.path.basename(fname)}: {succ}/{total} = {ratio:.2%}")

def collect_metrics(all_runs):
    """
    收集成功记录的指标，返回六个列表，分别对应：
        lengths, times,
        total_gear_shift_cnts, escape_headings,
        gear_shift_cnt_slots, corner_obstacle_avoidance_dists
    """
    lengths = []
    times = []
    total_gear_shift_cnts = []
    escape_headings = []
    gear_shift_cnt_slots = []
    corner_obstacle_avoidance_dists = []

    for records in all_runs:
        for r in records:
            if not r.get("success", False):
                continue
            lengths.append(r.get("path_length", np.nan))
            times.append(r.get("computation_time", np.nan))
            total_gear_shift_cnts.append(r.get("total_gear_shift_cnt", np.nan))
            escape_headings.append(r.get("escape_heading", np.nan))
            gear_shift_cnt_slots.append(r.get("gear_shift_cnt_slot", np.nan))
            corner_obstacle_avoidance_dists.append(r.get("corner_obstacle_avoidance_dist", np.nan))

    # 打印 total_gear_shift_cnt 分布（可选，供调试）
    valid_counts = [int(v) for v in total_gear_shift_cnts if not np.isnan(v)]
    print(">> total_gear_shift_cnt 分布 (Counter):")
    print(Counter(valid_counts))
    if total_gear_shift_cnts:
        print("type = ", type(total_gear_shift_cnts[0]))

    return (
        lengths,
        times,
        total_gear_shift_cnts,
        escape_headings,
        gear_shift_cnt_slots,
        corner_obstacle_avoidance_dists,
    )

def box_stats(vals):
    """
    计算四分位数和须范围以保留在箱线图中：
    - 如果 IQR == 0，则 lower = min(vals), upper = max(vals)
    - 否则 lower = max(min(vals), q1 - 1.5*IQR), upper = min(max(vals), q3 + 1.5*IQR)
    返回 dict(q1,q2,q3,lower,upper)。
    """
    arr = np.array(vals, dtype=float)
    cleaned = arr[~np.isnan(arr)]
    if len(cleaned) == 0:
        return dict(q1=np.nan, q2=np.nan, q3=np.nan, lower=np.nan, upper=np.nan)

    q1, q2, q3 = np.percentile(cleaned, [25, 50, 75])
    iqr = q3 - q1
    if iqr == 0:
        lower = float(np.min(cleaned))
        upper = float(np.max(cleaned))
    else:
        lower = max(float(np.min(cleaned)), q1 - 1.5 * iqr)
        upper = min(float(np.max(cleaned)), q3 + 1.5 * iqr)
    return dict(q1=q1, q2=q2, q3=q3, lower=lower, upper=upper)

def create_boxplot_bokeh(values, title, y_tick_count=5, color="#1f77b4"):
    """
    使用 Bokeh 创建单个字段的箱线图，并返回对应的 Figure 对象。
    - values: 数值列表
    - title: 图表标题（用于 x 轴标签显示）
    - y_tick_count: y 轴主刻度数量（等间隔），默认 5
    - color: 箱体和须的边线颜色
    """
    arr = np.array(values, dtype=float)
    cleaned = arr[~np.isnan(arr)]
    if len(cleaned) == 0:
        # 如果没有有效数据，返回一个空白 Figure
        p_empty = figure(title=title,
                         tools="",
                         toolbar_location=None,
                         plot_width=300,
                         plot_height=500,
                         background_fill_color="#fafafa")
        p_empty.text(x=0.5, y=0.5, text=["No data"], text_align="center", text_baseline="middle")
        return p_empty

    # 计算五数摘要
    stats = box_stats(values)
    q1, q2, q3 = stats["q1"], stats["q2"], stats["q3"]
    lower, upper = stats["lower"], stats["upper"]

    # 创建画布：只需要一个数值型 x 轴
    p = figure(title=title,
               tools="",
               toolbar_location=None,
               plot_width=300,
               plot_height=500,
               background_fill_color="#fafafa")

    # 画箱体（透明填充，仅画边框）
    # x 设为 1。所有图都在 x=1 处叠加
    p.rect(x=1,
           y=(q1 + q3) / 2,
           width=0.4,
           height=(q3 - q1),
           fill_alpha=0.0,
           line_color=color,
           line_width=2)

    # 画中位数横线
    p.segment(x0=0.8, x1=1.2,
              y0=q2, y1=q2,
              line_color=color, line_width=2)

    # 画上须竖线：从 q3 到 upper
    p.segment(x0=1, x1=1,
              y0=q3, y1=upper,
              line_color=color, line_width=2)

    # 画下须竖线：从 lower 到 q1
    p.segment(x0=1, x1=1,
              y0=lower, y1=q1,
              line_color=color, line_width=2)

    # 画须端帽（短横线）
    # 顶部帽
    p.segment(x0=0.9, x1=1.1,
              y0=upper, y1=upper,
              line_color=color, line_width=2)
    # 底部帽
    p.segment(x0=0.9, x1=1.1,
              y0=lower, y1=lower,
              line_color=color, line_width=2)

    # 设置 x 轴（只显示一个刻度 1，标签为 title）
    p.xaxis.ticker = [1]
    p.xaxis.major_label_overrides = {1: title}
    p.xaxis.axis_label = ""

    # 设置 y 轴范围和主刻度
    p.y_range.start = 0
    p.y_range.end = upper * 1.05  # 留 5% 空白
    p.yaxis.ticker = list(np.linspace(0, upper, y_tick_count))

    # 去掉 x 方向网格，仅保留 y 主刻度网格
    p.xgrid.grid_line_color = None
    p.ygrid.grid_line_color = "#DDDDDD"
    p.ygrid.grid_line_dash = "dashed"
    p.ygrid.grid_line_width = 0.5

    return p

def main():
    defaults = ["../out/proposed_geometric_method.json"]
    filenames = sys.argv[1:] or defaults

    # 加载数据并打印成功率
    all_runs = load_records(filenames)
    compute_success_ratios(all_runs, filenames)

    # 收集六个指标
    (
        lengths,
        times,
        total_gear_shift_cnts,
        escape_headings,
        gear_shift_cnt_slots,
        corner_obstacle_avoidance_dists,
    ) = collect_metrics(all_runs)


    # 预定义每个字段对应的颜色
    color_map = {
        "path_length": "#1f77b4",                # 深蓝
        "computation_time": "#ff7f0e",           # 橙
        "total_gear_shift_cnt": "#2ca02c",       # 绿
        "escape_heading": "#d62728",             # 红
        "gear_shift_cnt_slot": "#9467bd",        # 紫
        "corner_obstacle_avoidance_dist": "#8c564b"  # 棕
    }

    # 为每个指标创建一个 Figure
    figs = []
    figs.append(create_boxplot_bokeh(
        lengths,
        "path_length",
        y_tick_count=5,
        color=color_map["path_length"]
    ))
    figs.append(create_boxplot_bokeh(
        times,
        "computation_time",
        y_tick_count=5,
        color=color_map["computation_time"]
    ))
    figs.append(create_boxplot_bokeh(
        total_gear_shift_cnts,
        "total_gear_shift_cnt",
        y_tick_count=5,
        color=color_map["total_gear_shift_cnt"]
    ))
    figs.append(create_boxplot_bokeh(
        escape_headings,
        "escape_heading",
        y_tick_count=5,
        color=color_map["escape_heading"]
    ))
    figs.append(create_boxplot_bokeh(
        gear_shift_cnt_slots,
        "gear_shift_cnt_slot",
        y_tick_count=5,
        color=color_map["gear_shift_cnt_slot"]
    ))
    figs.append(create_boxplot_bokeh(
        corner_obstacle_avoidance_dists,
        "corner_obstacle_avoidance_dist",
        y_tick_count=5,
        color=color_map["corner_obstacle_avoidance_dist"]
    ))

    # 按 2 列 x 3 行 的网格排列所有图（去掉 sizing_mode）
    grid = gridplot([[figs[0], figs[1], figs[2]],
                     [figs[3], figs[4], figs[5]]])

    # 输出到一个 HTML
    output_file("all_boxplots.html")
    save(grid)
    print("已保存: all_boxplots.html")

if __name__ == "__main__":
    main()






