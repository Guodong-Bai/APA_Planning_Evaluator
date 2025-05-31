# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# analyze_with_bokeh_fixed_iqr.py

# 对原脚本做如下修改：
# - 在 box_stats 中，如果 IQR==0，则强制把 lower/min, upper/max。
# - 其余逻辑保持不变。

# 用法：
#     python analyze_with_bokeh_fixed_iqr.py [file1.json file2.json ...]
# """

# import json
# import os
# import sys
# import numpy as np
# from collections import Counter
# from bokeh.plotting import figure, show
# from bokeh.layouts import gridplot
# from bokeh.models import Whisker, ColumnDataSource, Range1d


# def load_records(filenames):
#     """从给定文件列表加载所有记录，返回嵌套列表：每个文件对应一个记录列表。"""
#     all_runs = []
#     for fname in filenames:
#         with open(fname, "r", encoding="utf-8") as f:
#             data = json.load(f)
#         records = []
#         for rec_list in data.values():
#             records.extend(rec_list)
#         all_runs.append(records)
#     return all_runs


# def compute_success_ratios(all_runs, filenames):
#     """打印每个文件中 success==True 的比例。"""
#     for fname, records in zip(filenames, all_runs):
#         total = len(records)
#         succ = sum(1 for r in records if r.get("success", False))
#         ratio = succ / total if total else 0
#         print(f"{os.path.basename(fname)}: {succ}/{total} = {ratio:.2%}")


# def collect_metrics(all_runs):
#     """
#     收集成功记录的指标，返回：
#         lengths, times,
#         total_gear_shift_cnts, escape_headings,
#         gear_shift_cnt_slots, corner_obstacle_avoidance_dists
#     """
#     lengths = []
#     times = []
#     total_gear_shift_cnts = []
#     escape_headings = []
#     gear_shift_cnt_slots = []
#     corner_obstacle_avoidance_dists = []

#     for records in all_runs:
#         for r in records:
#             if not r.get("success", False):
#                 continue
#             lengths.append(r.get("path_length", np.nan))
#             times.append(r.get("computation_time", np.nan))
#             total_gear_shift_cnts.append(r.get("total_gear_shift_cnt", np.nan))
#             escape_headings.append(r.get("escape_heading", np.nan))
#             gear_shift_cnt_slots.append(r.get("gear_shift_cnt_slot", np.nan))
#             corner_obstacle_avoidance_dists.append(r.get("corner_obstacle_avoidance_dist", np.nan))

#     # 打印 total_gear_shift_cnt 分布，验证无误
#     valid_counts = [int(v) for v in total_gear_shift_cnts if not np.isnan(v)]
#     print(">> 收集到的 total_gear_shift_cnt 列表 (前 50 个):")
#     print(valid_counts[:50], "... (共 %d 条)" % len(valid_counts))
#     print(">> 值分布 (Counter):")
#     print(Counter(valid_counts))

#     return (
#         lengths,
#         times,
#         total_gear_shift_cnts,
#         escape_headings,
#         gear_shift_cnt_slots,
#         corner_obstacle_avoidance_dists,
#     )


# def box_stats(vals):
#     """
#     计算四分位数和须范围：
#     - 如果 IQR == 0（即 q1 == q3），则强制把 lower=min(vals), upper=max(vals)；
#     - 否则按常规公式 lower = max(min(vals), q1 - 1.5*IQR)， upper = min(max(vals), q3 + 1.5*IQR)。
#     最终返回 dict(q1,q2,q3,lower,upper)。
#     """
#     arr = np.array(vals)
#     cleaned = arr[~np.isnan(arr)]
#     if len(cleaned) == 0:
#         # 全部为 NaN 的情况下，直接返回全 Nan
#         return dict(q1=np.nan, q2=np.nan, q3=np.nan, lower=np.nan, upper=np.nan)

#     # 先计算 q1, q2, q3
#     q1, q2, q3 = np.percentile(cleaned, [25, 50, 75])
#     iqr = q3 - q1

#     if iqr == 0:
#         # 如果 IQR==0，说明 q1 == q2 == q3，都为同一个常数，此时我们把须拉到 min/ max
#         lower = float(np.min(cleaned))
#         upper = float(np.max(cleaned))
#     else:
#         # 常规计算
#         lower = max(float(np.min(cleaned)), q1 - 1.5 * iqr)
#         upper = min(float(np.max(cleaned)), q3 + 1.5 * iqr)

#     return dict(q1=float(q1), q2=float(q2), q3=float(q3), lower=lower, upper=upper)


# def make_box_figure(stats, label, is_count_field=False):
#     """
#     根据 stats 绘制箱线图。如果 is_count_field=True，则认为该字段为离散计数，
#     强制使用 whisker_pos_fraction=1 以避免进一步压缩须范围；否则使用 0.75。
#     """
#     whisker_frac = 1.0 if is_count_field else 0.75

#     # 如果 upper 是 NaN 或 whisker_frac <= 0，就给一个默认 y_max
#     if np.isnan(stats["upper"]) or whisker_frac <= 0:
#         y_max = 1
#     else:
#         y_max = stats["upper"] / whisker_frac

#     x = [label]
#     source = ColumnDataSource(
#         dict(
#             x=x,
#             q1=[stats["q1"]],
#             q2=[stats["q2"]],
#             q3=[stats["q3"]],
#             lower=[stats["lower"]],
#             upper=[stats["upper"]],
#         )
#     )

#     p = figure(
#         x_range=[label],
#         y_range=Range1d(start=0, end=y_max),
#         width=300,
#         height=400,
#         title=label,
#         tools="save,pan,box_zoom,reset,wheel_zoom",
#     )
#     # 箱体：上半部分
#     p.vbar(x="x", width=0.3, bottom="q2", top="q3", source=source, line_color="black")
#     # 箱体：下半部分（空心）
#     p.vbar(
#         x="x", width=0.3, bottom="q1", top="q2", source=source, line_color="black", fill_color=None
#     )

#     whisker = Whisker(base="x", upper="upper", lower="lower", source=source)
#     whisker.upper_head.size = whisker.lower_head.size = 8
#     p.add_layout(whisker)

#     p.xgrid.grid_line_color = None
#     return p


# def main():
#     defaults = ["../out/proposed_geometric_method.json"]
#     filenames = sys.argv[1:] or defaults

#     # 1. 加载数据
#     all_runs = load_records(filenames)
#     # 2. 打印每个文件的成功率
#     compute_success_ratios(all_runs, filenames)
#     # 3. 收集六个指标并打印 total_gear_shift_cnt 分布
#     (
#         lengths,
#         times,
#         total_gear_shift_cnts,
#         escape_headings,
#         gear_shift_cnt_slots,
#         corner_obstacle_avoidance_dists,
#     ) = collect_metrics(all_runs)

#     # 4. 分别计算 6 个字段的箱线图统计量
#     stats_len = box_stats(lengths)
#     stats_time = box_stats(times)
#     stats_total_gear = box_stats(total_gear_shift_cnts)
#     stats_escape_heading = box_stats(escape_headings)
#     stats_gear_shift_slot = box_stats(gear_shift_cnt_slots)
#     stats_corner_dist = box_stats(corner_obstacle_avoidance_dists)

#     # 5. 生成 6 张箱线图
#     fig1 = make_box_figure(stats_len, "path_length")
#     fig2 = make_box_figure(stats_time, "computation_time")
#     # total_gear_shift_cnt 是离散计数，这里传 is_count_field=True
#     fig3 = make_box_figure(stats_total_gear, "total_gear_shift_cnt", is_count_field=True)
#     fig4 = make_box_figure(stats_escape_heading, "escape_heading")
#     # gear_shift_cnt_slot 也是计数型，同样设为 True
#     fig5 = make_box_figure(stats_gear_shift_slot, "gear_shift_cnt_slot", is_count_field=True)
#     fig6 = make_box_figure(stats_corner_dist, "corner_obstacle_avoidance_dist")

#     # 6. 按 2×3 网格展示
#     grid = gridplot([[fig1, fig2, fig3], [fig4, fig5, fig6]], toolbar_location="right")
#     show(grid)


# if __name__ == "__main__":
#     main()







#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_and_save_boxplots.py

将原有 Bokeh 交互式展示改为使用 matplotlib 生成箱线图，并分别保存为6个独立的 PDF 文件。

生成的 PDF 文件：
    - path_length.pdf
    - computation_time.pdf
    - total_gear_shift_cnt.pdf
    - escape_heading.pdf
    - gear_shift_cnt_slot.pdf
    - corner_obstacle_avoidance_dist.pdf

用法：
    python analyze_and_save_boxplots.py [file1.json file2.json ...]
如果未提供文件名，则使用默认的 ../out/proposed_geometric_method.json。
"""

import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

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
    arr = np.array(vals)
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

def save_boxplot(values, title, filename, y_tick_count=5):
    """
    使用 matplotlib 生成单个字段的箱线图并保存为 PDF。
    - values: 数值列表
    - title: 图表标题
    - filename: 输出 PDF 文件名
    - y_tick_count: y 轴主刻度数量（等间隔），默认 5
    """
    cleaned = np.array(values)[~np.isnan(values)]
    if len(cleaned) == 0:
        print(f"警告: {title} 没有有效数据，跳过绘图。")
        return

    # 计算四分位数和须范围
    stats = box_stats(values)
    lower, q1, q2, q3, upper = stats["lower"], stats["q1"], stats["q2"], stats["q3"], stats["upper"]

    fig, ax = plt.subplots(figsize=(4, 6))
    # 绘制箱线图，使用自定义统计数据
    ax.bxp([{
        'med': q2,
        'q1': q1,
        'q3': q3,
        'whislo': lower,  # 下须
        'whishi': upper,  # 上须
        'fliers': []      # 不绘制离群点
    }], vert=True, showfliers=False)

    ax.set_title(title)
    ax.set_xticks([1])
    ax.set_xticklabels([title])

    # 设置 y 轴为 0 到 upper（或稍微留白），并设置 y_tick_count 个等间隔刻度
    y_max = upper
    ax.set_ylim(0, y_max * 1.05)  # 上方留 5% 空白
    ticks = np.linspace(0, y_max, y_tick_count)
    ax.set_yticks(ticks)
    # 移除次要刻度
    ax.minorticks_off()

    ax.grid(axis='y', which='major', linestyle='--', linewidth=0.5)
    ax.grid(axis='x', which='both', linestyle='')

    plt.tight_layout()
    plt.savefig(filename, format='pdf')
    plt.close(fig)
    print(f"已保存: {filename}")

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

    # 保存各字段的 PDF 箱线图，用户可调整 y_tick_count 参数
    save_boxplot(lengths, "path_length", "path_length.pdf", y_tick_count=5)
    save_boxplot(times, "computation_time", "computation_time.pdf", y_tick_count=5)
    save_boxplot(total_gear_shift_cnts, "total_gear_shift_cnt", "total_gear_shift_cnt.pdf", y_tick_count=5)
    save_boxplot(escape_headings, "escape_heading", "escape_heading.pdf", y_tick_count=5)
    save_boxplot(gear_shift_cnt_slots, "gear_shift_cnt_slot", "gear_shift_cnt_slot.pdf", y_tick_count=5)
    save_boxplot(corner_obstacle_avoidance_dists, "corner_obstacle_avoidance_dist", "corner_obstacle_avoidance_dist.pdf", y_tick_count=5)

if __name__ == "__main__":
    main()
