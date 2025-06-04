#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

def plot_comparison_boxplot(fig, data, ylabel, yrange, num_segments, show_legend=True):
    """
    在给定的 fig 中绘制箱线图，比较三种算法在三个 Case 上的效果，并根据 num_segments
    将 y 轴均分为指定段数。可通过 show_legend 参数控制是否显示图例。

    参数：
    - fig: matplotlib.figure.Figure 对象，用于绘图。
    - data: 长度为 9 的列表，每个元素是一个包含数据的数组或列表：
        - data[0:3] 对应 Case A 的 [Vor+PPM, Proposed, Hybrid A*]
        - data[3:6] 对应 Case B 的 [Vor+PPM, Proposed, Hybrid A*]
        - data[6:9] 对应 Case C 的 [Vor+PPM, Proposed, Hybrid A*]
    - ylabel: 字符串，纵坐标名称。
    - yrange: 二元元组 (ymin, ymax)，指定 y 轴的取值范围。
    - num_segments: 整数，将 [ymin, ymax] 区间均分为 num_segments 段，并设置相应刻度。
    - show_legend: 布尔值，若为 True 则在图中添加图例；若为 False 则不显示图例。
    """
    ax = fig.add_subplot(1, 1, 1)
    word_size = 8

    # 字体与负号正常显示
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['axes.unicode_minus'] = False

    # 三组位置：Case A → 1,2,3；Case B → 5,6,7；Case C → 9,10,11
    positions = [1, 2, 3, 5, 6, 7, 9, 10, 11]
    box = ax.boxplot(
        data,
        positions=positions,
        widths=0.6,
        patch_artist=True,
        whiskerprops=dict(),  # 后面统一设置
        capprops=dict(),      # 后面统一设置
        medianprops=dict(),   # 后面统一设置
        showfliers=False
    )

    # 每组三种算法对应的颜色、线型和线宽
    colors     = ['blue',   'black',  'red'] * 3
    linestyles = [':',      '-',      '--'] * 3
    linewidths = [1.0,      1.5,      1.0] * 3

    # 设置每个箱体边框样式
    for idx, patch in enumerate(box['boxes']):
        patch.set_facecolor('none')
        patch.set_edgecolor(colors[idx])
        patch.set_linestyle(linestyles[idx])
        patch.set_linewidth(linewidths[idx])

    # 设置 whiskers（每根 whisker 都要用对应的颜色、样式、宽度）
    for i in range(9):
        lw = linewidths[i]
        ls = linestyles[i]
        col = colors[i]
        w1, w2 = box['whiskers'][2*i], box['whiskers'][2*i + 1]
        for w in (w1, w2):
            w.set_color(col)
            w.set_linestyle(ls)
            w.set_linewidth(lw)

    # 设置 caps（每根 cap 都要用对应的颜色、样式、宽度）
    for i in range(9):
        lw = linewidths[i]
        ls = linestyles[i]
        col = colors[i]
        c1, c2 = box['caps'][2*i], box['caps'][2*i + 1]
        for c in (c1, c2):
            c.set_color(col)
            c.set_linestyle(ls)
            c.set_linewidth(lw)

    # 设置 median 线
    for idx, med in enumerate(box['medians']):
        med.set_color(colors[idx])
        med.set_linestyle(linestyles[idx])
        med.set_linewidth(linewidths[idx])

    # X 轴分组标签
    ax.set_xticks([2, 6, 10])
    ax.set_xticklabels(['Case A', 'Case B', 'Case C'], fontsize=word_size)

    # 固定 Y 轴范围
    ymin, ymax = yrange
    ax.set_ylim((ymin, ymax))

    # 根据 num_segments 均分刻度
    ticks = np.linspace(ymin, ymax, num_segments + 1)
    ax.set_yticks(ticks)
    ax.set_ylabel(ylabel, fontsize=word_size)

    # 根据 show_legend 决定是否添加图例
    if show_legend:
        legend_handles = [
            Line2D([0], [0], color='blue',   linestyle=':',  linewidth=linewidths[0], label='Vor + PPM'),
            Line2D([0], [0], color='black',  linestyle='-',  linewidth=linewidths[1], label='Proposed'),
            Line2D([0], [0], color='red',    linestyle='--', linewidth=linewidths[2], label='Hybrid A*'),
        ]
        ax.legend(handles=legend_handles, loc='upper right', fontsize=word_size)


def main():
    # 随机生成示例数据：9 组，每组 50 个样本
    np.random.seed(42)
    data = [
        np.random.randn(50) + 0.0,  # Case A, Vor+PPM
        np.random.randn(50) + 0.5,  # Case A, Proposed
        np.random.randn(50) + 1.0,  # Case A, Hybrid A*
        np.random.randn(50) - 0.5,  # Case B, Vor+PPM
        np.random.randn(50) + 0.0,  # Case B, Proposed
        np.random.randn(50) + 0.5,  # Case B, Hybrid A*
        np.random.randn(50) + 1.5,  # Case C, Vor+PPM
        np.random.randn(50) + 2.0,  # Case C, Proposed
        np.random.randn(50) + 2.5   # Case C, Hybrid A*
    ]

    # 创建 figure，尺寸 8.89cm x 5cm（单栏宽度约 8.89cm）
    fig = plt.figure(figsize=(8.89 / 2.54, 5 / 2.54))

    # 设置 Y 轴范围为 (-3,5)，分成 4 段，并选择显示图例
    plot_comparison_boxplot(
        fig,
        data,
        ylabel="Computation time",
        yrange=(-3, 5),
        num_segments=4,
        show_legend=True
    )

    plt.tight_layout()
    # 保存为 SVG 和 PDF 格式
    plt.savefig("comparison_boxplot.svg")
    plt.savefig("comparison_boxplot.pdf")


if __name__ == "__main__":
    main()
