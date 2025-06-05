# # import numpy as np
# # import matplotlib.pyplot as plt

# # # -------------------------------------
# # # 1. 假设 data_list 已经读入，每行是 [x, y, theta (rad)]
# # # -------------------------------------
# # # 你自己的数据填到 data_list 里即可
# # data_list = [
# #     [1.0, 2.0, np.deg2rad(30.0)],
# #     [3.5, 1.2, np.deg2rad(120.0)],
# #     [2.0, 4.5, np.deg2rad(270.0)],
# #     [5.0, 3.0, np.deg2rad(60.0)],
# #     [4.2, 5.5, np.deg2rad(315.0)],
# #     # … 其余样本
# # ]

# # data = np.array(data_list)
# # xs = data[:, 0]
# # ys = data[:, 1]
# # thetas = data[:, 2]

# # # -------------------------------------
# # # 2. 计算空间密度 Hexbin
# # # -------------------------------------
# # fig, axes = plt.subplots(1, 2, figsize=(10, 4),
# #                          gridspec_kw={'width_ratios': [1, 1]})

# # ax0 = axes[0]
# # hb = ax0.hexbin(xs, ys, gridsize=60, cmap='Blues', mincnt=1)
# # cb = fig.colorbar(hb, ax=ax0, label='Sample Count')
# # ax0.set_xlabel('X (m)')
# # ax0.set_ylabel('Y (m)')
# # ax0.set_title('(a) Spatial Sampling Density')

# # # -------------------------------------
# # # 3. 抽样少量点，叠加箭头
# # # -------------------------------------
# # # 如果想在 Hexbin 上加箭头，先做稀疏抽样
# # N = 10
# # idx_sample = np.arange(0, len(xs), N)
# # xs_sample = xs[idx_sample]
# # ys_sample = ys[idx_sample]
# # th_sample = thetas[idx_sample]

# # arrow_len = 0.3
# # for xi, yi, thi in zip(xs_sample, ys_sample, th_sample):
# #     dx = arrow_len * np.cos(thi)
# #     dy = arrow_len * np.sin(thi)
# #     ax0.arrow(xi, yi, dx, dy,
# #               head_width=0.1, head_length=0.1,
# #               fc='#006600', ec='#006600', alpha=0.6)

# # # -------------------------------------
# # # 4. 极坐标 Rose Diagram (Orientation)
# # # -------------------------------------
# # ax1 = fig.add_subplot(1, 2, 2, polar=True)
# # bins = 36
# # counts, bin_edges = np.histogram(thetas, bins=bins, range=(0, 2 * np.pi))
# # angles = (bin_edges[:-1] + bin_edges[1:]) / 2

# # bars = ax1.bar(
# #     angles, counts,
# #     width=(2 * np.pi / bins), bottom=0.0,
# #     color='teal', edgecolor='black', alpha=0.7
# # )
# # ax1.set_title('(b) Orientation Distribution')

# # # -------------------------------------
# # # 5. 调整整体布局并保存
# # # -------------------------------------
# # plt.tight_layout()
# # plt.show()  # 仅显示，不保存



# import numpy as np
# import matplotlib.pyplot as plt

# # 示例数据
# data_list = [
#     [1.0, 2.0, np.deg2rad(30.0)],
#     [3.5, 1.2, np.deg2rad(120.0)],
#     [2.0, 4.5, np.deg2rad(270.0)],
#     [5.0, 3.0, np.deg2rad(60.0)],
#     [4.2, 5.5, np.deg2rad(315.0)],
#     # …可继续补充
# ]
# data = np.array(data_list)
# xs = data[:, 0]
# ys = data[:, 1]
# thetas = data[:, 2]

# # 主图
# fig, ax = plt.subplots(figsize=(6, 6))

# # 散点图，按 theta 颜色映射
# sc = ax.scatter(xs, ys, c=thetas, cmap='hsv', s=60, edgecolors='none', alpha=0.8)
# cb = plt.colorbar(sc, ax=ax, label='Heading (rad)')
# cb.set_ticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
# cb.set_ticklabels(['0', '$\\frac{1}{2}\\pi$', '$\\pi$', '$\\frac{3}{2}\\pi$', '$2\\pi$'])

# ax.set_xlabel('X (m)')
# ax.set_ylabel('Y (m)')
# ax.set_title('Sample Points Colored by Orientation (Theta)')
# ax.set_aspect('equal', 'box')
# plt.tight_layout()
# plt.show()







import numpy as np
from bokeh.plotting import figure, show
from bokeh.models import ColorBar, LinearColorMapper, ColumnDataSource, BasicTicker
from bokeh.transform import linear_cmap
from bokeh.palettes import Turbo256  # Turbo256 是线性全彩渐变
import matplotlib.pyplot as plt
import matplotlib

# 示例数据
data_list = [
    [1.0, 2.0, np.deg2rad(-50.0)],
    [1.8, 2.5, np.deg2rad(-30.0)],
    [3.5, 1.2, np.deg2rad(0.0)],
    [2.0, 4.5, np.deg2rad(15.0)],
    [5.0, 3.0, np.deg2rad(33.0)],
    [4.2, 5.5, np.deg2rad(50.0)],
]
data = np.array(data_list)
xs = data[:, 0]
ys = data[:, 1]
thetas = data[:, 2]

theta_min = np.deg2rad(-50)
theta_max = np.deg2rad(50)

mapper = LinearColorMapper(palette=Turbo256, low=theta_min, high=theta_max)

source = ColumnDataSource(data=dict(
    x=xs,
    y=ys,
    theta=thetas,
))

p = figure(width=500, height=500, match_aspect=True,
           title="Sample Points Colored by Orientation (Theta)",
           x_axis_label='X (m)', y_axis_label='Y (m)')

r = p.circle('x', 'y', source=source, size=18, alpha=0.8,
             color=linear_cmap('theta', Turbo256, theta_min, theta_max), line_color=None)

color_bar = ColorBar(
    color_mapper=mapper, location=(0, 0), title="Heading (deg)",
    ticker=BasicTicker(desired_num_ticks=5),
    major_label_overrides={
        theta_min: "-50°",
        0: "0°",
        theta_max: "+50°",
    }
)
p.add_layout(color_bar, 'right')

p.grid.grid_line_alpha = 0.3
show(p)
