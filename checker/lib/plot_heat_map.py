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
