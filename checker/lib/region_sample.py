import numpy as np
import matplotlib.pyplot as plt

def grid_sample(initial_pose_vec, grid_step=0.1):
    """
    对 initial_pose_vec 中的位姿 ([x, y, theta]) 做网格化采样：
    在 x-y 平面上以 grid_step 为步长划网格，然后从每个非空格子中随机选取一个位姿。

    参数：
    - initial_pose_vec: List of [x, y, theta]（theta 为弧度）。
    - grid_step: 网格边长（默认为 0.1）。

    返回：
    - sampled_list: 同样形如 [[x, y, theta], …]，其中每个格子只保留一个位姿。
    """
    data = np.array(initial_pose_vec)  # shape=(N, 3)
    if data.ndim != 2 or data.shape[1] != 3:
        raise ValueError("initial_pose_vec 必须是形如 [[x,y,theta], …] 的列表")
    xs = data[:, 0]
    ys = data[:, 1]
    thetas = data[:, 2]

    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    x_edges = np.arange(x_min, x_max + grid_step, grid_step)
    y_edges = np.arange(y_min, y_max + grid_step, grid_step)

    ix = np.digitize(xs, x_edges) - 1
    iy = np.digitize(ys, y_edges) - 1
    ix = np.clip(ix, 0, len(x_edges) - 2)
    iy = np.clip(iy, 0, len(y_edges) - 2)

    perm = np.random.permutation(len(xs))
    seen = set()
    selected_indices = []
    for idx in perm:
        cell = (ix[idx], iy[idx])
        if cell not in seen:
            seen.add(cell)
            selected_indices.append(idx)

    sampled = []
    for idx in selected_indices:
        sampled.append([float(xs[idx]), float(ys[idx]), float(thetas[idx])])

    return sampled



# 生成示例数据
np.random.seed(42)
N = 500
xs = np.random.uniform(-2.0, 2.0, size=N)
ys = np.random.uniform(-1.5, 1.5, size=N)
thetas = np.random.uniform(0, 2 * np.pi, size=N)
initial_pose_vec = np.column_stack((xs, ys, thetas)).tolist()

# 网格化采样
sampled_list = grid_sample(initial_pose_vec, grid_step=0.5)

# 转换采样结果
sampled = np.array(sampled_list)
xs_s = sampled[:, 0]
ys_s = sampled[:, 1]
thetas_s = sampled[:, 2]

# 计算箭头分量
arrow_len = 0.2
dx = arrow_len * np.cos(thetas_s)
dy = arrow_len * np.sin(thetas_s)

# 绘图并保存为 SVG
fig, ax = plt.subplots(figsize=(6, 6))
ax.scatter(xs, ys, s=10, color='lightgray', label='原始采样点')
ax.scatter(xs_s, ys_s, s=30, color='green', alpha=0.8, label='网格化抽样点')

# 绘制箭头
ax.quiver(xs_s, ys_s, dx, dy, angles='xy', scale_units='xy', scale=1, width=0.005, color='blue')

ax.set_title('Grid-sampled Poses with Orientations')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.legend(loc='upper right')
ax.set_aspect('equal', 'box')
plt.tight_layout()

# 保存为 SVG
plt.savefig("grid_sampled_poses.svg", format="svg")

# 显示图像
plt.show()
