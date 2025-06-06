import numpy as np
import matplotlib.pyplot as plt

def grid_sample(initial_pose_vec, grid_step=0.1):
    """
    对 initial_pose_vec 中的位姿 ([x, y, theta]) 做网格化采样：
    每个格子中选取离格子中心最近的点。
    """
    data = np.array(initial_pose_vec)
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

    # 对每个格子，选离格子中心最近的点
    cell_to_idx = dict()
    for idx in range(len(xs)):
        cell = (ix[idx], iy[idx])
        # 格子中心坐标
        center_x = x_edges[cell[0]] + grid_step / 2
        center_y = y_edges[cell[1]] + grid_step / 2
        dist = (xs[idx] - center_x) ** 2 + (ys[idx] - center_y) ** 2
        # 如果该格子没存点，或者当前点更接近格子中心
        if (cell not in cell_to_idx) or (dist < cell_to_idx[cell][1]):
            cell_to_idx[cell] = (idx, dist)

    selected_indices = [v[0] for v in cell_to_idx.values()]

    sampled = []
    for idx in selected_indices:
        sampled.append([float(xs[idx]), float(ys[idx]), float(thetas[idx])])

    return sampled

# ------- 主函数 -------
if __name__ == "__main__":
    # 生成示例数据
    np.random.seed(42)
    N = 500
    xs = np.random.uniform(-2.0, 2.0, size=N)
    ys = np.random.uniform(-1.5, 1.5, size=N)
    thetas = np.random.uniform(0, 2 * np.pi, size=N)
    initial_pose_vec = np.column_stack((xs, ys, thetas)).tolist()

    # 网格化采样（格子中心选点）
    sampled_list = grid_sample(initial_pose_vec, grid_step=0.5)

    sampled = np.array(sampled_list)
    xs_s = sampled[:, 0]
    ys_s = sampled[:, 1]
    thetas_s = sampled[:, 2]

    arrow_len = 0.2
    dx = arrow_len * np.cos(thetas_s)
    dy = arrow_len * np.sin(thetas_s)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(xs, ys, s=10, color='lightgray', label='原始采样点')
    ax.scatter(xs_s, ys_s, s=30, color='green', alpha=0.8, label='均匀栅格采样点')
    ax.quiver(xs_s, ys_s, dx, dy, angles='xy', scale_units='xy', scale=1, width=0.005, color='blue')
    ax.set_title('Grid-sampled (center-near) Poses with Orientations')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(loc='upper right')
    ax.set_aspect('equal', 'box')
    plt.tight_layout()
    plt.savefig("grid_sampled_poses_centered.svg", format="svg")
    plt.show()
