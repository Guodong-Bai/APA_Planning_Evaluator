import sys, json
import matplotlib.pyplot as plt
import numpy as np

sys.path.append("..")
sys.path.append("../out/")
sys.path.append("../lib/")

from lib.load_json import load_json

# JSON 文件路径
file_path = "../out/proposed_geometric_method.json"

# 读取 JSON 数据
with open(file_path, 'r', encoding='utf-8') as f:
    data = json.load(f)
keys = list(data.keys())
print("size = ", keys)

proposed_shift_cnt_vec = []
proposed_escape_cnt_vec = []
x_vec = []
for i in range(len(keys)):
    x_vec.append(5.8 + i * 0.05)


# 遍历所有键，提取数据
for key in keys:
    proposed_shift_cnt_vec.append(data[key][0]["gear_shift_cnt_slot"])
    proposed_escape_cnt_vec.append(data[key][0]["escape_heading"] * 57.3)  # 转换为角度

print("keys =", keys)

# 绘制库内换挡次数图
plt.figure(figsize=(7, 4))
plt.plot(x_vec, proposed_shift_cnt_vec, label='Proposed', marker='o', linestyle='-', color='red')
plt.xlabel("Slot available length (m)", fontsize=8)
plt.ylabel("Gear shift count in slot", fontsize=8)
plt.xticks(np.arange(5.5, 8, 1), fontsize=8)
plt.yticks(np.arange(0, 41, 10), fontsize=8)
plt.ylim(0, 40)
plt.legend(fontsize=8)
plt.tight_layout()
plt.savefig("gear_shift_cnt.png", format="png", dpi=1500)

# 绘制脱困偏角图
plt.figure(figsize=(7, 4))
plt.plot(x_vec, proposed_escape_cnt_vec, marker='o', linestyle='-', color='red')
plt.xlabel("Slot available length (m)", fontsize=8)
plt.ylabel("Escape Heading (deg)", fontsize=8)
plt.xticks(np.arange(5.5, 8, 0.5), fontsize=8)
plt.yticks(np.arange(0, 10, 2), fontsize=8)
# plt.legend(fontsize=8)
plt.tight_layout()
plt.savefig("escape_heading.png", format="png", dpi=1500)
