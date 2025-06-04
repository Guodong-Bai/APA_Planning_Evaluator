#!/bin/bash

# 获取 Git 仓库根目录（绝对路径）
root_dir=$(git rev-parse --show-toplevel)
echo "项目根目录：$root_dir"

# 定义 simulation 目录的绝对路径
sim_dir="$root_dir/simulation"

# 检查 simulation 目录是否存在
if [ ! -d "$sim_dir" ]; then
    echo "Error: 目录 '$sim_dir' 不存在。"
    exit 1
fi

# 进入 simulation 目录
cd "$sim_dir"

# 如果传入的第一个参数是 clean，则执行清理操作
if [ "$1" = "clean" ]; then
    echo "-- Start clean build..."
    rm -rf "$sim_dir/code/APA_Planning/build"
    rm -rf "$sim_dir/lib"
    echo -e "-- Cleaning Done!\n"
fi

echo "-- Compiling planning project!"
rm -rf "$sim_dir/__pycache__"

# 定义 task 目录的绝对路径
task_dir="$sim_dir/task"

# 进入 task 目录并执行 download.py，如目录不存在则报错退出
if [ ! -d "$task_dir" ]; then
    echo "Error: 目录 '$task_dir' 不存在。"
    exit 1
fi

cd "$task_dir"
python download.py

cd "$root_dir"
