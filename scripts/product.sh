#!/bin/bash

root_dir=$(git rev-parse --show-toplevel)
echo "项目根目录：$root_dir"

checker_dir="$root_dir/checker"

if [ ! -d "$checker_dir" ]; then
    echo "Error: 目录 '$checker_dir' 不存在。"
    exit 1
fi

if [ "$1" = "clean" ]; then
    echo "-- Start cleaning for checker!"
    rm -rf "$checker_dir/out/initial_pose_obs_slot.json"
    find "$root_dir/simulation/data/" -maxdepth 1 -type f -name 'front*' -delete
    echo -e "Cleaning Done!\n\n"
fi

# 进入 checker 目录再执行 Python 脚本
cd "$checker_dir/task" || { echo "Error: 无法进入目录 "$checker_dir/task""; exit 1; }

python generate_initial_pose.py
python generate_planning_result.py
python evaluation.py
