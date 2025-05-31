#!/bin/bash

if [ "$1" == "clean" ]; then
    echo "Start cleaning for checker!"
    rm -rf ../out/initial_pose_obs_slot.json
    find ../../simulation/data/ -maxdepth 1 -type f -name 'front*' -delete
    echo "Cleaning Done!"
    echo "--------------------------"
fi

python generate_initial_pose.py
python generate_planning_result.py

# python plot_planning_result_line_chart.py


