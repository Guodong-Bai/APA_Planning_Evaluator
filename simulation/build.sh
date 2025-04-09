#!/bin/bash
# 如果传入参数为 "clean"，则执行清理操作
if [ "$1" == "clean" ]; then
    echo "Start cleaning!"
    rm -rf code/APA_Planning/build
    rm -rf lib
    echo "Cleaning Done!"
    echo "-------------------------------------------------------------------------"
fi

echo "Building!"
rm -rf __pycache__
# 进入 task 目录并执行 download.py，如目录不存在则报错退出
cd task || { echo "Error: Directory 'task' not found."; exit 1; }
python download.py
cd ..