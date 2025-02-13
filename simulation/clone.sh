#!/bin/bash

if [ "$1" == "clean" ]; then
    echo "Cleaning!"
    rm -rf code
    rm -rf lib
    rm -rf out
elif [ "$1" == "build" ]; then
    echo "Building!"
    cd task
    python download.py
    cd ..
else
    echo "Usage: ./product.sh {clean|build}"
    echo "  clean  删除某些文件夹"
    echo "  build  运行指定的 Python 文件"
fi