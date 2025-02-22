#!/bin/bash


echo "Start cleaning!"
rm -rf code
rm -rf lib
rm -rf out
echo "Cleaning Done!"
echo "-------------------------------------------------------------------------"
echo "Building!"
cd task
python download.py
cd ..