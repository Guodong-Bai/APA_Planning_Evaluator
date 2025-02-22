import os
import sys
import json
import subprocess

sys.path.append("..")
sys.path.append("../..")
sys.path.append("../../checker")

from checker.lib.load_json import load_json

# Read configuration file
param = load_json("source_config.json")

output_path = param['output_path']
planning_branch = param['planning']['branch']
planning_commit = param['planning']['commit']
planning_address = param['planning']['address']
planning_repository = param['planning']['repository']
planning_lib_address = param['planning']['lib_address']

print("read planning code:")
print(f"branch: {planning_branch}")
print(f"commit: {planning_commit}")
print(f"address: {planning_address}")

# Clone code
os.makedirs('../code', exist_ok=True)
os.makedirs('../lib', exist_ok=True)
os.chdir('../code')
subprocess.run(['git', 'clone', planning_address])

# Update code
os.chdir(planning_repository)
subprocess.run(['git', 'fetch'])
subprocess.run(['git', 'reset', '--hard'])
subprocess.run(['git', 'clean', '-fd', '--force'])
subprocess.run(['git', 'reset' '--hard', 'origin/', planning_branch])
# subprocess.run(['git', 'checkout', planning_commit])
subprocess.run(['git', 'submodule', 'foreach', 'git', 'reset', '--hard'])
subprocess.run(['git', 'submodule', 'foreach', 'git', 'clean', '--fd', '--force'])
subprocess.run(['git', 'submodule', 'update', '--init', '--recursive'])

# Compile planning library
print("compiling planning library for simulation ...")
subprocess.run(['make', 'pybind_build'])

# Sync Python libs
print("sync python libs ...")
subprocess.run(['cp', planning_lib_address, '../../lib'])

# Go back to the task directory and run the simulator
os.chdir('../../task')