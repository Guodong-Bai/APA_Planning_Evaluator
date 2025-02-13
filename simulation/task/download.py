import os
import sys
import json
import subprocess

# Read configuration file
with open('source_config.json') as f:
    param = json.load(f)

output_path = param['output_path']
planning_address = param['planning']['address']
planning_branch = param['planning']['branch']
planning_commit = param['planning']['commit']
planning_repository = param['planning']['repository']
planning_lib_address = param['planning']['lib_address']

print("read planning code:")
print(f"address: {planning_address}")
print(f"branch: {planning_branch}")
print(f"commit: {planning_commit}")

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
subprocess.run(['git', 'checkout', planning_commit])
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

try:
    os.mkdir(output_path)
except:
    pass

# subprocess.run(['python', 'apa_closed_loop_simulator.py', 'simulation_task.json'])