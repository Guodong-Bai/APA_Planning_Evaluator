## APA_Planning_Evaluator

Performance evaluator for different path planning algorithms of automatic parking assist(APA)

# 1. preparation for your planning module
Package your C++ code in pybind module, and provide the Init and Update function,
write all the information into simulation/task/simulation_task.json

# 2. clone and compile your code
Enter simulation/tast and run download.py to clone your planning code from git and compile it,
or you can copy from your own planning library to simulation/lib,

# 3. construct scenarios
run contruct_scenario.py to construct several scenarios with different longitudinal available distance of slot and different corridor,


# 4. Sample initial poses
Enter checker/task and run generate_initial_pose.py

# 5. plan form every initial pose
run generate_planning_result.py