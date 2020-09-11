# Benchmark runners

This package contains scripts to execute a benchmark run for a specific task an planning group.


## Executing a single task

First launch the planning context in ROS using the MoveIt configuration, for example executing tasks with `setup_1`:
```bash
roslaunch setup_1_moveit_config demo.launch
```
This should start rviz and visualize the robot with the tool. In addition there could be a table, a rail, a cage or other robot setup things.

Now load the task and the corresponding work object:
```bash
rosrun benchmark_runner publish_task.py setup_1 l_profile
```
The `publish_task.py` script takes two arguments, the name of the setup and the name of the task you want to execute. Now you should see a (green) object added in front of the robot in rviz. In addition, the poses defined in the task file are visualized.

You can list the aviablable tasks in a setup.
```bash
rosrun benchmark_runner load.py setup_1 -l
```

Before we can execute the task, we need to launch the [planning servers](https://github.com/JeroenDM/benchmark_planning_servers) that solve the subproblems. (In the future this could be added to the above script maybe.)
For example start all ompl planning servers:
```bash
rosrun ompl_planning_server server
```

Finally we also need to start the MongoDB server to log the results.
```bash
sudo systemctl start mongod
```

Now you can execute the task using a specific group of planners
```bash
rosrun benchmark_runner run.py group_1
```
You should see some output in the terminal from parsing and solving the task.
In the end the complete path is executed on the robot in rviz.

**notes**
The convention in ROS is to drop the `.py` extension on the files used to launch nodes. But I like to have python scripts with the `.py` extension.
Possible I will drop this in the future.
