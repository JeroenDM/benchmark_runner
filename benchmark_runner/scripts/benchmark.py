#!/usr/bin/env python
"""
Run a complete benchmark based on a config file.
In this config file different planning groups are defined.
Each planning group specifies which low-level planners are used,
and the configurations for these low-level planners.

For parameter sweeps, use sweep.py
"""
# python standard library
import json
import sys
import subprocess

# ros related
import rospy
import rospkg


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("One argument required:")
        print("rosrun benchmark_runner benchmark.py <config_file_name>")
        exit()
    else:
        config_file_name = sys.argv[1]

    rospy.init_node("execute_benchmark")
    rospack = rospkg.RosPack()

    config_file_path = rospack.get_path("benchmark_runner") + "/config/"
    path = config_file_path + config_file_name

    # with open(config_file_path + config_file_name) as file:
    #     config = json.load(file)
    subprocess.call(["rosparam", "load", path])

    # print(config)
