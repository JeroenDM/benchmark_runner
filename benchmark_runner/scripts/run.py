#!/usr/bin/env python
"""
This script runs a task.
It takes as a command line argument the planner group it has to use.
The task is read from the parameter server for now
(parameter: '/planning_task_path').
"""
import sys
import json

import rospy
import rospkg
import rosparam

from nexon.robot import Robot
from benchmark_runner.task_solver import solve_task


def execute_plans(robot, plans):
    """
    Execute a list of plans, this list is returned when solving a task.
    """
    # make sure the robot is actually in the home position
    # before executing a plan
    robot.mg.set_joint_value_target(
        plans[0].joint_trajectory.points[0].positions)
    robot.mg.go(wait=True)
    print("Moved to home, start executing task.")

    # TODO quick fix, add first point to lin path
    plans[1].joint_trajectory.points.insert(
        0, plans[0].joint_trajectory.points[-1])

    for plan in plans:
        print("========================================")
        print("executing plan of lenght")
        print(len(plan.joint_trajectory.points))
        print(plan.joint_trajectory.points[0])
        print(plan.joint_trajectory.points[1])
        print("\n...\n")
        print(plan.joint_trajectory.points[-1])
        print("========================================")
        # print(plan)
        robot.mg.execute(plan, wait=True)
        rospy.sleep(1.0)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("One argument required:")
        print("rosrun benchmark_runner run.py <planning_group_name>")
        exit()
    else:
        planning_group_name = sys.argv[1]

    rospy.init_node("execute_simple_task")
    rospack = rospkg.RosPack()

    # Open a connection to database for logging
    # DB = LogDB()

    filepath = rosparam.get_param("/planning_task_path")

    config_file = "planning_groups.json"
    config_file_path = rospack.get_path("benchmark_runner") + "/config/"

    with open(config_file_path + config_file) as file:
        config = json.load(file)

    group_config = config["groups"][planning_group_name]
    print("Using planning group: {}".format(planning_group_name))

    # load parameters to parameter server
    ptp_config = config["groups"][planning_group_name]["ptp_config"]
    print(ptp_config)
    rospy.set_param("/ptp_config", ptp_config)

    cart_config = config["groups"][planning_group_name]["cart_config"]
    print(cart_config)
    rospy.set_param("/cart_config", cart_config)

    plans = solve_task(filepath, group_config)

    # print("LOGGING ===========================")
    # # print(psi.logs)
    # # log_run_to_db(task, filepath)

    # robot = Robot()
    # execute_plans(robot, plans)
