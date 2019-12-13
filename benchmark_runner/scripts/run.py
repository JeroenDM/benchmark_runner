#!/usr/bin/env python
"""
Basic planning execution.
This file loads the "simple_line.txt" planning tasks,
and then executes it.
"""
import sys
import json
import rospy
import rospkg
import rosparam

from nexon.io import parse_file
from nexon.robot import Robot
from nexon.util import Plotter
from nexon.interface import Sections

from benchmark_runner.planner_interface import PlannerInterface
from benchmark_runner.task_runner import create_pose_msg, plan_task


def show_task(plotter, task):
    variables = task[Sections.VARS]
    for v in variables:
        print(v, variables[v])
        try:
            plotter.plot_axis(create_pose_msg(variables[v]))
        except:
            # TODO bad style, a bare except statement.
            # change task data structure to fix this
            continue


def execute_plans(robot, plans):
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

    # filename = "simple_line.irl"
    # filename = "multiple_lines.irl"
    # filename = "kingpin.irl"
    # filename = "l_profile.irl"
    # filepath = rospack.get_path("benchmark_runner") + "/data/" + filename

    filepath = rosparam.get_param("/planning_task_path")
    task = parse_file(filepath)

    # plotter = Plotter(ref_frame="/world")
    plotter = Plotter(ref_frame="/work")
    show_task(plotter, task)

    config_file = "planning_groups.json"
    config_file_path = rospack.get_path("benchmark_runner") + "/config/"

    with open(config_file_path + config_file) as file:
        config = json.load(file)

    group_config = config["groups"][planning_group_name]
    print("Using planning group: {}".format(planning_group_name))
    psi = PlannerInterface(group_config)

    plans = plan_task(psi, task)

    print("LOGGING ===========================")
    # print(psi.logs)
    # log_run_to_db(task, filepath)

    robot = Robot()
    execute_plans(robot, plans)
