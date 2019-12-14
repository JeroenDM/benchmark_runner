#!/usr/bin/env python
import json
import math
import rospy
import rospkg
import rosparam

import nexon_msgs.msg

from nexon.robot import Robot
from nexon.io import parse_file

from benchmark_runner.planner_interface import PlannerInterface
from benchmark_runner.task_solver import create_pose_msg


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
    planning_group_name = "group_1"

    rospy.init_node("execute_simple_task")
    rospack = rospkg.RosPack()

    # read and get all config parameter stuff
    filepath = rosparam.get_param("/planning_task_path")
    config_file = "planning_groups.json"
    config_file_path = rospack.get_path("benchmark_runner") + "/config/"
    with open(config_file_path + config_file) as file:
        config = json.load(file)
    group_config = config["groups"][planning_group_name]

    # hardcoded constraint
    # Free rotation around z-axis
    constraint = nexon_msgs.msg.PoseConstraint()
    constraint.relative = True
    constraint.rpy_min = [0, 0, -math.pi]
    constraint.rpy_max = [0, 0, math.pi]

    pi = PlannerInterface(group_config)
    task = parse_file(filepath)

    home_config = task["variables"]["home"]

    # sample valid joint configs for P1
    configs = pi.sample(create_pose_msg(task["variables"]["P1"]), constraint)
    configs = [list(config.positions) for config in configs.joint_poses]

    for config in configs:
        try:
            plan = pi.movej(home_config, config)
            start_config = plan.joint_trajectory.points[-1].positions

            plan2 = pi.movel(start_config, create_pose_msg(
                task["variables"]["P2"]))
            break
        except:
            continue

    print("############ found plan")
    print(plan)

    robot = Robot()
    execute_plans(robot, [plan, plan2])
