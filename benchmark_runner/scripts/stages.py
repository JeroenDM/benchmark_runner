#!/usr/bin/env python
"""
This script runs a task.
It takes as a command line argument the planner group it has to use.
The task is read from the parameter server for now
(parameter: '/planning_task_path').
"""
import numpy as np
import tf
import sys
import json
import copy

import rospy
import rospkg
import rosparam
import geometry_msgs.msg

from nexon.robot import Robot
from nexon.interface import Commands, Sections
from nexon.io import parse_file

from benchmark_runner.planner_interface import PlannerInterface


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


def create_pose_msg(goal):
    xyz, xyzw = goal["xyz"], goal["xyzw"]
    p = geometry_msgs.msg.Pose()
    p.position.x = xyz[0]
    p.position.y = xyz[1]
    p.position.z = xyz[2]
    p.orientation.x = xyzw[0]
    p.orientation.y = xyzw[1]
    p.orientation.z = xyzw[2]
    p.orientation.w = xyzw[3]
    return p

# rotate vector v1 by quaternion q1


def calc_retract_path(pi, pose, retract_vector, start_config):
    print("Planning retract motion.")
    pr = copy.deepcopy(pose)
    # pr.position.x += retract_vector[0]
    # pr.position.y += retract_vector[1]
    # pr.position.z += retract_vector[2]

    v = np.array(retract_vector)
    R = tf.transformations.quaternion_matrix(
        [pr.orientation.x, pr.orientation.y, pr.orientation.z, pr.orientation.w])
    vn = np.dot(R[:3, :3], v)

    pr.position.x += vn[0]
    pr.position.y += vn[1]
    pr.position.z += vn[2]

    plan = pi.movel(start_config, pr)
    return plan


def calc_approach_path(pi, pose, approach_vector, start_config):
    pass


def plan_task(psi, task):
    # fixed assumption, the robot starts from home
    initial_config = task[Sections.VARS]["home"]

    plans = []
    var = task[Sections.VARS]

    for command in task[Sections.COMMANDS]:
        # what is the inital configuration for the current planning command?
        if len(plans) == 0:
            start_config = initial_config
        else:
            start_config = plans[-1].joint_trajectory.points[-1].positions

        ctype = command["type"]
        if ctype == Commands.MOVEJ:
            plan = psi.movej(start_config, var[command["goal"]])
            plans.append(plan)

        elif ctype == Commands.MOVEP:
            plan = psi.movep(
                start_config, create_pose_msg(var[command["goal"]]))
            plans.append(plan)

        elif ctype == Commands.MOVELIN:
            goal_pose_msg = create_pose_msg(var[command["goal"]])
            plan = psi.movel(start_config, goal_pose_msg)
            plans.append(plan)

            # add retract motion
            end_config = plan.joint_trajectory.points[-1].positions
            r_plan = calc_retract_path(
                psi, goal_pose_msg, [0, 0, -0.1], end_config)
            plans.append(r_plan)

        else:
            raise Exception(
                "Unkown command type: {}".format(ctype))

    return plans


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

    task_file_path = rosparam.get_param("/planning_task_path")

    config_file = "planning_groups.json"
    config_file_path = rospack.get_path("benchmark_runner") + "/config/"

    with open(config_file_path + config_file) as file:
        config = json.load(file)

    group_config = config["groups"][planning_group_name]
    print("Using planning group: {}".format(planning_group_name))

    task = parse_file(task_file_path)
    pi = PlannerInterface(group_config)

    plans = plan_task(pi, task)

    print("LOGGING ===========================")
    # print(psi.logs)
    # log_run_to_db(task, filepath)

    robot = Robot()
    execute_plans(robot, plans)
