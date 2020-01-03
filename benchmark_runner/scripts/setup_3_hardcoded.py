#!/usr/bin/env python
import json
import math
import rospy
import rospkg
import rosparam
import copy

import nexon_msgs.msg

from nexon.robot import Robot
from nexon.io import parse_file
from nexon.interface import Commands, Sections

from benchmark_runner.planner_interface import PlannerInterface
from benchmark_runner.task_solver import create_pose_msg
from benchmark_runner.exceptions import PlanningFailedError


from nexon_msgs.msg import PoseConstraint


def create_constraint_message(con):
    pc = PoseConstraint()
    pc.relative = True
    if con["type"] != "rpy":
        raise NotImplementedError("Only rpy constraints implemented for now.")
    pc.rpy_min = con["min"]
    pc.rpy_max = con["max"]
    return pc


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


def movep_sampling(pi, start_config, goal, con):
    # print(start_config)
    # print(goal)
    # print(con)
    samples = pi.sample(goal, con)
    configs = [list(q.positions) for q in samples.joint_poses]
    for q in configs:
        try:
            plan = pi.movep(start_config, goal)
        except PlanningFailedError as e:
            print(e)
            continue
        return plan

    raise PlanningFailedError("Sampling movep failed.")


def plan_task(psi, task):
    # fixed assumption, the robot starts from home
    initial_config = task[Sections.VARS]["home"]

    plans = []
    var = task[Sections.VARS]
    con = task["constraints"]

    for command in task[Sections.COMMANDS]:
        # what is the inital configuration for the current planning command?
        if len(plans) == 0:
            start_config = initial_config
        else:
            start_config = plans[-1].joint_trajectory.points[-1].positions

        ctype = command["type"]
        print(command)
        if ctype == Commands.MOVEJ:
            plan = psi.movej(start_config, var[command["goal"]])
            plans.append(plan)

        elif ctype == Commands.MOVEP:
            plan = movep_sampling(
                psi,
                start_config,
                create_pose_msg(var[command["goal"]]),
                create_constraint_message(con[command["constraints"][0]])
            )
            # plan = psi.movep(
            #     start_config, create_pose_msg(var[command["goal"]]))
            plans.append(plan)

        elif ctype == Commands.MOVELIN:
            plan = psi.movel(
                start_config, create_pose_msg(var[command["goal"]]))
            plans.append(plan)

        else:
            raise Exception(
                "Unkown command type: {}".format(ctype))

    return plans


def print_task(task):
    for key in task:
        for v in task[key]:
            try:
                print(v, task[key][v])
            except TypeError:
                print(v)


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

    plans = plan_task(pi, task)
    robot = Robot()
    execute_plans(robot, plans)

    # home_config = task["variables"]["home"]

    # sample valid joint configs for P1
    # samples = pi.sample(create_pose_msg(task["variables"]["P1"]), constraint)
    # configs = [list(q.positions) for q in samples.joint_poses]

    # WARNING ugly code ahead
    # found = False
    # for config in configs:
    #     try:
    #         plan1 = pi.movej(home_config, config)
    #     except PlanningFailedError as e:
    #         print(e)
    #         print("#### PTP 1: Continue with for loop")
    #         continue

    #     start_config = copy.deepcopy(
    #         plan1.joint_trajectory.points[-1].positions)

    #     try:
    #         plan2 = pi.movel(start_config, create_pose_msg(
    #             task["variables"]["P2"]))
    #     except PlanningFailedError as e:
    #         print(e)
    #         print("#### LIN: Continue with for loop")
    #         continue

    #     start_config_2 = copy.deepcopy(
    #         plan2.joint_trajectory.points[-1].positions)

    #     try:
    #         plan3 = pi.movej(start_config_2, home_config)
    #     except PlanningFailedError as e:
    #         print(e)
    #         print("#### PTP 2: Continue with for loop")
    #         continue

    #     found = True
    #     print("### Plan found")
    #     break

    # if found:
    #     print("############ found plans, executing the plans")
    #     robot = Robot()
    #     execute_plans(robot, [plan1, plan2, plan3])
    # else:
    #     print("Failed to find plan.")
