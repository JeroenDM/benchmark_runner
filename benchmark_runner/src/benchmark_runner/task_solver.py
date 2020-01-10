"""
Functions to execute a planning task.
"""
import copy
import geometry_msgs.msg

from nexon.interface import Commands, Sections
from nexon.io import parse_file

from benchmark_runner.planner_interface import PlannerInterface


def solve_task(task_file_path, config):
    task = parse_file(task_file_path)
    pi = PlannerInterface(config)

    return plan_task(pi, task)


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


def plan_task(psi, task):
    # fixed assumption, the robot starts from home
    initial_config = task[Sections.VARS]["home"]

    plans = []
    var = task[Sections.VARS]

    for command in task[Sections.COMMANDS]:
        # what is the inital configuration for the current planning command?
        if len(plans) == 0:
            start_config = copy.copy(initial_config)
        else:
            start_config = copy.copy(
                plans[-1].joint_trajectory.points[-1].positions)

        start_config = list(start_config)
        # start_config.append(0.0)

        ctype = command["type"]
        if ctype == Commands.MOVEJ:
            plan = psi.movej(start_config, var[command["goal"]])
            plans.append(plan)

        elif ctype == Commands.MOVEP:
            plan = psi.movep(
                start_config, create_pose_msg(var[command["goal"]]))
            plans.append(plan)

        elif ctype == Commands.MOVELIN:
            plan = psi.movel(
                start_config, create_pose_msg(var[command["goal"]]))
            plans.append(plan)

        else:
            raise Exception(
                "Unkown command type: {}".format(ctype))

    return plans
