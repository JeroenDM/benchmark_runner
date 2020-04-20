#!/usr/bin/env python
"""
Based on a config file, run parameter sweeps
for a specific base configuration of a planning group.
"""
import sys
import json
import pprint
import copy
import datetime
import git

import rospy
import rospkg
import rosparam
from rospy_message_converter import message_converter

from nexon.io import LogDB
from benchmark_runner.task_solver import solve_task
from benchmark_runner.exceptions import PlanningFailedError


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


def sweep_generator(config):
    """ Generates valid planner configurations based on a parameter
    sweep config file.
    Can then be used like this:
        for config in sweep_generator(sweep_config_dict):
            # do something with config
            print(config)
    """

    base_config = config["base_config"]
    sweep_config = config["sweep_config"]
    ptp_sweep_params = list(sweep_config["ptp_config"].keys())
    cart_sweep_params = list(sweep_config["cart_config"].keys())
    current_config = copy.deepcopy(base_config)

    if len(ptp_sweep_params) == 0 or len(cart_sweep_params) == 0:
        # this is to keep this code simple for now
        raise Exception(
            "Every sweep config must have at least 1 parameter with 1 value"
        )

    for _ in range(sweep_config["num_repeat_all"]):
        for ptp_p in ptp_sweep_params:
            for value in sweep_config["ptp_config"][ptp_p]:
                current_config["ptp_config"][ptp_p] = value
                for cart_p in cart_sweep_params:
                    for value in sweep_config["cart_config"][cart_p]:
                        current_config["cart_config"][cart_p] = value
                        yield current_config


def log_to_db(db_handle, success, plans, config, run_id):
    git_repo = git.Repo(search_parent_directories=True)
    git_sha = git_repo.head.object.hexsha
    db_data = {
        "author": "JeroenDM",
        "log_time": datetime.datetime.utcnow(),
        "run_id": run_id,
        "config": config,
        "success": success,
        "plans": [
            message_converter.convert_ros_message_to_dictionary(plan)
            for plan in plans
        ],
        "git_commit_sha": git_sha
    }

    return db_handle.add_data(db_data)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("One argument required:")
        print("rosrun benchmark_runner run.py <config_file_name>")
        exit()
    else:
        config_file = sys.argv[1]

    rospy.init_node("execute_benchmark_run")
    rospack = rospkg.RosPack()

    # Open a connection to database for logging
    DB = LogDB(collection="screencast_example")

    filepath = rosparam.get_param("/planning_task_path")

    config_file_path = rospack.get_path("benchmark_runner") + "/config/"

    with open(config_file_path + config_file) as file:
        config = json.load(file)

    base_config = config["base_config"]
    print("Base configurations:")
    print("--------------------")
    pprint.pprint(base_config)

    print("Start with planner sweeps")
    print("-------------------------")

    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    counter = 0

    for current_config in sweep_generator(config):
        run_id = timestamp + str(counter)
        counter += 1
        print("=======================")
        print("Run id: {}".format(run_id))
        pprint.pprint(current_config)

        rospy.set_param("/benchmark_run_id", run_id)

        # load parameters to parameter server
        ptp_config = current_config["ptp_config"]
        rospy.set_param("/ptp_config", ptp_config)

        cart_config = current_config["cart_config"]
        rospy.set_param("/cart_config", cart_config)

        try:
            plans = solve_task(filepath, current_config)
            print("PLANNING SUCCES")
            success = True
        except PlanningFailedError:
            plans = []
            print("PLANNING FAILED")
            success = False

        db_id = log_to_db(DB, success, plans, current_config, run_id)
        print("Logged to db with id {}".format(db_id))

    # print("LOGGING ===========================")
    # # print(psi.logs)
    # # log_run_to_db(task, filepath)

    # robot = Robot()
    # execute_plans(robot, plans)
