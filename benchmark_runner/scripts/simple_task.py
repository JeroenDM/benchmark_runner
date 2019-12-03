#!/usr/bin/env python
"""
Basic planning execution.
This file loads the "simple_line.txt" planning tasks,
and then executes it.
"""
import sys
import copy
import json
import git
import rospy
import rospkg
import rosparam
import moveit_msgs.msg
import geometry_msgs.msg
import datetime
import functools

from nexon.io import parse_file, LogDB
from nexon.robot import Robot
from nexon.interface import Commands, Sections
from nexon_msgs.srv import PTPPlanning, PTPPlanningRequest
from nexon_msgs.srv import LINPlanning, LINPlanningRequest
from nexon.util import Plotter


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


def points_to_plan(points):
    plan = moveit_msgs.msg.RobotTrajectory()
    plan.joint_trajectory.points = points
    for pt in plan.joint_trajectory.points:
        pt.velocities = []
        pt.accelerations = []
    plan.joint_trajectory.header.frame_id = "world"
    plan.joint_trajectory.joint_names = [
        "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
    return plan


def log_motion_command(func):
    """ Decorator to log movel, movep and movej commands to database. """

    @functools.wraps(func)  # make debugging easier
    def wrapper(self, start_config, pose_goal):
        self.logs.append("Logging motion command")
        return func(self, start_config, pose_goal)

    return wrapper


class LINServices:
    timeout = 5.0

    def __init__(self, service_names):
        self.names = service_names
        self.s = [self._create_proxy(name) for name in self.names]

    def _create_proxy(self, service_name):
        rospy.wait_for_service(service_name, timeout=self.__class__.timeout)
        return rospy.ServiceProxy(service_name, LINPlanning)


class PlanningServersInterface:
    """
    Call the appropriate service depending on the planning commands.
    """
    DEFAULT_PLANNER = "RRTConnect"
    # PTP_SERVICE_NAME = "ompl_ptp_planning"
    # LIN_SERVICE_NAME = "ompl_lin_planning"
    # LIN_SERVICE_NAME = "desc_lin_planning"

    # def __init__(self, config_file_path):
    #     try:
    #         with open(config_file_path) as file:
    #             config = json.load(file)
    #             self.PTP_SERVICE_NAME = config["ptp_services"][0]
    #             self.LIN_SERVICE_NAME = config["cart_services"][0]
    #             # self.cart_servers = LINServices(config["cart_services"])
    #             rospy.loginfo("Read configuration file:")
    #             rospy.loginfo(str(config))
    #     except:
    #         rospy.loginfo("No config file found, using default values")
    #         rospy.loginfo("Supplied path was: {}".format(config_file_path))

    #     rospy.wait_for_service(self.PTP_SERVICE_NAME, timeout=5.0)
    #     self.ptp = rospy.ServiceProxy(self.PTP_SERVICE_NAME, PTPPlanning)
    #     rospy.wait_for_service(self.LIN_SERVICE_NAME, timeout=5.0)
    #     self.lin = rospy.ServiceProxy(self.LIN_SERVICE_NAME, LINPlanning)
    #     self.logs = []

    def __init__(self, config):
        rospy.wait_for_service(config["ptp_service"], timeout=5.0)
        self.ptp = rospy.ServiceProxy(config["ptp_service"], PTPPlanning)
        rospy.wait_for_service(config["cart_service"], timeout=5.0)
        self.lin = rospy.ServiceProxy(config["cart_service"], LINPlanning)
        self.logs = []

    @log_motion_command
    def movej(self, start_config, joint_values):
        req = PTPPlanningRequest()
        req.joint_goal = joint_values
        req.planner = self.DEFAULT_PLANNER
        req.start_config = start_config

        resp = self.ptp(req)
        if not resp.success:
            raise Exception("Movej command failed.")

        return points_to_plan(resp.joint_path)

    @log_motion_command
    def movep(self, start_config, pose_goal):
        req = PTPPlanningRequest()
        req.pose_goal = pose_goal
        req.planner = self.DEFAULT_PLANNER
        req.start_config = start_config

        resp = self.ptp(req)
        if not resp.success:
            raise Exception("Movej command failed.")

        return points_to_plan(resp.joint_path)

    @log_motion_command
    def movel(self, start_config, pose_goal):
        req = LINPlanningRequest()
        req.start_config = start_config
        req.pose_goal = pose_goal
        req.has_constraints = False

        resp = self.lin(req)
        # resp = self.cart_servers.s[0](req)

        if not resp.success:
            raise Exception("Movel command failed.")

        return points_to_plan(resp.joint_path)


def show_task(plotter, task):
    variables = task[Sections.VARS]
    for v in variables:
        # print(v, variables[v])
        try:
            plotter.plot_axis(create_pose_msg(variables[v]))
        except:
            continue


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
            plan = psi.movel(
                start_config, create_pose_msg(var[command["goal"]]))
            plans.append(plan)

        else:
            raise Exception(
                "Unkown command type: {}".format(ctype))

    return plans


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


def log_run_to_db(task, filepath):
    # git the git commit hash
    repo = git.Repo(search_parent_directories=True)

    db_data = {
        "author": "JeroenDM",
        "date": datetime.datetime.utcnow(),
        "filename": filepath,
        "git": {
            "branch": repo.active_branch.name,
            "sha": repo.head.object.hexsha
        }
    }

    DB.add_data(db_data)


if __name__ == "__main__":

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

    plotter = Plotter(ref_frame="/world")
    show_task(plotter, task)

    config_file = "benchmark_config.json"
    config_file_path = rospack.get_path("benchmark_runner") + "/data/" + config_file

    with open(config_file_path) as file:
        config = json.load(file)

    group_config = config["groups"][1]
    print("Using planning group: {}".format(group_config["name"]))
    psi = PlanningServersInterface(group_config)

    # psi = PlanningServersInterface(config_file_path)

    plans = plan_task(psi, task)

    print("LOGGING ===========================")
    # print(psi.logs)
    # log_run_to_db(task, filepath)

    robot = Robot()
    execute_plans(robot, plans)
