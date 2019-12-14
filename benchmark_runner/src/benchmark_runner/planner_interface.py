import rospy
import moveit_msgs.msg

from nexon_msgs.srv import PTPPlanning, PTPPlanningRequest
from nexon_msgs.srv import LINPlanning, LINPlanningRequest

from benchmark_runner.logging import log_motion_command

"""
Class that defines interface to execute low-level planning request
through the ROS-services that the low-level planners publish.

Different low-level planning services are:
- ptp_service: point-to-point planning (joint or pose goal)
- lin_service: linear Cartisian motion to pose goal
"""


def points_to_plan(points):
    """ Convert a list of robot configurations to a MoveIt plan.

    MoveIt plan is a RobotTrajectory message.
    """
    plan = moveit_msgs.msg.RobotTrajectory()
    plan.joint_trajectory.points = points
    for pt in plan.joint_trajectory.points:
        pt.velocities = []
        pt.accelerations = []
    plan.joint_trajectory.header.frame_id = "world"
    plan.joint_trajectory.joint_names = [
        "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
    return plan


class PlannerInterface:
    """
    Call the appropriate service depending on the planning commands.
    """

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
        req.start_config = start_config

        resp = self.ptp(req)
        if not resp.success:
            raise Exception("Movej command failed.")

        return points_to_plan(resp.joint_path)

    @log_motion_command
    def movep(self, start_config, pose_goal):
        req = PTPPlanningRequest()
        req.pose_goal = pose_goal
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