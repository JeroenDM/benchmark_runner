import rospy
import math
import moveit_msgs.msg
import trajectory_msgs.msg
import nexon_msgs

from nexon_msgs.srv import PTPPlanning, PTPPlanningRequest
from nexon_msgs.srv import LINPlanning, LINPlanningRequest, LINPlanningResponse
from nexon_msgs.srv import SampleConstraint, SampleConstraintRequest

from benchmark_runner.logging import log_motion_command
from benchmark_runner.exceptions import PlanningFailedError

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
    if not rospy.has_param("/manipulator/joint_names"):
        raise Exception("The planner interface needs a parameter \
        /manipulator/joint_names on the parameter server.")

    plan.joint_trajectory.joint_names = rospy.get_param(
        "/manipulator/joint_names")
    # plan.joint_trajectory.joint_names = [
    #     "rail_base_to_carrier", "joint_a1", "joint_a2",
    #     "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
    # plan.joint_trajectory.joint_names = [
    #     "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
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
        # rospy.wait_for_service(config["sample_service"], timeout=5.0)
        # self.sample = rospy.ServiceProxy(
        #     config["sample_service"], SampleConstraint)
        self.logs = []

    @log_motion_command
    def movej(self, start_config, joint_values):
        req = PTPPlanningRequest()
        req.joint_goal = joint_values
        req.start_config = start_config

        resp = self.ptp(req)
        if not resp.success:
            raise PlanningFailedError("Movej command failed.")

        return points_to_plan(resp.joint_path)

    @log_motion_command
    def movep(self, start_config, pose_goal):
        req = PTPPlanningRequest()
        req.pose_goal = pose_goal
        req.start_config = start_config

        resp = self.ptp(req)
        if not resp.success:
            raise PlanningFailedError("Movep command failed.")

        return points_to_plan(resp.joint_path)

    @log_motion_command
    def movel(self, start_config, pose_goal):
        req = LINPlanningRequest()
        req.use_start_config = True
        req.start_config = start_config
        req.pose_goal = pose_goal
        req.has_constraints = False

        resp = LINPlanningResponse(success=False)
        try:
            resp = self.lin(req)
        except rospy.service.ServiceException as e:
            print(e)
        # resp = self.cart_servers.s[0](req)

        if not resp.success:
            raise PlanningFailedError("Movel command failed.")

        return points_to_plan(resp.joint_path)

    @log_motion_command
    def movel_no_start_config(self, pose_start, pose_goal):
        print("===========Planning to pose")
        print(pose_goal)
        req = LINPlanningRequest()
        req.use_start_config = False
        req.pose_start = pose_start
        req.pose_goal = pose_goal
        req.has_constraints = False

        resp = LINPlanningResponse(success=False)
        try:
            resp = self.lin(req)
        except rospy.service.ServiceException as e:
            print(e)
        # resp = self.cart_servers.s[0](req)

        if not resp.success:
            raise PlanningFailedError("Movel command failed.")

        return points_to_plan(resp.joint_path)

    # def sample_cons(self, start_config, pose_goal):
    #     req = SampleConstraintRequest()
    #     constraint = nexon_msgs.msg.PoseConstraint()
    #     constraint.relative = True
    #     constraint.rpy_min = [0, 0, -math.pi]
    #     constraint.rpy_max = [0, 0, math.pi]
    #     req.pose = pose_goal
    #     req.constraint = constraint
    #     resp = self.sample(req)

    #     if len(resp.joint_poses) == 0:
    #         raise PlanningFailedError("Sampling command failed.")

    #     return resp
