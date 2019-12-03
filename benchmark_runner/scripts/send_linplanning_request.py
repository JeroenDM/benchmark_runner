#!/usr/bin/env python
import rospy
import rospkg
from geometry_msgs.msg import Pose, Vector3, Quaternion
from nexon_msgs.msg import PoseConstraint
from nexon_msgs.srv import LINPlanning, LINPlanningRequest


def pose_to_msg(pose):
    pos = pose["xyz"]
    quat = pose["xyzw"]
    pos_msg = Vector3(x=pos[0], y=pos[1], z=pos[2])
    quat_msg = Quaternion(
        x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return Pose(position=pos_msg, orientation=quat_msg)


def send_test_request(pose):
    print("Waiting for linear planning server")
    rospy.wait_for_service("linear_planning_request")
    print("Linear planning server found, sending test request!")
    try:
        lp_service = rospy.ServiceProxy("linear_planning_request", LINPlanning)
        request = LINPlanningRequest()
        request.pose_goal = pose_to_msg(pose)
        request.start_config = [0, 0, 0, 0, 0, 0]

        con = PoseConstraint()
        con.symmetric = True
        con.relative = True
        con.xyz = [0, 0, 0]
        con.rpy = [0, 0, 3.14]

        request.pose_constraint = con

        resp = lp_service(request)
        print(resp)

    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))


if __name__ == "__main__":
    test_pose = {}
    test_pose["xyz"] = [0, 0, 0]
    test_pose["xyzw"] = [0, 0, 0, 1]

    send_test_request(test_pose)
