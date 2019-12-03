#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        rospy.sleep(rospy.Duration(1.0))
        print "are we publishing?"
        print "number of connections: %d" % display_trajectory_publisher.get_num_connections()

        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):

        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3

        plan = move_group.plan(joint_goal)
        move_group.execute(plan, wait=True)
        # plan = move_group.go(joint_goal, wait=True)
        # print(plan.joint_trajectory)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()

        N = 10
        q1 = np.array(current_joints)
        q2 = q1 + np.array([1.5, 0.1, 0, 0, 0, 0])
        q_path = np.zeros((N, 6))
        for i, s in enumerate(np.linspace(0, 1, N)):
            q_path[i] = (1 - s) * q1 + s * q2

        trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(N):
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            pt.positions = list(q_path[i])
            pt.time_from_start = rospy.Duration(float(0.1 * i))
            trajectory.points.append(pt)
            print(pt)
            plan.joint_trajectory.points[i].positions = list(q_path[i])

        # robot_trajectory = moveit_msgs.msg.RobotTrajectory()
        # robot_trajectory.joint_trajectory.header.frame_id = "world"
        # # robot_trajectory.joint_names = move_group.get_joints()
        # robot_trajectory.joint_trajectory = trajectory

        print("Show trajectory -------------------------")
        # rospy.Duration(1.0).sleep()
        self.display_trajectory(plan)
        rospy.sleep(rospy.Duration(5.0))

        print("Moving to home position")
        move_group.set_named_target("home")
        move_group.go(wait=True)
        move_group.stop()

        return all_close(joint_goal, current_joints, 0.01)

    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory.model_id = "demo_table"
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    # def display_trajectory_2(self, joint_trajectory):

    #     robot = self.robot
    #     display_trajectory_publisher = self.display_trajectory_publisher

    #     robot_trajectory = moveit_msgs.msg.RobotTrajectory()
    #     robot_trajectory.joint_trajectory = joint_trajectory

    #     print "Robot trajectory ----------------------------"
    #     print(robot_trajectory)

    #     display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    #     display_trajectory.trajectory_start = robot.get_current_state()

    #     display_trajectory.trajectory.append(robot_trajectory)

    #     # Publish
    #     display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)


def main():
    try:
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.go_to_joint_state()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
