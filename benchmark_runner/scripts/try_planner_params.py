#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.srv

from nexon.robot import Robot


if __name__ == "__main__":
    rospy.init_node("try_planner_params")

    # set_planner_params = rospy.ServiceProxy(
    #     "/set_planner_params", moveit_msgs.srv.SetPlannerParams)

    # moveit_commander.roscpp_initialize(sys.argv)
    # mc = moveit_commander.RobotCommander()
    # mg = moveit_commander.MoveGroupCommander("manipulator")

    # req = moveit_msgs.srv.SetPlannerParamsRequest()
    # req.planner_config = "FMT"
    # req.group = "manipulator"
    # req.params = moveit_msgs.msg.PlannerParams(
    #     keys=["longest_valid_segment_fraction", "num_samples"],
    #     values=["0.001", "100"]
    # )

    # set_planner_params(req)

    # mg.set_planning_time(3.0)

    # mg.set_planner_id("FMT")

    robot = Robot()

    config = {"planner_id": "FMT",
              "planning_time": "3.0",
              "planner_params": {
                  "longest_valid_segment_fraction": "0.001"
              }}

    print(config)

    robot.set_ompl_planner_params(config)

    jv = mg.get_current_joint_values()
    jv[0] += 0.5
    mg.set_joint_value_target(jv)
    mg.plan()

    print(jv)
