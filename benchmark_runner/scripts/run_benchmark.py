#!/usr/bin/env python
import rospy
import rospkg
from nexon_msgs.srv import PTPPlanning, PTPPlanningRequest
from geometry_msgs.msg import Pose, Vector3, Quaternion

from nexon.benchmarking import Task, create_runners
from nexon.io import parse_file


if __name__ == "__main__":
    rospy.init_node('benchmark_node')

    taskname = "pick_task_1"

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("main_server")
    filepath = pkg_path + "/data/" + taskname + ".txt"

    rospy.set_param(taskname, parse_file(filepath))

    task = Task(taskname)
    runners = create_runners("benchmark_planners")

    runners[1].run(task)

    # rospy.spin()
