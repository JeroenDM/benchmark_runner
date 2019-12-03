#!/usr/bin/env python
"""
Load work objects from urdf file.
Assume the objects are defined with respect to a global link
called 'world', or with respect to links in the urdf file.
(The latter implies that the links are defined in the correct order.)
"""
import sys
import rospy
import rospkg
import rosparam
import moveit_commander
import urdfpy
import nexon.io

from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped

# This convention should stay relatively constant
REL_WORK_PATH = "/urdf/work/"
REL_TASK_PATH = "/tasks/"


def numpy_to_pose(arr):
    """ Numpy 4x4 array to geometry_msg.Pose

    Code from: https://github.com/eric-wieser/ros_numpy
    TODO move this to some utility module if I have one.
    """
    from tf import transformations
    assert arr.shape == (4, 4)

    trans = transformations.translation_from_matrix(arr)
    quat = transformations.quaternion_from_matrix(arr)

    return Pose(
        position=Vector3(*trans),
        orientation=Quaternion(*quat)
    )


def remove_all_objects(scene):
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)


def parse_file(package_name, work_name):
    rospack = rospkg.RosPack()

    filepath = rospack.get_path(package_name)
    filepath += REL_WORK_PATH
    filepath += work_name + ".urdf"

    return urdfpy.URDF.load(filepath)


def parse_link(link):
    """ Assume a link has only a single collision object.
        Assume this collision object is a box.
        Assume the link named "world" has no collision objects.
    """
    assert len(link.collisions) == 1
    assert link.collisions[0].geometry.box is not None
    assert link.name != "world"
    return link.collisions[0].geometry.box.size


def publish_parsed_urdf(parsed_urdf, scene):
    links = {}
    for link in parsed_urdf.links:
        if link.name == "world":
            continue
        else:
            links[link.name] = {"size": parse_link(link)}

    for joint in parsed_urdf.joints:
        p = PoseStamped()
        p.header.frame_id = joint.parent
        p.pose = numpy_to_pose(joint.origin)
        scene.add_box(joint.child, p, links[joint.child]["size"])


def list_work_objects(package_name):
    import glob
    rospack = rospkg.RosPack()

    filepath = rospack.get_path(package_name)
    filepath += REL_WORK_PATH
    print("Looking in directory:\n{}".format(filepath))
    print("\nFound:")
    for file in glob.glob(filepath + "*.urdf"):
        print("\t" + file.replace(filepath, "").replace(".urdf", ""))


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Two argument required:")
        print("<package_name> <work_name>:\tpublish urdf file <work_name>.urdf.")
        print("<package_name> -l:\tlist available work objects.")
        exit()
    else:
        package_name = sys.argv[1]
        command = sys.argv[2]
        if command == "-l":
            list_work_objects(package_name)
            exit()
        else:
            work_name = command

    rospy.init_node("publish_work")

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)  # wait for the above things to setup

    remove_all_objects(scene)

    work = parse_file(package_name, work_name)
    publish_parsed_urdf(work, scene)

    # parse and load task to parameter server
    rospack = rospkg.RosPack()
    filename = work_name + ".irl"
    task_path = rospack.get_path(package_name) + REL_TASK_PATH + filename
    rosparam.set_param("/planning_task_path", task_path)

    print("Done!")
