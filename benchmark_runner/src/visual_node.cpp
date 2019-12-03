#include <string>

// ROS
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include <nexon_msgs/ShowJointTrajectory.h>
#include <nexon_msgs/ShowJointPath.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace rvt = rviz_visual_tools;

static const std::string PLANNING_GROUP_NAME = "manipulator";

class VisualServices
{
public:
  /**
   * \brief Constructor
   */
  VisualServices()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/visual_utils"));
    visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->loadMarkerPub(true);
    visual_tools_->loadRobotStatePub("display_robot_state");
    visual_tools_->setManualSceneUpdating();

    robot_state_ = visual_tools_->getSharedRobotState();
    jmg_ = robot_state_->getJointModelGroup(PLANNING_GROUP_NAME);

    // Allow time to publish messages
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    // Clear collision objects and markers
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->triggerPlanningSceneUpdate();
    ros::Duration(0.1).sleep();

    show_trajectory_server_ = nh_.advertiseService("show_trajectory", &VisualServices::showTrajectory, this);

    // runRobotStateTests();
  }

  void runRobotStateTests()
  {
    // Show 5 random robot states
    for (std::size_t i = 0; i < 5; ++i)
    {
      robot_state_->setToRandomPositions(jmg_);
      visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
      ros::Duration(0.2).sleep();
    }

    // Show 5 robot state in different colors
    for (std::size_t i = 0; i < 5; ++i)
    {
      robot_state_->setToRandomPositions(jmg_);
      visual_tools_->publishRobotState(robot_state_, visual_tools_->getRandColor());
      ros::Duration(0.2).sleep();
    }

    // Hide the robot
    visual_tools_->hideRobot();
    ros::Duration(0.5).sleep();

    // Show the robot
    visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
    ros::Duration(0.1).sleep();
  }

  bool showTrajectory(nexon_msgs::ShowJointPathRequest& req, nexon_msgs::ShowJointPathResponse& resp)
  {
    // moveit_msgs::RobotTrajectory robot_trajectory;
    // robot_trajectory.joint_trajectory = req.joint_trajectory;
    // visual_tools_->publishTrajectoryPath(robot_trajectory, robot_state_);

    for (auto q : req.joint_path.path)
    {
      robot_state_->setJointGroupPositions(jmg_, q.positions);
      visual_tools_->publishRobotState(robot_state_, visual_tools_->getRandColor());
      ros::Duration(0.2).sleep();
    }
    return true;
  }

private:
  ros::NodeHandle nh_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const moveit::core::JointModelGroup* jmg_;
  moveit::core::RobotStatePtr robot_state_;

  ros::ServiceServer show_trajectory_server_;
};  // end class

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_node");
  ROS_INFO_STREAM("Launching visualization node.");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  VisualServices demoer;

  // while (ros::ok())
  // {
  demoer.runRobotStateTests();
  // }

  ros::waitForShutdown();
  ROS_INFO_STREAM("Shutting down.");

  return 0;
}