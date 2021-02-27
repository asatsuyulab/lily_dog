#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveit_commander");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Set up the my_leg_robot planning interface
  moveit::planning_interface::MoveGroupInterface my_leg_robot("leg");

  // Prepare
  ROS_INFO("Moving to prepare pose");
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", my_leg_robot.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", my_leg_robot.getEndEffectorLink().c_str());
  my_leg_robot.setPlanningTime(0.100);
  my_leg_robot.setPlannerId("RRTConnect");
  my_leg_robot.setGoalTolerance(0.01);

  // pose1
  geometry_msgs::PoseStamped pose1;
  pose1 = my_leg_robot.getCurrentPose();
  pose1.pose.orientation.x += 0.02;

  // pose2
  geometry_msgs::PoseStamped pose2 = my_leg_robot.getCurrentPose();
  pose2.header.frame_id = "world";
  pose2.pose.position.x = 0.0008;
  pose2.pose.position.y = 0.171;
  pose2.pose.position.z = 0.1;

  ROS_INFO("wait...");
  ros::Duration(20.0).sleep();

  moveit::planning_interface::MoveItErrorCode ret;
  my_leg_robot.setGoalOrientationTolerance(1.0);

  ROS_INFO("move to WP1");
  my_leg_robot.setApproximateJointValueTarget(pose1);
  ROS_INFO("current pose: frame:%s  x:%f  y:%f  z:%f  rx:%f  ry:%f  rz:%f  rw:%f", 
            my_leg_robot.getCurrentPose().header.frame_id.c_str(), 
            my_leg_robot.getCurrentPose().pose.position.x, my_leg_robot.getCurrentPose().pose.position.y, my_leg_robot.getCurrentPose().pose.position.z, 
            my_leg_robot.getCurrentPose().pose.orientation.x, my_leg_robot.getCurrentPose().pose.orientation.y, my_leg_robot.getCurrentPose().pose.orientation.z, my_leg_robot.getCurrentPose().pose.orientation.w);
  ROS_INFO("goal pose: frame:%s  x:%f  y:%f  z:%f  rx:%f  ry:%f  rz:%f  rw:%f", 
            pose1.header.frame_id.c_str(), 
            pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z, 
            pose1.pose.orientation.x, pose1.pose.orientation.y, pose1.pose.orientation.z, pose1.pose.orientation.w);
  ret = my_leg_robot.move();
  if (!ret) {
    ROS_WARN("Fail: %i", ret.val);
  }
  ros::Duration(2.0).sleep();

  ROS_INFO("move to WP2");
  my_leg_robot.setApproximateJointValueTarget(pose2);
  ROS_INFO("current pose: frame:%s  x:%f  y:%f  z:%f  rx:%f  ry:%f  rz:%f  rw:%f", 
            my_leg_robot.getCurrentPose().header.frame_id.c_str(), 
            my_leg_robot.getCurrentPose().pose.position.x, my_leg_robot.getCurrentPose().pose.position.y, my_leg_robot.getCurrentPose().pose.position.z, 
            my_leg_robot.getCurrentPose().pose.orientation.x, my_leg_robot.getCurrentPose().pose.orientation.y, my_leg_robot.getCurrentPose().pose.orientation.z, my_leg_robot.getCurrentPose().pose.orientation.w);
  ROS_INFO("goal pose: frame:%s  x:%f  y:%f  z:%f  rx:%f  ry:%f  rz:%f  rw:%f", 
            pose2.header.frame_id.c_str(), 
            pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z, 
            pose2.pose.orientation.x, pose2.pose.orientation.y, pose2.pose.orientation.z, pose2.pose.orientation.w);
  ret = my_leg_robot.move();
  if (!ret) {
    ROS_WARN("Fail: %i", ret.val);
  }
  ros::Duration(2.0).sleep();

  ros::shutdown();
  return 0;
}