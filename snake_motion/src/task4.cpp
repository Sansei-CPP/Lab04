#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <stdio.h>

// Main moveit libraries are included
int main(int argc, char **argv)
{
  double PI = 3.1415;
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(0);
  spinner.start(); // For moveit implementation we need AsyncSpinner, we cant use ros::spinOnce()
  static const std::string PLANNING_GROUP = "snake_arm"; /* Now we specify with what group we want work,
  here group1 is the name of my group controller*/
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); // loading move_group

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //For joint control
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped target_pose; // Pose in ROS is implemented using geometry_msgs::PoseStamped, google what is the type of this msg
  current_pose = move_group.getCurrentPose(); /* Retrieving the information about the
  current position and orientation of the end effector*/
  target_pose = current_pose; /* Basically our target pose is the same as current,
  except that we want to move it a little bit along x-axis*/
  ros::Rate loop_rate(50); //Frequency
  while (ros::ok()){
	  
	  double rad = 0.5; // we need to shift in along x in negative direction to allow it to do a full circle
	  double angle = 9.0;
	  double sin_ang = 0.0;
	  double cos_ang = 0.0;
	  double origin = current_pose.pose.position.x - rad;
	  
	  while (angle <= 367){
			sin_ang = sin(angle*PI/180); //calculating sine of angle in radians
			cos_ang = cos(angle*PI/180); // calcualting cos of angle in radians
			
			if (angle >= 367){
				break;
			}
			target_pose.pose.position.y = sin_ang; // assigning y coordinate as sine of angle
			target_pose.pose.position.x = origin + cos_ang; //assigning x coordinate as cosine of angle relative to the center of the circle
			move_group.setApproximateJointValueTarget(target_pose); // To calculate the trajectory
			move_group.move(); // Move the robot
			current_pose = move_group.getCurrentPose();
			angle = angle + 15; // updating the angle
	}
	break;
    loop_rate.sleep();
  }

  ROS_INFO("Done circle");
  ros::shutdown();
  return 0;
}