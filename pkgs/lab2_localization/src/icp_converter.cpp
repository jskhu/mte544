
//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>

ros::Publisher pose_publisher;
ros::Publisher noisy_pose_publisher;

void pose_sim_callback(const gazebo_msgs::ModelStates& msg) {
	//This function is called when a new position message is received
	geometry_msgs::PoseWithCovarianceStamped curpose;

	curpose.pose.pose = msg.pose[19];
	// curpose.pose.pose = msg.pose[6];
    curpose.header.stamp = ros::Time::now();

	//republish pose for rviz
	pose_publisher.publish(curpose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "icp_converter");
    ros::NodeHandle n;

    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_sim_callback);
    pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/indoor_pos", 1, true);

    ros::spin();

    return 0;
}