
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
	geometry_msgs::PoseStamped curpose;

	curpose.pose = msg.pose[19];
	curpose.header.frame_id="/robot";

	//republish pose for rviz
	pose_publisher.publish(curpose);
}

void noisy_pose_callback(const geometry_msgs::PoseStamped& msg) {
    geometry_msgs::PoseStamped noisy_msg = msg;
    // TODO: add noise
    noisy_pose_publisher.publish(noisy_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "icp_converter");
    ros::NodeHandle n;

    bool use_sim;
    n.param("use_sim", use_sim, true);

    ros::Subscriber pose_sub;
    if (use_sim) {
        pose_sub = n.subscribe("/gazebo/model_states", 1, pose_sim_callback);
    } else {
        pose_sub = n.subscribe("/ips", 1, pose_sim_callback);
    }
    ros::Subscriber noisy_pose_sub = n.subscribe("/pose", 1, noisy_pose_callback);

    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    noisy_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/noisy_pose", 1, true);

    ros::spin();

    return 0;
}