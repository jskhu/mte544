
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
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>


ros::Publisher occ_publisher;

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"occ");
    ros::NodeHandle n;

    //Setup topics to Publish from this node
    ros::Publisher occ_publisher = n.advertise<nav_msgs::OccupancyGrid>("/occ", 1);

    //Velocity control variable
    nav_msgs::MapMetaData meta_data;
    nav_msgs::OccupancyGrid occ_grid;

    meta_data.height = 3;
    meta_data.width = 3;
    meta_data.resolution = 1.0f;
    meta_data.origin = geometry_msgs::Pose();
    occ_grid.info = meta_data;
    occ_grid.data.resize(9);
    // int foo[] = {255, 0, 0,
    //              0, 0, 255,
    //              255, 255, 0};
    // for (int i = 0; i < 9; ++i) {
    //     occ_grid.data.push_back(foo[i]);
    // }

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate


    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

	 //Draw Curves
     occ_publisher.publish(occ_grid);
    }

    return 0;
}
