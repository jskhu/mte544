
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

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace Eigen;

enum UpdateState : int
{
   UNKNOWN,
   FREE,
   OCCUPIED,
};

// function for line generation from https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/
std::vector<Vector2i> bresenham(int x1, int y1, int x2, int y2)
{

   std::vector<Vector2i> intermediate_points;

   int m_new = 2 * (y2 - y1);
   int slope_error_new = m_new - (x2 - x1);
   for (int x = x1, y = y1; x <= x2; x++)
   {
      intermediate_points.push_back(Vector2i(x, y));

      // Add slope to increment angle formed
      slope_error_new += m_new;

      // Slope error reached limit, time to
      // increment y and update slope error.
      if (slope_error_new >= 0)
      {
         y++;
         slope_error_new -= 2 * (x2 - x1);
      }
   }
   return intermediate_points;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {}

int main(int argc, char **argv)
{
   // Initialize the ROS framework
   ros::init(argc, argv, "occ");
   ros::NodeHandle n;

   // Setup topics to Publish from this node
   ros::Publisher occ_publisher =
       n.advertise<nav_msgs::OccupancyGrid>("/occ", 1);

   ros::Subscriber scan_subscriber = n.subscribe("/scan", 1, scan_callback);

   // Velocity control variable
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

   // Set the loop rate`
   ros::Rate loop_rate(20); // 20Hz update rate

   while (ros::ok())
   {
      loop_rate.sleep(); // Maintain the loop rate
      ros::spinOnce();   // Check for new messages

      // Draw Curves
      occ_publisher.publish(occ_grid);
   }

   return 0;
}
