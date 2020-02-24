
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

#include "OccGrid.hpp"

ros::Publisher pose_publisher;
tf::Transform world_t_robot;

enum UpdateState : int {
  UNKNOWN,
  FREE,
  OCCUPIED,
};

// function for line generation from
// https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/
std::vector<Eigen::Vector2i> get_bresenham_points(int x1, int y1, int x2,
                                                  int y2) {

  std::vector<Eigen::Vector2i> intermediate_points;

  int m_new = 2 * (y2 - y1);
  int slope_error_new = m_new - (x2 - x1);
  for (int x = x1, y = y1; x <= x2; x++) {
    intermediate_points.push_back(Eigen::Vector2i(x, y));

    // Add slope to increment angle formed
    slope_error_new += m_new;

    // Slope error reached limit, time to
    // increment y and update slope error.
    if (slope_error_new >= 0) {
      y++;
      slope_error_new -= 2 * (x2 - x1);
    }
  }
  return intermediate_points;
}

void pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  tf::poseMsgToTF(msg->pose.pose, world_t_robot);
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  tf::Vector3 cur_origin = world_t_robot.getOrigin();

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    float cur_angle = msg->angle_min + msg->angle_increment * i;

    float x = msg->ranges[i] * cos(cur_angle);
    float y = msg->ranges[i] * sin(cur_angle);

    tf::Vector3 laser_coords(x, y, cur_origin.getZ());
    tf::Transform robot_t_laser;
    robot_t_laser.setOrigin(laser_coords);

    tf::Transform world_t_laser = world_t_robot * robot_t_laser;

    get_bresenham_points(cur_origin.getX(), cur_origin.getY(),
                         world_t_laser.getOrigin().getX(),
                         world_t_laser.getOrigin().getY());
  }
}

int main(int argc, char **argv) {
  // Initialize the ROS framework
  ros::init(argc, argv, "occ");
  ros::NodeHandle n;

  bool use_sim;
  n.param("use_sim", use_sim, true);

  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);

  ros::Subscriber scan_subscriber = n.subscribe("/scan", 1, scan_callback);

  ros::Subscriber pose_subscriber =
      n.subscribe("/indoor_pos", 1, pose_callback);

  OccGrid occ_grid(3.0f, 3.0f, 1.0f, Eigen::Vector3f(0, 0, 0));
  occ_grid.init_publisher(n, "/occ");

  // occ_grid.data.resize(9);
  int foo[] = {100, 0, 0, 0, 0, 100, 100, 100, 0};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ROS_INFO_STREAM(foo[i * 3 + j]);
      occ_grid[Eigen::Vector2i(j, i)] = foo[i * 3 + j];
    }
  }

  // Set the loop rate`
  ros::Rate loop_rate(20); // 20Hz update rate

  while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages

    // Draw Curves
    occ_grid.publish();
  }

  return 0;
}
