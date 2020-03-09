
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
std::map<UpdateState, float> update_states;
OccGrid occ_grid(2000, 2000, 0.01, Eigen::Vector3f(-10, -10, 0), update_states);

// Function for line generation adapted from
// http://tech-algorithm.com/articles/drawing-line-using-bresenham-algorithm/
std::vector<Eigen::Vector2i> get_bresenham_points(int x1, int y1, int x2,
                                                  int y2) {

  std::vector<Eigen::Vector2i> intermediate_points;

  int w = x2 - x1;
  int h = y2 - y1;
  int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;
  if (w < 0)
    dx1 = -1;
  else if (w > 0)
    dx1 = 1;
  if (h < 0)
    dy1 = -1;
  else if (h > 0)
    dy1 = 1;
  if (w < 0)
    dx2 = -1;
  else if (w > 0)
    dx2 = 1;
  int longest = abs(w);
  int shortest = abs(h);
  if (!(longest > shortest)) {
    longest = abs(h);
    shortest = abs(w);
    if (h < 0)
      dy2 = -1;
    else if (h > 0)
      dy2 = 1;
    dx2 = 0;
  }
  int numerator = longest >> 1;
  for (int i = 0; i <= longest; i++) {
    intermediate_points.push_back(Eigen::Vector2i(x1, y1));
    numerator += shortest;
    if (!(numerator < longest)) {
      numerator -= longest;
      x1 += dx1;
      y1 += dy1;
    } else {
      x1 += dx2;
      y1 += dy2;
    }
  }

  return intermediate_points;
}

void pose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  tf::poseMsgToTF(msg->pose.pose, world_t_robot);
  world_t_robot.getOrigin().setZ(0);
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  tf::Vector3 cur_origin = world_t_robot.getOrigin();

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    if (std::isnan(msg->ranges[i])) {
      continue;
    }
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max){
      continue;
    }
    float cur_angle = msg->angle_min + msg->angle_increment * i;

    float x = msg->ranges[i] * cos(cur_angle);
    float y = msg->ranges[i] * sin(cur_angle);

    tf::Vector3 laser_coords(x, y, cur_origin.getZ());
    tf::Transform robot_t_laser;
    robot_t_laser.setOrigin(laser_coords);

    tf::Transform world_t_laser = world_t_robot * robot_t_laser;

    double cur_origin_x = cur_origin.getX();
    double cur_origin_y = cur_origin.getY();
    double laser_x = world_t_laser.getOrigin().getX();
    double laser_y = world_t_laser.getOrigin().getY();

    Eigen::Vector2i occ_cur_origin;
    Eigen::Vector2i occ_laser;
    if (occ_grid.convert_to_occ_coords(cur_origin_x, cur_origin_y,
                                       occ_cur_origin) &&
        occ_grid.convert_to_occ_coords(laser_x, laser_y, occ_laser)) {
      std::vector<Eigen::Vector2i> points = get_bresenham_points(
          occ_cur_origin.x(), occ_cur_origin.y(), occ_laser.x(), occ_laser.y());
      for (int i = 0; i < points.size() - 1; i++) {
        occ_grid.update_pixel(points[i], FREE);
      }
      occ_grid.update_pixel(points[points.size() - 1], OCCUPIED);
    }
  }
}

int main(int argc, char **argv) {
  // Initialize the ROS framework
  ros::init(argc, argv, "occ");
  ros::NodeHandle n;

  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);

  ros::Subscriber scan_subscriber = n.subscribe("/scan", 1, scan_callback);

  ros::Subscriber pose_subscriber =
      n.subscribe("/indoor_pos", 1, pose_callback);

  // Initialize the occupancy grid
  // For some reason, can't add members to update states outside function
  // Add here instead
  update_states[UNKNOWN] = 0.5;
  update_states[FREE] = 0.4;
  update_states[OCCUPIED] = 0.7;
  occ_grid.init_publisher(n, "/occ");

  // Set the loop rate`
  ros::Rate loop_rate(20); // 20Hz update rate

  while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages
    occ_grid.publish();
  }

  return 0;
}
