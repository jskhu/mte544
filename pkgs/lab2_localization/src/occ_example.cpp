
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

// function for line generation from
// https://www.thecrazyprogrammer.com/2017/01/bresenhams-line-drawing-algorithm-c-c.html
std::vector<Eigen::Vector2i> get_bresenham_points(int x1, int y1, int x2,
                                                  int y2) {

  std::vector<Eigen::Vector2i> intermediate_points;

  if (x1 > x2) {
    int t = x1;
    x1 = x2;
    x2 = t;
    t = y1;
    y1 = y2;
    y2 = t;
  }

  int dx = x2 - x1;
  int dy = y2 - y1;

  int x = x1;
  int y = y1;

  int p = 2 * dy - dx;

  while (x < x2) {
    if (p >= 0) {
      intermediate_points.push_back(Eigen::Vector2i(x, y));
      y = y + 1;
      p = p + 2 * dy - 2 * dx;
    } else {
      intermediate_points.push_back(Eigen::Vector2i(x, y));
      p = p + 2 * dy;
    }
    x = x + 1;
    // ROS_INFO_STREAM("x: " << x << " x1: " << x1 << " x2: " << x2);
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
    float cur_angle = msg->angle_min + msg->angle_increment * i;

    float x = msg->ranges[i] * cos(cur_angle);
    float y = msg->ranges[i] * sin(cur_angle);

    tf::Vector3 laser_coords(x, y, cur_origin.getZ());
    tf::Transform robot_t_laser;
    robot_t_laser.setOrigin(laser_coords);

    tf::Transform world_t_laser = world_t_robot * robot_t_laser;

    // TODO: Add bresenham, hack for now
    double cur_origin_x = cur_origin.getX();
    double cur_origin_y = cur_origin.getY();
    double laser_x = world_t_laser.getOrigin().getX();
    double laser_y = world_t_laser.getOrigin().getY();

    std::vector<std::pair<double,double>> points = {{cur_origin_x, cur_origin_y},
                                     {laser_x, laser_y}};

    for (int i = 0; i < points.size() - 1; i++) {
      Eigen::Vector2i point;
      if (occ_grid.convert_to_occ_coords(points[i].first, points[i].second, point)) {
        occ_grid.update_pixel(point, FREE);
      }
    }
    Eigen::Vector2i point;
    if (occ_grid.convert_to_occ_coords(points[points.size()-1].first, points[points.size()-1].second, point)) {
      occ_grid.update_pixel(point, OCCUPIED);
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
  update_states[FREE] = 0.3;
  update_states[OCCUPIED] = 0.6;
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
