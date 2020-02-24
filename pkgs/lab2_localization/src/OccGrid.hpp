#pragma once

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class OccGrid {
public:
    OccGrid(float width, float height, float resolution,
            const Eigen::Vector3f& origin);
    ~OccGrid();

    int8_t operator[] (const Eigen::Vector2i& coord) const;
    int8_t& operator[] (const Eigen::Vector2i& coord);
    void init_publisher(ros::NodeHandle& n, const std::string& topic);
    void publish();

private:
   ros::Publisher m_occ_publisher;
   float m_width, m_height, m_resolution;
   Eigen::Vector3f m_origin;
   nav_msgs::OccupancyGrid m_msg;

   bool is_init;
};