#include "OccGrid.hpp"

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

OccGrid::OccGrid(float width, float height, float resolution,
                 const Eigen::Vector3f& origin) :
    m_width(width), m_height(height),
    m_resolution(resolution), m_origin(origin),
    is_init(false) {
}

void OccGrid::init_publisher(ros::NodeHandle& n, const std::string& topic) {
    // Populate the m_grid message
    m_msg.info.height = m_height;
    m_msg.info.width = m_width;
    m_msg.info.resolution = m_resolution;
    m_msg.info.origin.position.x = m_origin[0];
    m_msg.info.origin.position.y = m_origin[1];
    m_msg.info.origin.position.z = m_origin[2];

    m_msg.data.resize(m_height * m_width);

   m_occ_publisher = n.advertise<nav_msgs::OccupancyGrid>(topic, 1);

   is_init = true;
}

OccGrid::~OccGrid() {}

int8_t OccGrid::operator[] (const Eigen::Vector2i& coord) const {
    return m_msg.data[coord[1] * m_width + coord[0]];
}

int8_t& OccGrid::operator[] (const Eigen::Vector2i& coord) {
    return m_msg.data[coord[1] * m_width + coord[0]];
}

void OccGrid::publish() {
    if (!is_init) {
        ROS_WARN_STREAM("Occupancy Grid publisher not initialized!");
        return;
    }
    m_msg.info.map_load_time = ros::Time::now();
    m_occ_publisher.publish(m_msg);
}