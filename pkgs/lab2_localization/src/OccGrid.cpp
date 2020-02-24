#include "OccGrid.hpp"

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map>

OccGrid::OccGrid(float width, float height, float resolution,
                 const Eigen::Vector3f& origin,
                 const std::map<UpdateState, float>& update_values) :
    m_width(width), m_height(height),
    m_resolution(resolution), m_origin(origin),
    is_init(false), m_update_values(update_values) {
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
    for (auto& pixel : m_msg.data) {
        pixel = m_update_values[UNKNOWN];
    }

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

void OccGrid::update_pixel(const Eigen::Vector2i& coord, UpdateState state) {
    float l = log_odds((*this)[coord]) +
        log_odds(m_update_values[state]) -
        log_odds(m_update_values[UNKNOWN]);
    (*this)[coord] = prob(l);
}

void OccGrid::publish() {
    if (!is_init) {
        ROS_WARN_STREAM("Occupancy Grid publisher not initialized!");
        return;
    }
    m_msg.info.map_load_time = ros::Time::now();
    m_occ_publisher.publish(m_msg);
}

float OccGrid::log_odds(float prob) {
    return log(prob/(1-prob));
}

float OccGrid::prob(float log_odds) {
    return  exp(log_odds) / (1 + exp(log_odds));
}
