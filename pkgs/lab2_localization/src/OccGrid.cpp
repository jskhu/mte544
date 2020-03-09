#include "OccGrid.hpp"

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map>

OccGrid::OccGrid(int cells_x, int cells_y, float resolution,
                 const Eigen::Vector3f& origin,
                 std::map<UpdateState, float>& update_values) :
    m_cells_x(cells_x), m_cells_y(cells_y),
    m_resolution(resolution), m_origin(origin),
    is_init(false), m_update_values(update_values) {
        m_width = resolution * m_cells_x;
        m_height = resolution * m_cells_y;
}

void OccGrid::init_publisher(ros::NodeHandle& n, const std::string& topic) {
    // Populate the m_grid message
    m_msg.info.height = m_cells_y;
    m_msg.info.width = m_cells_x;
    m_msg.info.resolution = m_resolution;
    m_msg.info.origin.position.x = m_origin[0];
    m_msg.info.origin.position.y = m_origin[1];
    m_msg.info.origin.position.z = m_origin[2];

    for (int i = 0; i < m_cells_x * m_cells_y; ++i) {
        m_msg.data.push_back(100 * m_update_values[UNKNOWN]);
    }

   m_occ_publisher = n.advertise<nav_msgs::OccupancyGrid>(topic, 1);

   is_init = true;
}

OccGrid::~OccGrid() {}

int8_t OccGrid::operator[] (const Eigen::Vector2i& coord) const {
    return m_msg.data[coord[1] * m_cells_x + coord[0]];
}

int8_t& OccGrid::operator[] (const Eigen::Vector2i& coord) {
    return m_msg.data[coord[1] * m_cells_x + coord[0]];
}

void OccGrid::update_pixel(const Eigen::Vector2i& coord, UpdateState state) {
    float l = log_odds((*this)[coord] * 0.01f) +
        log_odds(m_update_values[state]) -
        log_odds(m_update_values[UNKNOWN]);
    (*this)[coord] = prob(l) * 100;
}

void OccGrid::publish() {
    if (!is_init) {
        ROS_WARN_STREAM("Occupancy Grid publisher not initialized!");
        return;
    }
    m_msg.info.map_load_time = ros::Time::now();
    m_occ_publisher.publish(m_msg);
}

bool OccGrid::convert_to_occ_coords(double x_world, double y_world, Eigen::Vector2i& point_occ) {
    double x_occ = x_world - m_origin[0];
    double y_occ = y_world - m_origin[1];
    // ROS_INFO_STREAM("x_occ: " << x_occ << " y_occ: " << y_occ);
    if (x_occ < 0 || x_occ > m_width) {
        return false;
    }
    if (y_occ < 0 || y_occ >= m_height) {
        return false;
    }

    point_occ[0] = (x_occ + 1e-5) / m_width * m_cells_x;
    point_occ[1] = (y_occ + 1e-5) / m_height * m_cells_y;
    return true;
}

float OccGrid::log_odds(float prob) {
    return log(prob/(1-prob));
}

float OccGrid::prob(float log_odds) {
    return  exp(log_odds) / (1 + exp(log_odds));
}
