#pragma once

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map>

enum UpdateState : int {
    UNKNOWN,
    FREE,
    OCCUPIED,
};

class OccGrid {
public:
    OccGrid(float width, float height, float resolution,
            const Eigen::Vector3f& origin,
            const std::map<UpdateState, float>& update_values);
    ~OccGrid();

    int8_t operator[] (const Eigen::Vector2i& coord) const;
    int8_t& operator[] (const Eigen::Vector2i& coord);
    void update_pixel(const Eigen::Vector2i& coord, UpdateState state);

    void init_publisher(ros::NodeHandle& n, const std::string& topic);
    void publish();

private:
    float log_odds(float prob);
    float prob(float log_odds);

   ros::Publisher m_occ_publisher;
   float m_width, m_height, m_resolution;
   Eigen::Vector3f m_origin;
   nav_msgs::OccupancyGrid m_msg;
   std::map<UpdateState, float> m_update_values;

   bool is_init;
};
