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
    OccGrid(int cells_x, int cells_y, float resolution,
            const Eigen::Vector3f& origin,
            std::map<UpdateState, float>& update_values);
    ~OccGrid();

    int8_t operator[] (const Eigen::Vector2i& coord) const;
    int8_t& operator[] (const Eigen::Vector2i& coord);
    void update_pixel(const Eigen::Vector2i& coord, UpdateState state);

    void init_publisher(ros::NodeHandle& n, const std::string& topic);
    void publish();

    bool convert_to_occ_coords(double x_world, double y_world, Eigen::Vector2i& point_occ);

private:
    float log_odds(float prob);
    float prob(float log_odds);

    ros::Publisher m_occ_publisher;
    int m_cells_x, m_cells_y;
    float m_resolution, m_width, m_height;
    Eigen::Vector3f m_origin;
    nav_msgs::OccupancyGrid m_msg;
    std::map<UpdateState, float>& m_update_values;

    bool is_init;
};
