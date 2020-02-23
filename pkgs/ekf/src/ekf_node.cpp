#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

#include <ekf/kalman_filter.h>

/**
 * Turtlebot EKF localization node
 *
 * Uses odometry, ips, and teleop for localization
 *
 * Note: Frame of reference is initial odometry position
 *       The direction of turtlebot is +x (0 yaw)
 */


/*======== TODO:
1. Publish pose with header (with odom as frame of reference)
2. What is Q? Identity is too large, but zeros is too low??
3. Handle full rotation of yaw (modulo 2pi?)

========== */

#define USE_IPS
//#define PUBLISH_EKF

KalmanFilter ekf;
Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");

Eigen::Vector3d y_odom;
Eigen::Matrix3d R_odom;

Eigen::Vector3d y_ips;
Eigen::Matrix3d R_ips;

bool have_ips = false;
bool have_transform = false;
Eigen::Vector3d trans;
Eigen::Matrix3d rot;

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.0, 0.10);

int updateTimeStamp(ros::Time now)
{
    static int curr_time = 0; // in milli
    int temp = curr_time;
    curr_time = (now.sec % 1000) * 1000 + (now.nsec / 1000000);
    int diff = curr_time - temp;
    return (diff < 0) ? diff + 1000000 : diff;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    y_odom << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;

    R_odom << msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[5],
         msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[11],
         msg->pose.covariance[30], msg->pose.covariance[31], msg->pose.covariance[35];

    if (ekf.state == KFStates::START)
    {
        updateTimeStamp(ros::Time::now());
        ekf.init(y_odom, R_odom);
    }
    else
    {
      //std::cout << "ERROR" << std::endl;
      //std::cout << (ekf.x - y_odom).format(fmt) << std::endl;
    }
}

void teleopCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    if (ekf.state == KFStates::RUN)
    {
        float dt = updateTimeStamp(ros::Time::now())/1000.0; // in seconds
        Eigen::Vector2d u;
        u << msg->linear.x, msg->angular.z;
        ekf.predict(dt, u);
        if (have_ips)
        {
            ekf.update(y_ips, R_ips);
            have_ips = false;
        }
        else
            ekf.update(y_odom, R_odom);
    }
}

#ifdef USE_IPS
void ipsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (ekf.state == KFStates::RUN)
    {
        double yaw = tf::getYaw(msg->pose.pose.orientation);

        y_ips << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;
        if (!have_transform)
        {
            double yaw_diff = y_odom(2) - yaw;
            rot << cos(yaw_diff), -sin(yaw_diff), 0,
                   sin(yaw_diff), cos(yaw_diff), 0,
                   0, 0, 1;
            y_ips = rot * y_ips;
            trans = y_odom - y_ips;
            y_ips += trans;
            have_transform = true;
        }
        else
        {
            y_ips = rot * y_ips;
            y_ips += trans;
        }
        std::cout << "ERROR" << std::endl;
        std::cout << (ekf.x - y_ips).format(fmt) << std::endl;
        y_ips(0) += distribution(generator);
        y_ips(1) += distribution(generator);
        // Don't degrade the yaw?
        R_ips << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.005;
        have_ips = true;
    }
}
#endif

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    ros::Subscriber teleop_sub = nh.subscribe("/cmd_vel_mux/input/teleop", 1, teleopCallback);
#ifdef PUBLISH_EKF
    ros::Publisher ekf_pub = nh.advertise<geometry_msgs::PoseWithCovariance>("/ekf_result", 1);
#endif
#ifdef USE_IPS
    ros::Subscriber ips_sub = nh.subscribe("/indoor_pos", 1, ipsCallback);
#endif

    ros::spin();

    return 0;
}
