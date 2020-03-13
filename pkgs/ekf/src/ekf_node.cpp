#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
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
1. What is Q? Identity is too large, but zeros is too low??
2. Handle full rotation of yaw (modulo 2pi?)
========== */

#define USE_IPS
#define PUBLISH_MARKER

class EKFNode
{
  public:
    EKFNode(ros::NodeHandle *nh) : have_ips(false),
                                   distribution(0.0, 0.10),
                                   fmt(4, 0, ", ", "\n", "[", "]")
    {
        odom_sub = nh->subscribe("/odom", 1, &EKFNode::odomCallback, this);
        teleop_sub= nh->subscribe("/cmd_vel_mux/input/teleop", 1, &EKFNode::teleopCallback, this);
#ifdef USE_IPS
        ips_sub = nh->subscribe("/fixed_ips", 1, &EKFNode::ipsCallback, this);
#endif
#ifdef PUBLISH_MARKER
        ekf_pub = nh->advertise<visualization_msgs::Marker>("/ekf_result", 1, this);
        ekf_conf_pub = nh->advertise<visualization_msgs::Marker>("/ekf_result_conf", 1, this);
        initMarker();
#endif
    }
  private:
    ros::Subscriber odom_sub;
    ros::Subscriber teleop_sub;
#ifdef USE_IPS
    ros::Subscriber ips_sub;
#endif
    KalmanFilter ekf;

    Eigen::IOFormat fmt;

    Eigen::Vector3d y_odom;
    Eigen::Matrix3d R_odom;

    Eigen::Vector3d y_ips;
    Eigen::Matrix3d R_ips;

    bool have_ips;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

#ifdef PUBLISH_MARKER
    ros::Publisher ekf_pub;
    visualization_msgs::Marker trajectory;

    ros::Publisher ekf_conf_pub;
    visualization_msgs::Marker confidence;

    void initMarker()
    {
        trajectory.header.frame_id = "odom";
        trajectory.ns = "ekf_trajectory";
        trajectory.id = 0;
        trajectory.type = visualization_msgs::Marker::POINTS;
        trajectory.color.r = 1.0f;
        trajectory.color.g = 0.0f;
        trajectory.color.b = 0.0f;
        trajectory.color.a = 0.8;
        trajectory.scale.x = 0.01;
        trajectory.scale.y = 0.01;

        confidence.header.frame_id = "odom";
        confidence.ns = "ekf_confidence";
        confidence.id = 0;
        confidence.type = visualization_msgs::Marker::CYLINDER;
        confidence.color.r = 0.0f;
        confidence.color.g = 0.5f;
        confidence.color.b = 0.5f;
        confidence.color.a = 0.4;
    }

    void updateMarker()
    {
        trajectory.header.stamp = ros::Time::now();
        geometry_msgs::Point p;
        p.x = ekf.x(0);
        p.y = ekf.x(1);
        trajectory.points.push_back(p);

        confidence.header.stamp = ros::Time::now();
        confidence.pose.position.x = ekf.x(0);
        confidence.pose.position.y = ekf.x(1);
        confidence.scale.x = 2*sqrt(ekf.P(0,0));
        confidence.scale.y = 2*sqrt(ekf.P(1,1));
    }
#endif

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
        R_odom *= 2.0;

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
#ifdef PUBLISH_MARKER
            updateMarker();
            ekf_pub.publish(trajectory);
            ekf_conf_pub.publish(confidence);
#endif
        }
    }

#ifdef USE_IPS
    void ipsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        if (ekf.state == KFStates::RUN)
        {
            y_ips << msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation);
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
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;
    EKFNode node(&nh);

    ros::spin();

    return 0;
}
