#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
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
#define PUBLISH_POSE
#define PUBLISH_ERROR

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
#ifdef PUBLISH_ERROR
        d_error_pub = nh->advertise<std_msgs::Float32>("/d_error", 1, this);
        th_error_pub = nh->advertise<std_msgs::Float32>("/th_error", 1, this);
#endif
#ifdef PUBLISH_MARKER
        ekf_pub = nh->advertise<visualization_msgs::Marker>("/ekf_result", 1, this);
        ekf_conf_pub = nh->advertise<visualization_msgs::Marker>("/ekf_result_conf", 1, this);
        initMarker();
#endif
#ifdef PUBLISH_POSE
        pose_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/ekf_pose", 1, this);
#endif
    }
    virtual ~EKFNode(){}
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
#ifdef PUBLISH_ERROR
    ros::Publisher d_error_pub;
    ros::Publisher th_error_pub;
#endif

#ifdef PUBLISH_POSE
    ros::Publisher pose_pub;

    void publishPose(){
        geometry_msgs::Point point;
        point.x = ekf.x(0);
        point.y = ekf.x(1);

        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.pose.pose.position = point;
        msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ekf.x(2));
        msg.pose.covariance[0] = ekf.P(0,0);
        msg.pose.covariance[1] = ekf.P(0,1);
        msg.pose.covariance[5] = ekf.P(0,2);
        msg.pose.covariance[6] = ekf.P(1,0);
        msg.pose.covariance[7] = ekf.P(1,1);
        msg.pose.covariance[11] = ekf.P(1,2);
        msg.pose.covariance[30] = ekf.P(2,0);
        msg.pose.covariance[31] = ekf.P(2,1);
        msg.pose.covariance[35] = ekf.P(2,2);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";
        pose_pub.publish(msg);
    }
#endif

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
        trajectory.color.a = 0.5;
        trajectory.scale.x = 0.03;
        trajectory.scale.y = 0.03;

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
        R_odom *= 3.0;

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
            //ekf_conf_pub.publish(confidence);
#endif
#ifdef PUBLISH_POSE
            publishPose();
#endif
        }
    }

#ifdef USE_IPS
    void ipsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        if (ekf.state == KFStates::RUN)
        {
            y_ips << msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation);
            Eigen::Vector3d x = ekf.x - y_ips;

#ifdef PUBLISH_ERROR
            std_msgs::Float32 d_err;
            std_msgs::Float32 th_err;
            d_err.data = sqrt(x(0)*x(0) + x(1)*x(1));
            th_err.data = abs(x(2));
            ROS_INFO("D_ERR: %e", d_err.data);
            ROS_INFO("TH_ERR: %e", th_err.data);
            d_error_pub.publish(d_err);
            th_error_pub.publish(th_err);
#endif

            y_ips(0) += distribution(generator);
            y_ips(1) += distribution(generator);
            // Don't degrade the yaw?
            R_ips << 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.001;
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
