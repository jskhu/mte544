
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// Function to get a measurement
// Function to update mu -> This is the EKF algorithm
Eigen::Vector3d mu;
Eigen::Vector2d u;
Eigen::Matrix3d S;
double dt;
Eigen::Matrix3d Q, R;

void pose_callback(const geometry_msgs::PoseStamped& msg) {
    // Process measurement
    double roll, pitch, yaw;

    tf::Quaternion quat(msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Eigen::Vector3d y;
    y << msg.pose.position.x, msg.pose.position.y, yaw;

    // PREDICTION UPDATE
    // Create G matrix
    Eigen::Matrix3d G;

    G << 1, 0, -dt * u[0] * sin(mu[2]),
         0, 1,  dt * u[0] * cos(mu[2]),
         0, 0, 1;

    // Set up initial prediction
    Eigen::Vector3d mup;
    Eigen::Matrix3d Sp;
    mup = mu + dt * Eigen::Vector3d(u[0]*cos(mu[2]), u[0]*cos(mu[2]), u[1]);
    Sp = G * S * G.transpose() + R;

    // MEASUREMENT UPDATE
    Eigen::Matrix3d K = Sp * (Sp + Q).inverse();

    mu = mup + K * (y - mup);
    S = (Eigen::Matrix3d::Identity() - K) * Sp;

    ROS_INFO_STREAM(mu);
}

void odom_callback(const nav_msgs::Odometry& msg) {
    u[0] = msg.twist.twist.linear.x;
    u[1] = msg.twist.twist.angular.z;
}

// What can we modify? Q, R, dt
int main(int argc, char **argv) {
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ros::Subscriber pose_sub = n.subscribe("/noisy_pose", 1, pose_callback);
    ros::Subscriber odometry_sub = n.subscribe("/odom", 1, odom_callback);

    // Set node params
    n.param("dt", dt, 0.1);
    // TODO: make these dynamic???
    Q << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    R << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    // Inital estimates of mu and S
    mu << 0, 0, 0;
    S << 4, 0, 0,
         0, 4, 0,
         0, 0, 4;

    ros::spin();
    return 0;
}