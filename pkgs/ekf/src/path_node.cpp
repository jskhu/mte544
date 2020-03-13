#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

/**
 * Turtlebot path node
 *
 * Uses odometry and ips to determine the robot path
 *
 * Note: Frame of reference is initial odometry position
 */

#define PUBLISH_PATH

class FixedIPS
{
  private:
    ros::Publisher fixed_ips_pub;
#ifdef PUBLISH_PATH
    ros::Publisher path_pub;
#endif
    ros::Subscriber ips_sub;
    ros::Subscriber odom_sub;

    std::string frame_id;
    tf::Pose odom_t_robot;
    tf::Transform odom_t_ips;
    nav_msgs::Path path_msg;

    bool odomReady;
    bool haveTransform;
  public:
    FixedIPS(ros::NodeHandle *nh):odomReady(false), haveTransform(false)
    {
        fixed_ips_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/fixed_ips", 1, this);
#ifdef PUBLISH_PATH
        path_pub = nh->advertise<nav_msgs::Path>("/robot_path", 1, this);
#endif
        odom_sub = nh->subscribe("/odom", 1, &FixedIPS::odomCallback, this);
        ips_sub = nh->subscribe("/indoor_pos", 1, &FixedIPS::ipsCallback, this);
    }

    virtual ~FixedIPS()
    {}

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (!haveTransform)
        {
            odomReady = true;
            tf::poseMsgToTF(msg->pose.pose, odom_t_robot);
            frame_id = msg->header.frame_id;
        }
    }

    void ipsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseWithCovarianceStamped new_msg = *msg;
        new_msg.pose.pose.position.z = 0;
        if (!haveTransform)
        {
            if (odomReady)
            {
                tf::Pose ips_t_robot;
                tf::poseMsgToTF(new_msg.pose.pose, ips_t_robot);
                odom_t_ips = odom_t_robot * ips_t_robot.inverse();
                //odom_t_ips = odom_t_robot.inverseTimes(ips_t_robot);
                haveTransform = true;
            }
        }
        if (haveTransform)
        {
            tf::Pose ips_t_robot_truth;
            tf::poseMsgToTF(new_msg.pose.pose, ips_t_robot_truth);
            //tf::Pose odom_t_robot_truth = ips_t_robot_truth * odom_t_ips;
            tf::Pose odom_t_robot_truth = odom_t_ips * ips_t_robot_truth;
            tf::poseTFToMsg(odom_t_robot_truth, new_msg.pose.pose);
            new_msg.header.frame_id = frame_id;
            fixed_ips_pub.publish(new_msg);
#ifdef PUBLISH_PATH
            new_msg.header.stamp = ros::Time::now();
            path_msg.header = new_msg.header;
            geometry_msgs::PoseStamped ps;
            ps.header = new_msg.header;
            ps.pose = new_msg.pose.pose;
            path_msg.poses.push_back(ps);
            path_pub.publish(path_msg);
#endif
        }
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_node");
    ros::NodeHandle nh;
    FixedIPS fifi = FixedIPS(&nh);

    ros::spin();

    return 0;
}

