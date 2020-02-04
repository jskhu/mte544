//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>


//Callback function for the Position topic
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

//Velocity control variable
geometry_msgs::Twist vel;
double distance = 0.25;
double prevYaw = 0;
double prevX = 0;
double prevY = 0;
double linX = 0.2;
double angZ = 0.2;

enum STATE {
    NOT_MOVING,
    STRAIGHT,
    TURN
};
STATE state = NOT_MOVING;

double get_distance(const double x1, const double x2, const double y1, const double y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	double X = msg->pose.pose.position.x; // Robot X psotition
	double Y = msg->pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

    if (state == NOT_MOVING)
    {
        prevX = X;
        prevY = Y;
        prevYaw = Yaw;
        vel.linear.x = linX;
        state = STRAIGHT;
    }
    else if (state == STRAIGHT)
    {
        double curDist = get_distance(X, prevX, Y, prevY);
        if (curDist >= distance)
        {
            prevYaw = Yaw;
            vel.linear.x = 0;
            vel.angular.z = angZ;
            state = TURN;
        }
    }
    else
    {
        if (fmod((Yaw - prevYaw + M_PI * 2), M_PI * 2) >= M_PI_2 - 0.1)
        {
            prevX = X;
            prevY = Y;
            vel.linear.x = linX;
            vel.angular.z = 0;
            state = STRAIGHT;
        }
    }
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

    	//Main loop code goes here:
    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
