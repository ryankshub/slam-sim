/// \file odometry
/// \brief Odometry publishes the odometry messages and odometry transform
///
/// PARAMETERS:
///   track_width (double) : distance between the robot's wheels 
///   body_id (string): Name of the body frame of the robot (Required)
///   odom_id (string): Name of the odometry frame (Defaulted to `odom`)
///   wheel_left (string): Name of the left wheel joint (Required)
///   wheel_right (string): Name of the right wheel joint (Required)
///    
/// PUBLISHES:
///
/// SUBSCRIBERS:
///


//RKS

//Project include
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

//C++ includes
#include <vector>
#include <string>

//3rd-party includes
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "nuturtle_control/PoseConfig.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

//Odometry's Constants
static const int DEFAULT_RATE = 500;
static const std::string DEFAULT_ODOM = "odom";
static const std::uint32_t QUEUE_SIZE = 1000;


//Odometry's Variables
static turtlelib::DiffDrive Dodom;
static turtlelib::Twist2D body_twist;
static std::string wheel_left = "";
static std::string wheel_right = "";

//Odometry's Callbacks

/// \brief JointState subscriber callback fcn. Given the joint states
/// update the robot's configuation and body_twist
/// \param msg - contains joint state information
void state_handler(const sensor_msgs::JointState& msg)
{
    double left_wheel = Dodom.left_wheel_pos();
    double right_wheel = Dodom.right_wheel_pos();
    //Parse updates
    for (unsigned int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name.at(i) == wheel_left)
        {
            left_wheel += msg.velocity.at(i);
        } else if (msg.name.at(i) == wheel_right) {
            right_wheel += msg.velocity.at(i);
        }
    }
    //Update body_twist
    body_twist = Dodom.cal_fw_kin(left_wheel, right_wheel);
    //Update configuration
    Dodom.apply_fw_kin(left_wheel, right_wheel);
}

/// \brief set_pose service callback fcn. Teleports the robot 
/// to a given position. (Does not reset the twist)
bool pose_service(nuturtle_control::PoseConfig::Request &req,
                  nuturtle_control::PoseConfig::Response &)
{
    Dodom.set_configuration(req.theta, req.x, req.y);
    return true;
}

//Odometry Main Loop
int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;

    //Get Params
    std::string body_id = "";
    std::string odom_id = "";
    double wheel_radius = 0.0;
    double track_width = 0.0;

    if (!nh.getParam("wheel_radius", wheel_radius))
    {
        ROS_ERROR_STREAM("Cannot find wheel_radius"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("track_width", track_width))
    {
        ROS_ERROR_STREAM("Cannot find track_width"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("body_id", body_id))
    {
        ROS_ERROR_STREAM("Cannot find body_id"); 
        return(1); //return 1 to indicate error
    }  

    nh.param("odom_id", odom_id, DEFAULT_ODOM);

    if (!nh.getParam("wheel_left", wheel_left))
    {
        ROS_ERROR_STREAM("Cannot find wheel_left"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("wheel_right", wheel_right))
    {
        ROS_ERROR_STREAM("Cannot find wheel_right"); 
        return(1); //return 1 to indicate error
    }

    //ROS Objects
    const auto state_sub = nh.subscribe("joint_state", QUEUE_SIZE, state_handler);
    const auto odom_pub = nh.advertise<nav_msgs::Odometry>("odom", QUEUE_SIZE);
    const auto pose_srv = nh.advertiseService("set_pose", pose_service);
    geometry_msgs::TransformStamped Tob;

    //Init DiffDrive object
    Dodom.set_wheel_config(track_width, wheel_radius);
    
    //Set up Rate object
    ros::Rate loop_rate(DEFAULT_RATE);

    //Setup nav_odom object
    nav_msgs::Odometry nav_odom;
    nav_odom.header.frame_id = odom_id;
    nav_odom.child_frame_id = body_id;

    //Setup Transform
    Tob.header.frame_id = odom_id;
    Tob.child_frame_id = body_id;
    //Main Loop
    while(ros::ok())
    {
        //Grab data
        ros::spinOnce();

        // Broadcast Transform
        Tob.header.stamp = ros::Time::now();
        Tob.transform.translation.x = Dodom.location().x; 
        Tob.transform.translation.y = Dodom.location().y;
        Tob.transform.translation.z =  0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, Dodom.theta());
        Tob.transform.rotation.x = q.x();
        Tob.transform.rotation.y = q.y(); 
        Tob.transform.rotation.z = q.z(); 
        Tob.transform.rotation.w = q.w();

        br.sendTransform(Tob);

        // Publish odom
        nav_odom.header.stamp = ros::Time::now();
        // Provide pose
        nav_odom.pose.pose.position.x = Dodom.location().x; 
        nav_odom.pose.pose.position.y = Dodom.location().y; 
        nav_odom.pose.pose.position.z = 0.0;
        nav_odom.pose.pose.orientation.x = q.x();
        nav_odom.pose.pose.orientation.y = q.y();
        nav_odom.pose.pose.orientation.z = q.z(); 
        nav_odom.pose.pose.orientation.w = q.w();
        // Provide twist
        nav_odom.twist.twist.linear.x = body_twist.x_dot;
        nav_odom.twist.twist.linear.y = body_twist.y_dot;
        nav_odom.twist.twist.angular.z = body_twist.theta_dot;

        odom_pub.publish(nav_odom);

        //Sleep
        loop_rate.sleep();
    }
}