/// \file odometry
/// \brief Odometry publishes the odometry messages and odometry transform
///
/// PARAMETERS:
///   private (parameter scoped to node name):
///     body_id (string): Name of the body frame of the robot (Required)
///     odom_id (string): Name of the odometry frame (Defaulted to `odom`)
///     wheel_left (string): Name of the left wheel joint (Required)
///     wheel_right (string): Name of the right wheel joint (Required)
///    
/// PUBLISHES:
///
/// SUBSCRIBERS:
///


//RKS

//Project include
#include "turtlelib/diff_drive.hpp"

//C++ includes
#include <vector>
#include <string>

//3rd-party includes
#include "nav_msgs/Odometry.h"
#include "nuturtle_control/PoseConfig.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"

//Odometry's Constants
const static std::string DEFAULT_ODOM = "odom";
static const std::uint32_t QUEUE_SIZE = 1000;

//Odometry's Callbacks
void state_handler(const sensor_msgs::JointState& msg)
{
    //Stuff
}

bool pose_service(nuturtle_control::PoseConfig::Request &req,
                  nuturtle_control::PoseConfig::Response &)
{
    return true;
}

//Odometry Main Loop
int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;

    //Get Params
    std::string body_id = "";
    std::string odom_id = "";
    std::string wheel_left = "";
    std::string wheel_right = "";
    // Get Params
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
}