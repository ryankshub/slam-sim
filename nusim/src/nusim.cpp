/// \file nusim
/// \brief The Nusim node maintains the state of the simulation
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service
//RKS

//Project include

//C++ includes
#include <cstdint>
//3rd-party includes
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt64.h"
#include "std_srvs/Empty.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


//Nusim's Constants
static const int DEFAULT_RATE = 500;
static const std::uint32_t QUEUE_SIZE = 1000;
static const double DEFAULT_X = 0.0;
static const double DEFAULT_Y = 0.0;
static const double DEFAULT_THETA = 0.0;

// Nusim Node's variables
static std::uint64_t timestep = 0;
static int rate;
static sensor_msgs::JointState joint_states;
static double x;
static double x_init;
static double y;
static double y_init;
static double theta;
static double theta_init;

//Nusim Node's callbacks
bool reset(std_srvs::Empty::Request &req,
           std_srvs::Empty::Response &res)
{
    timestep = 0;
    x = x_init;
    y = y_init;
    theta = theta_init;
    return true;
}

//Nusim Main Function
int main(int argc, char *argv[])
{
    //Setup node
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~"); //Private scoping node handler
    ros::NodeHandle pub_nh;
    tf2_ros::TransformBroadcaster br; 

    //Get Params
    nh.param("rate", rate, DEFAULT_RATE);
    nh.param("x0", x, DEFAULT_X);
    nh.param("y0", y, DEFAULT_Y);
    nh.param("theta0", theta, DEFAULT_THETA);
    x_init = x;
    y_init = y;
    theta_init = theta;

    //Build ROS Objects
    const auto timestep_pub = nh.advertise<std_msgs::UInt64>("timestep", QUEUE_SIZE);
    const auto joint_state_pub = pub_nh.advertise<sensor_msgs::JointState>("red/joint_states", QUEUE_SIZE);
    const auto reset_srv = nh.advertiseService("reset", reset);
    geometry_msgs::TransformStamped ts;

    //Set up Rate object
    ros::Rate loop_rate(rate);

    //Set up joint states(they don't change here)
    joint_states.name.push_back("red-wheel_left_joint");
    joint_states.name.push_back("red-wheel_right_joint");
    joint_states.position.push_back(0.0);
    joint_states.position.push_back(0.0);
    joint_states.header.frame_id = "base_footprint";

    //Main loop
    while(ros::ok())
    {
        //Update data
        timestep++; //this means we start at T = 1

        //Populate transform
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "red:base_footprint";
        ts.transform.translation.x = x;
        ts.transform.translation.y = y;
        ts.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        br.sendTransform(ts);

        //Package and send data
        std_msgs::UInt64 msg;
        msg.data = timestep;
        timestep_pub.publish(msg);
        joint_state_pub.publish(joint_states);

        //Grab data
        ros::spinOnce();

        //Sleep 
        loop_rate.sleep();
        
    }

    return(0);

}
