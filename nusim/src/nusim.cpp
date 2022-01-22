/// \file nusim
/// \brief The Nusim node maintains the state of the simulation
///
/// PARAMETERS:
///   private (parameter scoped to node name):
///     rate (int): The running rate of the node (Hz)
///     x0 (double): Initial x position of the robot (m)
///     y0 (double): Initial y position of the robot (m)
///     theta0 (double): Initial theta of the robot (rad)
///     obstacles/obs_x: List of x position for cylinder obstacles (m)
///     obstacles/obs_y: List of y position for cylinder obstacles (m)
///     obstacles/radius: Radius of the cylinder obstacles (m)
///     parameter_name (parameter_type): description of the parameter
///     
/// PUBLISHES:
///     nusim/timestep (std_msgs::UInt64): The timestep of the simulation
///     nusim/obstacles (visualization_msgs::MarkerArray): The list of cylinder Marker obstacles to 
///         display in rviz
///     red/jointes (sensor_msgs::JointState): The positions of the wheels on the robot. 
///         Currently set to 0.
/// BROADCASTER:
///     broadcaster (non-static world to red-base_footprint): Transformation matrix from world frame
///         to red-base_footprint frame
/// SERVICES:
///     reset (std_srvs::Empty): Call this service to reset the simulation's timestep and
///         the position of the robot back to x0, y0, theta0
///     teleport (nusim::Teleport): Given a x, y, and theta, this service will teleport 
///         the robot to desired pose

//RKS

//Project include

//C++ includes
#include <cstdint>
#include <vector>
//3rd-party includes
#include "geometry_msgs/TransformStamped.h"
#include "nusim/Teleport.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/UInt64.h"
#include "std_srvs/Empty.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


//Nusim's Constants
static const int DEFAULT_RATE = 500;
static const std::uint32_t QUEUE_SIZE = 1000;
static const double DEFAULT_X = 0.0;
static const double DEFAULT_Y = 0.0;
static const double DEFAULT_THETA = 0.0;
static const double DEFAULT_RADIUS = 0.0;
static const std::vector<double> DEFAULT_OBS_LIST;
static const double CYLINDER_HEIGHT = 0.25;

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
static std::vector<double> obs_x;
static std::vector<double> obs_y;
static double obs_radius; 

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

bool teleport(nusim::Teleport::Request &req,
              nusim::Teleport::Response &res)
{
    x = req.x;
    y = req.y;
    theta = req.theta;
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
    nh.param("obstacles/obs_x", obs_x, DEFAULT_OBS_LIST);
    nh.param("obstacles/obs_y", obs_y, DEFAULT_OBS_LIST);
    nh.param("obstacles/radius", obs_radius, DEFAULT_RADIUS);

    //Build ROS Objects
    const auto timestep_pub = nh.advertise<std_msgs::UInt64>("timestep", QUEUE_SIZE);
    const auto joint_state_pub = pub_nh.advertise<sensor_msgs::JointState>("red/joint_states", QUEUE_SIZE);
    const auto cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", QUEUE_SIZE, true);
    const auto reset_srv = nh.advertiseService("reset", reset);
    const auto teleport_srv = nh.advertiseService("teleport", teleport);
    geometry_msgs::TransformStamped ts;

    //Set up Rate object
    ros::Rate loop_rate(rate);

    //Set up joint states(they don't change here)
    joint_states.name.push_back("red-wheel_left_joint");
    joint_states.name.push_back("red-wheel_right_joint");
    joint_states.position.push_back(0.0);
    joint_states.position.push_back(0.0);

    //Publish Markers
    visualization_msgs::MarkerArray cylinders;
    if (obs_radius > 0.0 && obs_x.size() > 0.0 && obs_x.size() == obs_y.size())
    {
        for(int i=0;i < obs_x.size(); i++)
        {
            visualization_msgs::Marker cylin;
            cylin.header.stamp = ros::Time::now();
            cylin.header.frame_id = "world";
            cylin.ns = "obstacles";
            cylin.id = i;
            cylin.type = visualization_msgs::Marker::CYLINDER;
            cylin.action = visualization_msgs::Marker::ADD;
            cylin.pose.position.x = obs_x[i];
            cylin.pose.position.y = obs_y[i];
            cylin.pose.position.z = CYLINDER_HEIGHT/2.0;
            cylin.pose.orientation.x = 0.0;
            cylin.pose.orientation.y = 0.0;
            cylin.pose.orientation.z = 0.0;
            cylin.pose.orientation.w = 1.0;
            cylin.scale.x = obs_radius;
            cylin.scale.y = obs_radius;
            cylin.scale.z = CYLINDER_HEIGHT;
            cylin.color.r = 1.0;
            cylin.color.g = 0.0;
            cylin.color.b = 0.0;
            cylin.color.a = 1.0;

            cylinders.markers.push_back(cylin);
        }
    }

    cylinder_pub.publish(cylinders);

    //Main loop
    while(ros::ok())
    {
        //Update data
        timestep++; //this means we start at T = 1
        joint_states.header.stamp = ros::Time::now();

        //Populate transform
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "red-base_footprint";
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
