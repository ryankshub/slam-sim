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
#include "turtlelib/diff_drive.hpp"
//C++ includes
#include <cstdint>
#include <vector>
//3rd-party includes
#include "geometry_msgs/TransformStamped.h"
#include "nusim/Teleport.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "nuturtlebot_msgs/WheelCommands.h"
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
static const double WALL_THICKNESS = 0.1;
static const double WALL_HEIGHT = 0.25;

// Nusim Node's variables
static std::uint64_t timestep = 0;
static int rate;
static double x_init;
static double y_init;
static double theta_init;
static double encoder_ticks_to_rad;
static turtlelib::DiffDrive diff_drive;
static nuturtlebot_msgs::SensorData sensor_data;
static std::vector<double> obs_x;
static std::vector<double> obs_y;
static double obs_radius;
static double x_length;
static double y_length; 

//Nusim Node's callbacks

/// \brief Callback fcn for reset service. Reset simulation
///
/// \param req - Empty Request
/// \param res - Empty Response 
/// \returns true if service completes successfully
bool reset(std_srvs::Empty::Request &,
           std_srvs::Empty::Response &)
{
    timestep = 0;
    diff_drive.set_configuration(theta_init, x_init, y_init);
    return true;
}

/// \brief Callback fcn teleport service. Teleports robot
///
/// \param req - Teleport Request
/// \param res - Teleport Response 
/// \returns true if service completes successfully
bool teleport(nusim::Teleport::Request &req,
              nusim::Teleport::Response &)
{
    diff_drive.set_configuration(req.theta, req.x, req.y);
    return true;
}

/// \brief Callback fcn to wheel_cmd subscriber. Grabs the current
/// wheel velocity commands in ticks and updates the internal position
/// of the robots and encoder angles
void wheel_cmd_handler(const nuturtlebot_msgs::WheelCommands& msg)
{
    sensor_data.left_encoder += msg.left_velocity;
    sensor_data.right_encoder += msg.right_velocity;
    double new_left_wheel_rad = sensor_data.left_encoder*encoder_ticks_to_rad;
    double new_right_wheel_rad = sensor_data.left_encoder*encoder_ticks_to_rad;
    diff_drive.apply_fw_kin(new_left_wheel_rad, new_right_wheel_rad);

}

//Nusim Main Function
int main(int argc, char *argv[])
{
    //Setup node
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~"); //Private scoping node handler
    ros::NodeHandle pub_nh;
    tf2_ros::TransformBroadcaster br; 

    double wheel_radius = 0.0;
    double track_width = 0.0;

    //Get Params
    nh.param("rate", rate, DEFAULT_RATE);
    nh.param("x0", x_init, DEFAULT_X);
    nh.param("y0", y_init, DEFAULT_Y);
    nh.param("theta0", theta_init, DEFAULT_THETA);
    nh.param("obstacles/obs_x", obs_x, DEFAULT_OBS_LIST);
    nh.param("obstacles/obs_y", obs_y, DEFAULT_OBS_LIST);
    nh.param("obstacles/radius", obs_radius, DEFAULT_RADIUS);
    
    //Get Arena Param
    if(!nh.getParam("x_length", x_length))
    {
        ROS_ERROR_STREAM("Cannot find x_length for arena");
        return(1);
    }

    if(!nh.getParam("y_length", y_length))
    {
        ROS_ERROR_STREAM("Cannot find y_length for arena");
        return(1);
    }

    //Get DiffDrive params
    if (!pub_nh.getParam("red/wheel_radius", wheel_radius))
    {
        ROS_ERROR_STREAM("Cannot find wheel_radius"); 
        return(1); //return 1 to indicate error
    }

    if (!pub_nh.getParam("red/track_width", track_width))
    {
        ROS_ERROR_STREAM("Cannot find track_width"); 
        return(1); //return 1 to indicate error
    }

    if (!pub_nh.getParam("red/encoder_ticks_to_rad", encoder_ticks_to_rad))
    {
        ROS_ERROR_STREAM("Cannot find encoder_ticks_to_rad"); 
        return(1); //return 1 to indicate error
    }

    //Build ROS Objects
    const auto timestep_pub = nh.advertise<std_msgs::UInt64>("timestep", QUEUE_SIZE);
    const auto encoder_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", QUEUE_SIZE);
    const auto cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", QUEUE_SIZE, true);
    const auto walls_pub = nh.advertise<visualization_msgs::MarkerArray>("walls", QUEUE_SIZE, true); 
    const auto wheel_cmd_sub = nh.subscribe("red/wheel_cmd", QUEUE_SIZE, wheel_cmd_handler);
    const auto reset_srv = nh.advertiseService("reset", reset);
    const auto teleport_srv = nh.advertiseService("teleport", teleport);
    geometry_msgs::TransformStamped ts;

    //Set up Rate object
    ros::Rate loop_rate(rate);

    //Init diff drive
    diff_drive.set_wheel_config(track_width, wheel_radius);
    diff_drive.set_configuration(theta_init, x_init, y_init);

    //Publish Obstacle Markers
    visualization_msgs::MarkerArray cylinders;
    if (obs_radius > 0.0 && obs_x.size() > 0.0 && obs_x.size() == obs_y.size())
    {
        for(long unsigned int i=0;i < obs_x.size(); i++)
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


    //Publish Wall Markers
    visualization_msgs::MarkerArray walls;        
    //Front Wall
    visualization_msgs::Marker front_wall;
    front_wall.header.stamp = ros::Time::now();
    front_wall.header.frame_id = "world";
    front_wall.ns = "walls";
    front_wall.id = 0;
    front_wall.type = visualization_msgs::Marker::CUBE;
    front_wall.action = visualization_msgs::Marker::ADD;
    front_wall.pose.position.x = x_length/2.0 + WALL_THICKNESS/2.0;
    front_wall.pose.position.y = 0;
    front_wall.pose.position.z = WALL_HEIGHT/2.0;
    front_wall.pose.orientation.x = 0.0;
    front_wall.pose.orientation.y = 0.0;
    front_wall.pose.orientation.z = 0.0;
    front_wall.pose.orientation.w = 1.0;
    front_wall.scale.x = WALL_THICKNESS;
    front_wall.scale.y = y_length + 2.0*WALL_THICKNESS;
    front_wall.scale.z = WALL_HEIGHT;
    front_wall.color.r = 1.0;
    front_wall.color.g = 0.0;
    front_wall.color.b = 0.0;
    front_wall.color.a = 1.0;
    walls.markers.push_back(front_wall);

    //Back Wall
    visualization_msgs::Marker back_wall;
    back_wall.header.stamp = ros::Time::now();
    back_wall.header.frame_id = "world";
    back_wall.ns = "walls";
    back_wall.id = 1;
    back_wall.type = visualization_msgs::Marker::CUBE;
    back_wall.action = visualization_msgs::Marker::ADD;
    back_wall.pose.position.x = -x_length/2.0 - WALL_THICKNESS/2.0;
    back_wall.pose.position.y = 0;
    back_wall.pose.position.z = WALL_HEIGHT/2.0;
    back_wall.pose.orientation.x = 0.0;
    back_wall.pose.orientation.y = 0.0;
    back_wall.pose.orientation.z = 0.0;
    back_wall.pose.orientation.w = 1.0;
    back_wall.scale.x = WALL_THICKNESS;
    back_wall.scale.y = y_length + 2.0*WALL_THICKNESS;
    back_wall.scale.z = WALL_HEIGHT;
    back_wall.color.r = 1.0;
    back_wall.color.g = 0.0;
    back_wall.color.b = 0.0;
    back_wall.color.a = 1.0;
    walls.markers.push_back(back_wall);

    //Left Wall
    visualization_msgs::Marker left_wall;
    left_wall.header.stamp = ros::Time::now();
    left_wall.header.frame_id = "world";
    left_wall.ns = "walls";
    left_wall.id = 2;
    left_wall.type = visualization_msgs::Marker::CUBE;
    left_wall.action = visualization_msgs::Marker::ADD;
    left_wall.pose.position.x = 0;
    left_wall.pose.position.y = -y_length/2.0 - WALL_THICKNESS/2.0;
    left_wall.pose.position.z = WALL_HEIGHT/2.0;
    left_wall.pose.orientation.x = 0.0;
    left_wall.pose.orientation.y = 0.0;
    left_wall.pose.orientation.z = 0.0;
    left_wall.pose.orientation.w = 1.0;
    left_wall.scale.x = x_length + 2.0*WALL_THICKNESS;
    left_wall.scale.y = WALL_THICKNESS;
    left_wall.scale.z = WALL_HEIGHT;
    left_wall.color.r = 1.0;
    left_wall.color.g = 0.0;
    left_wall.color.b = 0.0;
    left_wall.color.a = 1.0;
    walls.markers.push_back(left_wall);

    //Right Wall
    visualization_msgs::Marker right_wall;
    right_wall.header.stamp = ros::Time::now();
    right_wall.header.frame_id = "world";
    right_wall.ns = "walls";
    right_wall.id = 3;
    right_wall.type = visualization_msgs::Marker::CUBE;
    right_wall.action = visualization_msgs::Marker::ADD;
    right_wall.pose.position.x = 0;
    right_wall.pose.position.y = y_length/2.0 + WALL_THICKNESS/2.0;
    right_wall.pose.position.z = WALL_HEIGHT/2.0;
    right_wall.pose.orientation.x = 0.0;
    right_wall.pose.orientation.y = 0.0;
    right_wall.pose.orientation.z = 0.0;
    right_wall.pose.orientation.w = 1.0;
    right_wall.scale.x = x_length + 2.0*WALL_THICKNESS;
    right_wall.scale.y = WALL_THICKNESS;
    right_wall.scale.z = WALL_HEIGHT;
    right_wall.color.r = 1.0;
    right_wall.color.g = 0.0;
    right_wall.color.b = 0.0;
    right_wall.color.a = 1.0;
    walls.markers.push_back(right_wall);

    //Pub Walls
    walls_pub.publish(walls);

    //Main loop
    while(ros::ok())
    {
        //Update data
        timestep++; //this means we start at T = 1

        //Populate transform
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "red-base_footprint";
        ts.transform.translation.x = diff_drive.location().x;
        ts.transform.translation.y = diff_drive.location().y;
        ts.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, diff_drive.theta());
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        br.sendTransform(ts);

        //Package and send data
        std_msgs::UInt64 msg;
        msg.data = timestep;
        timestep_pub.publish(msg);
        encoder_pub.publish(sensor_data);

        //Grab data
        ros::spinOnce();

        //Sleep 
        loop_rate.sleep();
        
    }

    return(0);

}
