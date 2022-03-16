/// \file odometry
/// \brief Odometry publishes the odometry messages and odometry transform
///
/// PARAMETERS:
///   wheel_radius(double, required): radius of the robot's wheels
///   track_width (double, required): distance between the robot's wheels 
///   body_id (string, required): Name of the body frame of the robot (Required)
///   odom_id (string, required): Name of the odometry frame (Defaulted to `odom`)
///   wheel_left (string, required): Name of the left wheel joint (Required)
///   wheel_right (string, required): Name of the right wheel joint (Required)
///   nusim/x0 (double, default 0): Starting x position of the turtlebot 
///     Read from nusim in case position is different from origin
///   nusim/y0 (double, default 0): Starting y position of the turtlebot
///     Read from nusim in case position is different from origin
///   nusim/theta0 (double, default 0): Starting angle position of the turtlebot
///     Read from nusim in case position is different from origin
///
///    
/// PUBLISHES:
///     odom (nav_mags::Odometry): publishes the robot's pose in the odometry frame
///
/// SUBSCRIBERS:
///     joint_states (sensor_msgs::JointState): listens for the encoder position and wheel 
///         angular speeds of the robot
///
/// SERVICES:
///     set_pose (nuturtle_control::PoseConfig): given the x, y, and theta of new configuration,
///         teleport the robot to that position
///
/// BROADCASTER:
///     broadcaster (non-static odom_id to body_id): Transformation matrix from odom_id frame
///         to body_id frame


//RKS

//Project include
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

//C++ includes
#include <vector>
#include <string>

//3rd-party includes
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
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
static const double DEFAULT_X = 0.0;
static const double DEFAULT_Y = 0.0;
static const double DEFAULT_THETA = 0.0;


//Odometry's Variables
static turtlelib::DiffDrive Dodom;
static turtlelib::Twist2D body_twist;
static std::string wheel_left = "";
static std::string wheel_right = "";

//Odometry's Callbacks

/// \brief JointState subscriber callback fcn. Given the joint states
/// update the robot's configuation and body_twist
/// ignores empty joint state messages
/// \param msg - contains joint state information
void state_handler(const sensor_msgs::JointState& msg)
{
    if (!msg.velocity.empty())
    {
        double left_wheel = Dodom.left_wheel_pos();
        double right_wheel = Dodom.right_wheel_pos();
        //Parse updates
        for (unsigned int i = 0; i < msg.name.size(); i++)
        {
            if (msg.name.at(i) == wheel_left)
            {
                left_wheel += msg.velocity.at(i)/static_cast<double>(DEFAULT_RATE);
            } else if (msg.name.at(i) == wheel_right) {
                right_wheel += msg.velocity.at(i)/static_cast<double>(DEFAULT_RATE);
            }
        }
        //Update body_twist
        body_twist = Dodom.cal_fw_kin(left_wheel, right_wheel);
        //Update configuration
        Dodom.apply_fw_kin(left_wheel, right_wheel);
    }
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
    double x0 = 0.0;
    double y0 = 0.0;
    double theta0 = 0.0;

    if (!nh.getParam("wheel_radius", wheel_radius))
    {
        ROS_ERROR_STREAM("Odomerty: Cannot find wheel_radius"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("track_width", track_width))
    {
        ROS_ERROR_STREAM("Odometry: Cannot find track_width"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("body_id", body_id))
    {
        ROS_ERROR_STREAM("Odometry: Cannot find body_id"); 
        return(1); //return 1 to indicate error
    }  

    nh.param("odom_id", odom_id, DEFAULT_ODOM);

    if (!nh.getParam("wheel_left", wheel_left))
    {
        ROS_ERROR_STREAM("Odometry: Cannot find wheel_left"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("wheel_right", wheel_right))
    {
        ROS_ERROR_STREAM("Odometry: Cannot find wheel_right"); 
        return(1); //return 1 to indicate error
    }

    nh.param("/nusim/x0", x0, DEFAULT_X);
    nh.param("/nusim/y0", y0, DEFAULT_Y);
    nh.param("/nusim/theta0", theta0, DEFAULT_THETA);

    //ROS Objects
    const auto state_sub = nh.subscribe("joint_states", QUEUE_SIZE, state_handler);
    const auto odom_pub = nh.advertise<nav_msgs::Odometry>("odom", QUEUE_SIZE);
    const auto path_pub = nh.advertise<nav_msgs::Path>("odom_path",QUEUE_SIZE, true);
    const auto pose_srv = nh.advertiseService("set_pose", pose_service);
    geometry_msgs::TransformStamped Tob;

    //Init DiffDrive object
    Dodom.set_wheel_config(track_width, wheel_radius);
    Dodom.set_configuration(theta0, x0, y0);
    
    //Set up Rate object
    ros::Rate loop_rate(DEFAULT_RATE);

    //Setup nav_odom object
    nav_msgs::Odometry nav_odom;
    nav_odom.header.frame_id = odom_id;
    nav_odom.child_frame_id = body_id;

    //Setup Transform
    Tob.header.frame_id = odom_id;
    Tob.child_frame_id = body_id;

    // Init Odom path
    nav_msgs::Path odom_path;
    odom_path.header.frame_id = "odom";
    double past_x = Dodom.location().x;
    double past_y = Dodom.location().y;
    double past_theta = Dodom.theta();

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

        // Odom path publishing
        if(!turtlelib::almost_equal(past_x, Dodom.location().x) ||
            !turtlelib::almost_equal(past_y, Dodom.location().y) ||
            !turtlelib::almost_equal(past_theta, Dodom.theta()))
        {
            //Update past reference pts
            past_x = Dodom.location().x;
            past_y = Dodom.location().y;
            //Create new pose
            geometry_msgs::Pose new_pose{};
            new_pose.position.x = Dodom.location().x;
            new_pose.position.y = Dodom.location().y;
            tf2::Quaternion qp;
            qp.setRPY(0, 0, Dodom.theta());
            new_pose.orientation.x = qp.x();
            new_pose.orientation.y = qp.y();
            new_pose.orientation.z = qp.z();
            new_pose.orientation.w = qp.w();
            
            //Create new pose stamped
            geometry_msgs::PoseStamped new_pose_stamped{};
            new_pose_stamped.pose = new_pose;
            odom_path.header.stamp = ros::Time::now();
            new_pose_stamped.header = odom_path.header;
            odom_path.poses.push_back(new_pose_stamped);
            path_pub.publish(odom_path);
        }

        //Sleep
        loop_rate.sleep();
    }
}