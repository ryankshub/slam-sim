/// \file turtle_interface
/// \brief The Turtle Interface converts body_twist to wheel_cmds and publish state of wheels on joints states
///
/// PARAMETERS:
///
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
#include <stdexcept>

//3rd-party includes
#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"

//Turtle_interface Constants
static const int DEFAULT_RATE = 500;
static const std::uint32_t QUEUE_SIZE = 1000;
static const int LEFT_WHEEL = 0;
static const int RIGHT_WHEEL = 1;

//Turtle_interface Variables
static turtlelib::DiffDrive diff_drive;
static std::vector<double> wheel_vels = {0, 0};
static nuturtlebot_msgs::WheelCommands wheel_cmds;
static sensor_msgs::JointState joint_states;
static bool new_wheel_cmd = false;
static bool new_joint_states = false;
static double motor_cmd_max = 0.0;
static double motor_cmd_to_rads = 0.0;
static double encoder_ticks_to_rad = 0.0;

//Turtle_Interface Callback

/// \brief Callback fcn handling twist msgs. Converts body_twist into wheel velocity cmds.
/// If Twist is valid, populate wheel_cmds for publishing. Else, ignore message
/// \param msg - geometry_msgs::Twist representing body_twist velocity
void twist_handler(const geometry_msgs::Twist& msg)
{   
    
    bool invalid_msg = false;
    //Set up twist
    turtlelib::Twist2D body_twist{msg.angular.z,
                                msg.linear.x,
                                msg.linear.y};
    // Inv Kinematics
    try
    {
        wheel_vels = diff_drive.cal_inv_kin(body_twist);
    } catch(const std::exception& e) {
        invalid_msg = true;
    }

    if (!invalid_msg){
        new_wheel_cmd = true;
        double left_ticks = wheel_vels.at(LEFT_WHEEL)/motor_cmd_to_rads;
        double right_ticks = wheel_vels.at(RIGHT_WHEEL)/motor_cmd_to_rads;
        //Cap left_ticks
        if (left_ticks > motor_cmd_max)
        {
            left_ticks = motor_cmd_max;
        } else if (left_ticks < -motor_cmd_max)
        {
            left_ticks = -motor_cmd_max;
        }

        //Cap right_ticks
        if (right_ticks > motor_cmd_max)
        {
            right_ticks = motor_cmd_max;
        } else if (right_ticks < -motor_cmd_max)
        {
            right_ticks = -motor_cmd_max;
        }

        wheel_cmds.left_velocity = static_cast<int>(left_ticks);
        wheel_cmds.right_velocity = static_cast<int>(right_ticks);
    }
}

/// \brief Callback fcn handling encoder sensors; grabs encoder position of wheels.
/// \param msg - nuturtlebot_msgs::SensorData representing turtlebot state
void sensor_handler(const nuturtlebot_msgs::SensorData& msg)
{
    //Set joint state flag
    new_joint_states = true;
    //Header Stamp
    joint_states.header.stamp = msg.stamp;

    joint_states.position.at(LEFT_WHEEL) = msg.left_encoder*encoder_ticks_to_rad;
    joint_states.position.at(RIGHT_WHEEL) = msg.right_encoder*encoder_ticks_to_rad;

}


//Turtle_interface Main Loop
int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;

    //Get Params
    double wheel_radius = 0.0;
    double track_width = 0.0;
    // Get Params
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

    if (!nh.getParam("motor_cmd_max", motor_cmd_max))
    {
        ROS_ERROR_STREAM("Cannot find motor_cmd_max"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("motor_cmd_to_rads", motor_cmd_to_rads))
    {
        ROS_ERROR_STREAM("Cannot find motor_cmd_to_rads"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad))
    {
        ROS_ERROR_STREAM("Cannot find encoder_ticks_to_rad"); 
        return(1); //return 1 to indicate error
    }

    //Make ROS objects
    const auto twist_sub = nh.subscribe("cmd_vel", QUEUE_SIZE, twist_handler);
    const auto sensor_sub = nh.subscribe("sensor_data", QUEUE_SIZE, sensor_handler);
    const auto wheel_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", QUEUE_SIZE);
    const auto state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", QUEUE_SIZE);

    //Init DiffDrive object
    diff_drive.set_wheel_config(track_width, wheel_radius);
    //Init joint states
    joint_states.name.push_back("wheel_left_joint");
    joint_states.name.push_back("wheel_right_joint");
    joint_states.position.push_back(0.0);
    joint_states.position.push_back(0.0);
    joint_states.velocity.push_back(0.0);
    joint_states.velocity.push_back(0.0);
    //Set up Rate object
    ros::Rate loop_rate(DEFAULT_RATE);

    //Main loop
    while(ros::ok())
    {
        //Grab data
        ros::spinOnce();

        //Check wheel commands
        if (new_wheel_cmd)
        {
            new_wheel_cmd = false;
            wheel_pub.publish(wheel_cmds);
        }

        if (new_joint_states)
        {
            new_joint_states = false;
            joint_states.velocity.at(LEFT_WHEEL) = wheel_vels.at(LEFT_WHEEL);
            joint_states.velocity.at(RIGHT_WHEEL) = wheel_vels.at(RIGHT_WHEEL);
            state_pub.publish(joint_states);
        }

        //Sleep
        loop_rate.sleep();
    }


}