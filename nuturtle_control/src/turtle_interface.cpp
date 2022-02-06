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

//Turtle_interface Variables
static turtlelib::DiffDrive diff_drive;
static nuturtlebot_msgs::WheelCommands wheel_cmds;
static bool new_wheel_cmd = false;

//Turtle_Interface Callback

/// \brief Callback fcn handling twist msgs. Converts body_twist into wheel velocity cmds.
/// If Twist is valid, populate wheel_cmds for publishing. Else, ignore message
/// \param msg - geometry_msgs::Twist representing body_twist velocity
void twist_handler(const geometry_msgs::Twist& msg)
{
    bool invalid_msg = false;
    turtlelib::Twist2D body_twist{msg.angular.z,
                                msg.linear.x,
                                msg.linear.y};
    std::vector<double> vels;
    try
    {
        ROS_INFO_STREAM("DIFF DRIVE " << diff_drive);
        vels = diff_drive.cal_inv_kin(body_twist);
    } catch(const std::exception& e) {
        invalid_msg = true;
    }

    if (!invalid_msg){
        new_wheel_cmd = true;
        wheel_cmds.left_velocity = static_cast<int>(vels.at(0));
        wheel_cmds.right_velocity = static_cast<int>(vels.at(1));
    }
}

void sensor_handler(const nuturtlebot_msgs::SensorData& msg)
{
    ///Stuff
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

    //Make ROS objects
    const auto twist_sub = nh.subscribe("cmd_vel", QUEUE_SIZE, twist_handler);
    //const auto sensor_sub = nh.subscriber("sensor_data", QUEUE_SIZE, sensor_handler);
    const auto wheel_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", QUEUE_SIZE);
    //const auto state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", QUEUE_SIZE);

    //Create DiffDrive object
    turtlelib::DiffDrive diff_drive{track_width, wheel_radius};

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

        //Sleep
        loop_rate.sleep();
    }


}