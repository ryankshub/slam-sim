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

//3rd-party includes
#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle pub_nh;
}