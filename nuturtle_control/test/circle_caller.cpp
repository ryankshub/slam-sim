/// \file circle_caller
/// \brief Test file for circle node
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

//C++ includes

//3rd-party includes
#include "catch_ros/catch.hpp"
#include "geometry_msgs/Twist.h"
#include "nuturtle_control/CircleControl.h"
#include "ros/ros.h"
#include "ros/console.h"

int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "circle_caller");
    ros::NodeHandle nh;

    //Make ROS objects
    auto pose_client = nh.serviceClient<nuturtle_control::CircleControl>("control");

    nuturtle_control::CircleControl srv;
    srv.request.velocity = .1;
    srv.request.radius = .2;
    pose_client.call(srv);
}