/// \file circle
/// \brief publishes the cmd_vel messages to have the robot drive in a circle
///
/// PARAMETERS:
///    
/// PUBLISHES:
///
/// SUBSCRIBERS:
///

// RKS

// Project Includes

// Standard Includes

// 3rd-party Includes
#include "geometry_msgs/Twist.h"
#include "nuturtle_control/CircleControl.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "std_srvs/Empty.h"

//Circle Constants
static const std::uint32_t QUEUE_SIZE = 1000;

//Circle states
enum class State { STOP,
                   STOPPING,
                   GO,};

//Circle variable
static State state {State::STOP};
static geometry_msgs::Twist body_twist;
static int frequency = 0;

//Circle callbacks
/// \brief Control service callback fcn. Given a req, set the angular and 
/// linear velocity of the robot to go in follow of circle of the given radius
/// Note: Giving a radius < 0.0 will be ignored.
/// \param req - Request with the angular velocity and radius of circle
bool control_service(nuturtle_control::CircleControl::Request& req,
                     nuturtle_control::CircleControl::Response&)
{
    // Ignore req
    if (req.radius < 0.0)
    {
        return true;
    }
    body_twist.angular.z = req.velocity;
    body_twist.linear.x = req.velocity*req.radius;
    state = State::GO;
    return true;
}

/// \brief Reverse service callback fcn. This service reverses the direction of 
/// the robot.
bool reverse_service(std_srvs::Empty::Request&,
                     std_srvs::Empty::Response&)
{
    body_twist.angular.z *= -1.0;
    body_twist.linear.x *= -1.0;
    return true;
}

/// \brief Stop service callback fcn. This service stops the robot. 
bool stop_service(std_srvs::Empty::Request&,
                  std_srvs::Empty::Response&)
{
    body_twist.angular.z = 0.0;
    body_twist.linear.x = 0.0;
    state = State::STOPPING;
    return true;
}

//Main loop
int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh;
    ros::NodeHandle pri_nh("~");

    //Grab Params
    pri_nh.param("frequency", frequency, 100);

    //Make ROS objects
    const auto cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", QUEUE_SIZE);
    const auto control_srv = nh.advertiseService("control", control_service);
    const auto reserve_srv = nh.advertiseService("reverse", reverse_service);
    const auto stop_srv = nh.advertiseService("stop", stop_service);

    //Init rate
    ros::Rate loop_rate(frequency);

    //Init body twist
    body_twist.linear.x = 0.0;
    body_twist.linear.y = 0.0;
    body_twist.linear.z = 0.0;
    body_twist.angular.x = 0.0;
    body_twist.angular.y = 0.0;
    body_twist.angular.z = 0.0;

    //Main loop
    while(ros::ok())
    {
        //Grab Data
        ros::spinOnce();

        //State Machine
        if (state == State::GO)
        {
            cmd_vel_pub.publish(body_twist);

        } else if (state == State::STOPPING) {
            cmd_vel_pub.publish(body_twist);
            state = State::STOP;
        }

        //Sleep
        loop_rate.sleep();
    }
}