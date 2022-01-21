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
#include "ros/ros.h"
#include "std_msgs/UInt64.h"
#include "std_srvs/Empty.h"

//Nusim's Constants
static const int DEFAULT_RATE = 500;
static const std::uint32_t QUEUE_SIZE = 1000;

// Nusim Node's variables
static std::uint64_t timestep = 0;
static int rate;

//Nusim Node's callbacks
bool reset(std_srvs::Empty::Request &req,
           std_srvs::Empty::Response &res)
{
    timestep = 0;
    return true;
}

//Nusim Main Function
int main(int argc, char *argv[])
{
    //Setup node
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~"); //Private scoping node handler

    //Get Params
    nh.param("rate", rate, DEFAULT_RATE);

    //Build ROS Objects
    const auto timestep_pub = nh.advertise<std_msgs::UInt64>("timestep", QUEUE_SIZE);
    const auto reset_srv = nh.advertiseService("reset", reset);

    //Set up Rate object
    ros::Rate loop_rate(rate);

    //Main loop
    while(ros::ok())
    {
        //Update data
        timestep++; //this means we start at T = 1
        
        //Package and send data
        std_msgs::UInt64 msg;
        msg.data = timestep;
        timestep_pub.publish(msg);


        //Grab data
        ros::spinOnce();

        //Sleep
        loop_rate.sleep();
        
    }

    return(0);

}
