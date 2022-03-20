/// \file landmarks
/// \brief Reads the laser sensor and produces the relative location of the landmarks
///
/// PARAMETERS:
///     cluster_threshold: maximum distance between points in a cluster(m)
/// PUBLISHES:
///     landmarks (visualization_msgs::MarkerArray) : locations of obstacles based 
///         on clustering lidar scan results
/// SUBSCRIBERS:
///     scan (sensor_msgs::LaserScan): Lidar scan 
/// SERVICES:
///
/// BROADCASTER:
///

//RKS

//Project include
#include "nuslam/cluster.hpp"

//C++ includes

//3rd-party includes
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

//Landmark's Constant
static const int DEFAULT_RATE = 500;
static const std::uint32_t QUEUE_SIZE = 1000;
static const double DEFAULT_THRES = 0.05;

//Landmark's Variables
static bool publish_landmarks = false;
static double cluster_thres;
//Landmark's Functions

//Landmark's Callbacks
void laser_handler(const sensor_msgs::LaserScan& msg)
{
    auto clusters = slam_ml::produce_clusters(msg.ranges, msg.angle_increment, cluster_thres);
    publish_landmarks = true;
}

//Landmark's Main Loop
int main(int argc, char *argv[])
{
    //Setup node
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;

    //Local Params

    //Get Params
    nh.param("cluster_threshold", cluster_thres, DEFAULT_THRES);

    //Build ROS Objects
    //Publisher
    const auto landmark_pub = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 100, true);
    //Subscribes
    const auto laser_scan_sub = nh.subscribe("scan", QUEUE_SIZE, laser_handler);

    //Set up Rate object
    ros::Rate loop_rate(DEFAULT_RATE);

    //Loop
    while(ros::ok())
    {
        //Grab Data
        ros::spinOnce();

        if(publish_landmarks)
        {
            publish_landmarks = false;
        }

        //Sleep
        loop_rate.sleep();
    }

    return(0);
}