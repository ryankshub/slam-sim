/// \file turtle_odom_test_node
/// \brief Test file for odometry node
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
#include "turtlelib/rigid2d.hpp"

//C++ includes
#include <string>

//3rd-party includes
#include "catch_ros/catch.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "nuturtle_control/PoseConfig.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"


//Odometry Test Constants
static const double TEST_SLEEP = 1.0;
static const std::uint32_t QUEUE_SIZE = 1000;
static const double EPSILON = 1.0e-12;
static const std::string DEFAULT_ODOM = "odom";

//Odometry Test Variables
static nav_msgs::Odometry test_odom;

//Odometry Test Callbacks

/// \brief Callback fcn handling odometry msgs. Grabs values for testing
/// \param msg - nav_msgs::Odometry for odometry
void odom_handler(const nav_msgs::Odometry& msg)
{   
    test_odom = msg;
}


TEST_CASE("Odometry Test: Set Pose", "[turtle_odom_test_node]")
{
    //Setup Node
    ros::NodeHandle nh;

    //Make ROS objects
    const auto odom_sub = nh.subscribe("odom", QUEUE_SIZE, odom_handler);
    auto pose_client = nh.serviceClient<nuturtle_control::PoseConfig>("set_pose");

    //Set up Rate object
    ros::Rate sleep_rate(TEST_SLEEP);
    sleep_rate.sleep();

    SECTION( "Testing set_pose zero")
    {
        //Setup Test Objects
        //Set up Odom to non-zero to check zero-out
        test_odom.pose.pose.position.x = 1.0;
        test_odom.pose.pose.position.y = 1.0;
        test_odom.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, turtlelib::PI/2.0);
        test_odom.pose.pose.orientation.x = q.x();
        test_odom.pose.pose.orientation.y = q.y();
        test_odom.pose.pose.orientation.z = q.z(); 
        test_odom.pose.pose.orientation.w = q.w();

        nuturtle_control::PoseConfig srv;
        srv.request.theta = 0.0;
        srv.request.x = 0.0;
        srv.request.y = 0.0;
        pose_client.call(srv);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK( test_odom.pose.pose.position.x == Approx(0.0).margin(EPSILON));
        CHECK( test_odom.pose.pose.position.y == Approx(0.0).margin(EPSILON));
        CHECK( test_odom.pose.pose.position.z == Approx(0.0).margin(EPSILON));
        q.setRPY(0.0, 0.0, 0.0);
        CHECK( test_odom.pose.pose.orientation.x == Approx(q.x()).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.y == Approx(q.y()).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.z == Approx(q.z()).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.w == Approx(q.w()).margin(EPSILON));
    }

    SECTION( "Testing set_pose non-zero")
    {
        //Setup Test Objects
        tf2::Quaternion q;

        nuturtle_control::PoseConfig srv;
        srv.request.theta = turtlelib::PI/2.0;
        srv.request.x = 5.0;
        srv.request.y = 6.0;
        pose_client.call(srv);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK( test_odom.pose.pose.position.x == Approx(5.0).margin(EPSILON));
        CHECK( test_odom.pose.pose.position.y == Approx(6.0).margin(EPSILON));
        CHECK( test_odom.pose.pose.position.z == Approx(0.0).margin(EPSILON));
        q.setRPY(0.0, 0.0, turtlelib::PI/2.0);
        CHECK( test_odom.pose.pose.orientation.x == Approx(q.x()).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.y == Approx(q.y()).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.z == Approx(q.z()).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.w == Approx(q.w()).margin(EPSILON));
    }
}

TEST_CASE("Test transform from odom to base_footprint", "[turtle_odom_test_node]")
{
    //Setup Node
    ros::NodeHandle nh;

    //get Param
    std::string body_id = "";
    std::string odom_id = "";

    if (!nh.getParam("body_id", body_id))
    {
        ROS_ERROR_STREAM("Cannot find body_id"); 
        REQUIRE(false); //return 1 to indicate error
    }  

    nh.param("odom_id", odom_id, DEFAULT_ODOM);

    //Make ROS objects
    const auto odom_sub = nh.subscribe("odom", QUEUE_SIZE, odom_handler);
    auto pose_client = nh.serviceClient<nuturtle_control::PoseConfig>("set_pose");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //Set up Rate object
    ros::Rate sleep_rate(TEST_SLEEP);
    sleep_rate.sleep();

    SECTION( "Testing identity transform")
    {
        //Setup Test Objects
        tf2::Quaternion q;
        geometry_msgs::TransformStamped transformStamped;

        nuturtle_control::PoseConfig srv;
        srv.request.theta = turtlelib::PI/2.0;
        srv.request.x = 5.0;
        srv.request.y = 6.0;
        pose_client.call(srv);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        transformStamped = tfBuffer.lookupTransform(odom_id, body_id, ros::Time(0));
        CHECK( test_odom.pose.pose.position.x == 
            Approx(transformStamped.transform.translation.x).margin(EPSILON));
        CHECK( test_odom.pose.pose.position.y == 
            Approx(transformStamped.transform.translation.y).margin(EPSILON));
        CHECK( test_odom.pose.pose.position.z == 
            Approx(transformStamped.transform.translation.z).margin(EPSILON));
        q.setRPY(0.0, 0.0, turtlelib::PI/2.0);
        CHECK( test_odom.pose.pose.orientation.x == 
            Approx(transformStamped.transform.rotation.x).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.y ==
            Approx(transformStamped.transform.rotation.y).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.z == 
            Approx(transformStamped.transform.rotation.z).margin(EPSILON));
        CHECK( test_odom.pose.pose.orientation.w == 
            Approx(transformStamped.transform.rotation.w).margin(EPSILON));
    }
}