/// \file turtle_interface_test
/// \brief Test file for turtle_interface
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
#include "geometry_msgs/Twist.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"

//Turtle_interface_Test Constants
static const double TEST_SLEEP = 1.0;
static const std::uint32_t QUEUE_SIZE = 1000;
static const double EPSILON = 1.0e-12;

//Turtle_interface_Test Variables
static turtlelib::DiffDrive diff_drive;
static std::vector<double> wheel_vels = {0, 0};
static nuturtlebot_msgs::WheelCommands test_wheel_cmds;
static sensor_msgs::JointState test_joint_states;

//Turtle_Interface_Test Callback

/// \brief Callback fcn handling wheel_cmd msgs. Grabs values for testing
/// \param msg - nuturtlebot_msgs::WheelCommands for encoder ticks
void wheel_handler(const nuturtlebot_msgs::WheelCommands& msg)
{   
    test_wheel_cmds = msg;
}

/// \brief Callback fcn handling encoder sensors; grabs encoder position of wheels.
/// \param msg - nuturtlebot_msgs::SensorData representing turtlebot state
void state_handler(const sensor_msgs::JointState& msg)
{
    test_joint_states = msg;
}


TEST_CASE("Turtle Interface Test: Cmd_Vel", "[turtle_interface_test]")
{
    //Setup Node
    ros::NodeHandle nh;
    //Get Set Params 
    //SET UP TEST YAML FOR PARAMS
    double wheel_radius = 1.0;
    double track_width = 1.0;

    //Make ROS objects
    const auto twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", QUEUE_SIZE);
    const auto wheel_sub = nh.subscribe("wheel_cmd", QUEUE_SIZE, wheel_handler);

    //Init DiffDrive object
    diff_drive.set_wheel_config(track_width, wheel_radius);
    
    //Init Test Objects
    geometry_msgs::Twist test_twist;

    //Set up Rate object
    ros::Rate sleep_rate(TEST_SLEEP);
    sleep_rate.sleep();

    SECTION( "Testing Zero Velocity Command")
    {
        //Setup Test Objects
        test_twist.linear.x = 0.0;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = 0.0;
        //Set-up non zero wheel cmds
        test_wheel_cmds.left_velocity = 1.0;
        test_wheel_cmds.right_velocity = 1.0;

        twist_pub.publish(test_twist);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK(test_wheel_cmds.left_velocity == 0);
        CHECK(test_wheel_cmds.right_velocity == 0);
    }

    SECTION( "Testing Forward Velocity Command")
    {
        //Setup Test Objects
        test_twist.linear.x = 1.0;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = 0.0;

        //Setup Ans
        int left_vel_ans = 41;
        int right_vel_ans = 41;
        twist_pub.publish(test_twist);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK(test_wheel_cmds.left_velocity == left_vel_ans);
        CHECK(test_wheel_cmds.right_velocity == right_vel_ans);
    }

    SECTION( "Testing Backward Velocity Command")
    {
        //Setup Test Objects
        test_twist.linear.x = -1.0;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = 0.0;

        //Setup Ans
        int left_vel_ans = -41;
        int right_vel_ans = -41;
        twist_pub.publish(test_twist);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK(test_wheel_cmds.left_velocity == left_vel_ans);
        CHECK(test_wheel_cmds.right_velocity == right_vel_ans);
    }

    SECTION( "Testing Positive Angular Velocity Command")
    {
        //Setup Test Objects
        test_twist.linear.x = 0.0;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = turtlelib::PI/4.0;

        //Setup Ans
        int left_vel_ans = -16;
        int right_vel_ans = 16;
        twist_pub.publish(test_twist);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK(test_wheel_cmds.left_velocity == left_vel_ans);
        CHECK(test_wheel_cmds.right_velocity == right_vel_ans);
    }

    SECTION( "Testing Negative Angular Velocity Command")
    {
        //Setup Test Objects
        test_twist.linear.x = 0.0;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = -turtlelib::PI/4.0;

        //Setup Ans
        int left_vel_ans = 16;
        int right_vel_ans = -16;
        twist_pub.publish(test_twist);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK(test_wheel_cmds.left_velocity == left_vel_ans);
        CHECK(test_wheel_cmds.right_velocity == right_vel_ans);
    }

    SECTION( "Testing Saturation Velocity Command")
    {
        //Setup Test Objects
        test_twist.linear.x = 200.0;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = 0.0;

        //Setup Ans
        int left_vel_ans = 265;
        int right_vel_ans = 265;
        twist_pub.publish(test_twist);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        CHECK(test_wheel_cmds.left_velocity == left_vel_ans);
        CHECK(test_wheel_cmds.right_velocity == right_vel_ans);
    }
}

/////////////////////////////////////////////////////////////////////////////
TEST_CASE("Turtle Interface Test: Sensor_Encoded", "[turtle_interface_test]")
{
    //Setup Node
    ros::NodeHandle nh;
    //Get Set Params 
    //SET UP TEST YAML FOR PARAMS
    double wheel_radius = 1.0;
    double track_width = 1.0;
    std::string wheel_left = "wheel_left_joint";
    std::string wheel_right = "wheel_right_joint";
    //Make ROS objects
    const auto twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", QUEUE_SIZE);
    const auto wheel_sub = nh.subscribe("wheel_cmd", QUEUE_SIZE, wheel_handler);
    const auto sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("sensor_data", QUEUE_SIZE);
    const auto state_sub = nh.subscribe("joint_states", QUEUE_SIZE, state_handler);

    //Init DiffDrive object
    diff_drive.set_wheel_config(track_width, wheel_radius);
    
    //Init Test Objects
    geometry_msgs::Twist test_twist;
    nuturtlebot_msgs::SensorData test_sensor;

    //Set up Rate object
    ros::Rate sleep_rate(TEST_SLEEP);
    sleep_rate.sleep();

    SECTION( "Testing Zero Joint States")
    {
        //Setup Test Objects
        test_twist.linear.x = 0.0;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = 0.0;

        test_sensor.stamp = ros::Time::now();
        test_sensor.left_encoder = 0;
        test_sensor.right_encoder = 0;

        //Publish
        twist_pub.publish(test_twist);
        sensor_pub.publish(test_sensor);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        for (unsigned int i = 0; i < test_joint_states.name.size(); i++)
        {   
            CHECK(test_joint_states.position.at(i) == Approx( 0.0 ).margin(EPSILON));
            CHECK(test_joint_states.velocity.at(i) == Approx( 0.0 ).margin(EPSILON));
        }
    }

    SECTION( "Testing Non-Zero Joint States")
    {
        //Setup Test Objects
        test_twist.linear.x = 1.5;
        test_twist.linear.y = 0.0;
        test_twist.linear.z = 0.0;
        test_twist.angular.x = 0.0;
        test_twist.angular.y = 0.0;
        test_twist.angular.z = 0.0;

        test_sensor.stamp = ros::Time::now();
        test_sensor.left_encoder = 1024;
        test_sensor.right_encoder = 3413;

        twist_pub.publish(test_twist);
        sensor_pub.publish(test_sensor);
        // Give node time to respond
        sleep_rate.sleep();
        ros::spinOnce();

        for (unsigned int i = 0; i < test_joint_states.name.size(); i++)
        {   
            if (test_joint_states.name.at(i) == wheel_left) 
            {
                CHECK(test_joint_states.position.at(i) == Approx( 1.570816 ).margin(EPSILON));
                CHECK(test_joint_states.velocity.at(i) == Approx( 1.5 ).margin(EPSILON));
            } else if (test_joint_states.name.at(i) == wheel_right){

                CHECK(test_joint_states.position.at(i) == Approx( -1.047643 ).margin(EPSILON));
                CHECK(test_joint_states.velocity.at(i) == Approx( 1.5 ).margin(EPSILON));
            }
        }
    }
}