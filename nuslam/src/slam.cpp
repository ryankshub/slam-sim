/// \file slam
/// \brief The Slam node drives a robot using the ekf filter and odometry
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
/// BROADCASTER:
///     broadcaster (non-static odom_id to body_id): Transformation matrix from odom_id frame
///         to body_id frame

//RKS

//Project include
#include "nuslam/ekflib.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"

//C++ includes
#include <vector>
#include <string>
//3rd-party include
#include <armadillo>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "nuturtle_control/PoseConfig.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


//Slam's Constants
static const int DEFAULT_RATE = 500;
static const std::uint32_t QUEUE_SIZE = 1000;
static const double DEFAULT_X = 0.0;
static const double DEFAULT_Y = 0.0;
static const double DEFAULT_THETA = 0.0;
static const std::string DEFAULT_ODOM = "odom";
static const std::string DEFAULT_TYPE = "slam";

//Slam's Variables
static bool sim_slam;
static turtlelib::DiffDrive Dodom;
static turtlelib::DiffDrive slam_drive;
static turtlelib::Twist2D body_twist;
static double new_left_wheel;
static double new_right_wheel;
static EKF_DD::EKF ekf_filter;
static std::string wheel_left = "";
static std::string wheel_right = "";

//Slam's function

void evoke_sim_slam(EKF_DD::EKF & ekf_slam, turtlelib::DiffDrive & dd, 
                    double mx, double my, int mid, 
                    turtlelib::Twist2D input_twist)
{
    //Convert measurement to polar
    double range = EKF_DD::get_polar_range(mx, my);
    double bearing = EKF_DD::get_polar_bearing(mx, my);
    arma::colvec real_measure{range, bearing};

    // Set EKF position with odometry
    ekf_slam.set_current_pose(dd.theta(), dd.location());

    //Predict pose
    arma::colvec predicted_pose{dd.theta(), dd.location().x, dd.location().y};

    //Predict cov
    arma::mat predicted_cov = ekf_slam.predict_pose_cov(input_twist);

    //Check Landmark init
    if (ekf_slam.is_new_landmark(mid))
    {   
        ekf_slam.initiate_landmark(mid, range, bearing, predicted_pose);
    } 

    //Update: get theoretical measurement
    arma::colvec est_measure = ekf_slam.compute_est_measurement(mid, predicted_pose);

    // //Update: Build sigma and H
    arma::mat sigma_mat = ekf_slam.build_sigma_mat(mid, predicted_cov);
    arma::mat H_mat = ekf_slam.build_H_mat(mid, predicted_pose);


    // //Update: Compute Kalman
    arma::mat kalman_gain = ekf_slam.compute_kalman_gain(H_mat, sigma_mat);

    // //Update state
    ekf_slam.update_poses(mid, predicted_pose, kalman_gain, est_measure, real_measure);

    // //Update covarance
    ekf_slam.update_covarances(mid, kalman_gain, H_mat, sigma_mat);

    // //Update diffdrive
    arma::colvec new_pose = ekf_slam.get_current_pose();
    dd.set_configuration(new_pose(0), new_pose(1), new_pose(2));
    ROS_WARN_STREAM("NEW POSE " << predicted_pose);
}


//Slam's callbacks
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

// /// \brief set_pose service callback fcn. Teleports the robot 
// /// to a given position. (Does not reset the twist)
bool pose_service(nuturtle_control::PoseConfig::Request &req,
                  nuturtle_control::PoseConfig::Response &)
{
    Dodom.set_configuration(req.theta, req.x, req.y);
    slam_drive.set_configuration(req.theta, req.x, req.y);
    return true;
}

/// \brief Fake Sensor Callback Fcn. This sensor simulates
/// reading a measurement from the environment. 
/// \param msg - contains relative x and y of obstacles in range 
void sim_sensor_handler(const visualization_msgs::MarkerArray& msg)
{
    for(visualization_msgs::Marker obs: msg.markers)
    {
        //Check measurement
        if (obs.action == visualization_msgs::Marker::DELETE)
        {
            continue; //Marker out of range
        }

        //Get measurement
        double mx = obs.pose.position.x;
        double my = obs.pose.position.y;
        int mid = obs.id;
        //Get new position from odometry angle
        double left_wheel = Dodom.left_wheel_pos();
        double right_wheel = Dodom.right_wheel_pos();
        //Get Input
        turtlelib::Twist2D input_twist = slam_drive.cal_fw_kin(left_wheel, right_wheel);
        slam_drive.set_left_wheel_pos(left_wheel);
        slam_drive.set_right_wheel_pos(right_wheel);
        //Call Slam
        evoke_sim_slam(ekf_filter, Dodom, mx, my, mid, input_twist);
        slam_drive.set_configuration(Dodom.theta(), Dodom.location().x, Dodom.location().y);
    }

}


//Slam's Main Function
int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "slam");
    ros::NodeHandle pri_nh("~");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;

    //Local Params
    std::string body_id = "";
    std::string odom_id = "";
    std::string odom_type = "";
    double wheel_radius = 0.0;
    double track_width = 0.0;
    double x0 = 0.0;
    double y0 = 0.0;
    double theta0 = 0.0;

    // Get Params
    nh.param("/nusim/x0", x0, DEFAULT_X);
    nh.param("/nusim/y0", y0, DEFAULT_Y);
    nh.param("/nusim/theta0", theta0, DEFAULT_THETA);
    pri_nh.param("odom_type", odom_type, DEFAULT_TYPE);
    nh.param("sim_slam", sim_slam, true);

    //
    if (!nh.getParam("wheel_radius", wheel_radius))
    {
        ROS_ERROR_STREAM("Slam: Cannot find wheel_radius"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("track_width", track_width))
    {
        ROS_ERROR_STREAM("Slam: Cannot find track_width"); 
        return(1); //return 1 to indicate error
    }

    if (!pri_nh.getParam("body_id", body_id))
    {
        ROS_ERROR_STREAM("Slam: Cannot find body_id"); 
        return(1); //return 1 to indicate error
    }  

    pri_nh.param("odom_id", odom_id, DEFAULT_ODOM);

    if (!nh.getParam("wheel_left", wheel_left))
    {
        ROS_ERROR_STREAM("Slam: Cannot find wheel_left"); 
        return(1); //return 1 to indicate error
    }

    if (!nh.getParam("wheel_right", wheel_right))
    {
        ROS_ERROR_STREAM("Slam: Cannot find wheel_right"); 
        return(1); //return 1 to indicate error
    }

    //Build ROS Objects
    //Publishers
    const auto odom_pub = nh.advertise<nav_msgs::Odometry>((odom_type + "/odom"), QUEUE_SIZE);
    const auto path_pub = nh.advertise<nav_msgs::Path>((odom_type + "/odom_path"),QUEUE_SIZE, true);
    //Subscribers
    const auto fake_sensor_sub = nh.subscribe("fake_sensor", QUEUE_SIZE, sim_sensor_handler);
    const auto state_sub = nh.subscribe("joint_states", QUEUE_SIZE, state_handler);
    //Services
    const auto pose_srv = nh.advertiseService("set_pose", pose_service);
    //Transforms
    geometry_msgs::TransformStamped Tob;
    geometry_msgs::TransformStamped Tmo;

    //Init DiffDrive
    Dodom.set_wheel_config(track_width, wheel_radius);
    Dodom.set_configuration(theta0, x0, y0);
    slam_drive.set_wheel_config(track_width, wheel_radius);
    slam_drive.set_configuration(theta0, x0, y0);
    new_left_wheel = slam_drive.left_wheel_pos();
    new_right_wheel = slam_drive.right_wheel_pos();
    
    //Init EKF filter
    ekf_filter.set_current_pose(theta0, x0, y0);
    //Set up Rate object
    ros::Rate loop_rate(DEFAULT_RATE);

    //Setup nav_odom object
    nav_msgs::Odometry nav_odom;
    nav_odom.header.frame_id = odom_id;
    nav_odom.child_frame_id = body_id;

    //Setup Transform
    Tob.header.frame_id = odom_id;
    Tob.child_frame_id = body_id;
    //Set up broadcaster from map to odom
    Tmo.header.frame_id = "map";
    Tmo.child_frame_id = odom_id;

    // Init Odom path
    nav_msgs::Path odom_path;
    odom_path.header.frame_id = "map";
    double past_x = slam_drive.location().x;
    double past_y = slam_drive.location().y;
    double past_theta = slam_drive.theta();

    while(ros::ok())
    {
        //Grab data
        ros::spinOnce();

        // Broadcast Transform Tob
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

        //Make Tob Transform
        turtlelib::Transform2D Tob_raw = turtlelib::Transform2D{Dodom.location(), Dodom.theta()};


        // Publish odom on nav stack
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


        //Calculate transform
        turtlelib::Transform2D Tmb_raw = turtlelib::Transform2D{slam_drive.location(), slam_drive.theta()};
        turtlelib::Transform2D Tmo_raw = Tmb_raw*Tob_raw.inv();

        //Broadcast transform
        Tmo.header.stamp = ros::Time::now();
        Tmo.transform.translation.x = Tmo_raw.translation().x; 
        Tmo.transform.translation.y = Tmo_raw.translation().y;
        Tmo.transform.translation.z =  0.0;
        q.setRPY(0, 0, Tmo_raw.rotation());
        Tmo.transform.rotation.x = q.x();
        Tmo.transform.rotation.y = q.y(); 
        Tmo.transform.rotation.z = q.z(); 
        Tmo.transform.rotation.w = q.w();

        br.sendTransform(Tmo);

        // slam path publishing
        if(!turtlelib::almost_equal(past_x, slam_drive.location().x) ||
            !turtlelib::almost_equal(past_y, slam_drive.location().y) ||
            !turtlelib::almost_equal(past_theta, slam_drive.theta()))
        {
            //Update past reference pts
            past_x = slam_drive.location().x;
            past_y = slam_drive.location().y;
            //Create new pose
            geometry_msgs::Pose new_pose{};
            new_pose.position.x = slam_drive.location().x;
            new_pose.position.y = slam_drive.location().y;
            tf2::Quaternion qp;
            qp.setRPY(0, 0, slam_drive.theta());
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

        loop_rate.sleep();
    }
    return(0);
}

