/// \file slam
/// \brief The Slam node drives a robot using the ekf filter
///
/// PARAMETERS:
///
/// PUBLISHES:
///
/// BROADCASTER:
///
/// SERVICES:
///

//RKS

//Project include
#include "nuslam/ekflib.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
//C++ includes
#include <string>
//3rd-party include
#include <armadillo>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
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

//Slam's Variables
static bool sim_slam;
static turtlelib::DiffDrive slam_drive;
static turtlelib::Transform2D Tor{turtlelib::Vector2D{0.0, 0.0}, 0.0}; //Transform from odom to robot
static double new_left_wheel;
static double new_right_wheel;
static std::string wheel_left_name = "";
static std::string wheel_right_name = "";
static EKF_DD::EKF ekf_filter;
//Slam's function

void evoke_sim_slam(EKF_DD::EKF & ekf_slam, turtlelib::DiffDrive & dd, 
                    double mx, double my, double mid, 
                    turtlelib::Twist2D input_twist)
{
    //Convert measurement to polar
    double range = EKF_DD::get_polar_range(mx, my);
    double bearing = EKF_DD::get_polar_bearing(mx, my);
    arma::colvec real_measure{range, bearing};
    //Predict pose
    arma::colvec predicted_pose = ekf_slam.predict_pose_vec(input_twist);
    //Predict cov
    arma::mat predicted_cov = ekf_slam.predict_pose_cov(input_twist);
    //Check Landmark init
    if (ekf_slam.is_new_landmark(mid))
    {
        ekf_slam.initiate_landmark(mid, range, bearing, predicted_pose);
    } 
    //Update: get theoretical measurement
    arma::colvec est_measure = ekf_slam.compute_est_measurement(mid, predicted_pose);

    //Update: Build sigma and H
    arma::mat sigma_mat = ekf_slam.build_sigma_mat(mid);
    arma::mat H_mat = ekf_slam.build_H_mat(mid, predicted_pose);

    //Update: Compute Kalman
    arma::mat kalman_gain = ekf_slam.compute_kalman_gain(H_mat, sigma_mat);

    //Update state
    ekf_slam.update_poses(mid, kalman_gain, est_measure, real_measure);

    //Update covarance
    ekf_slam.update_covarances(mid, kalman_gain, H_mat, sigma_mat);

    //Update diffdrive
    arma::colvec new_pose = ekf_slam.get_current_pose();
    dd.set_configuration(new_pose(0), new_pose(1), new_pose(2));
}


//Slam's callbacks

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
        double mid = obs.id;
        
        //Get Input
        turtlelib::Twist2D input_twist = slam_drive.cal_fw_kin_vel(new_left_wheel, new_right_wheel);
        slam_drive.set_left_wheel_pos(new_left_wheel);
        slam_drive.set_right_wheel_pos(new_right_wheel);

        //Call Slam
        evoke_sim_slam(ekf_filter, slam_drive, mx, my, mid, input_twist);
    }

}


/// \brief Joint State Callback Fcn. This accumulates
/// the joint state angles for converting to body twist
/// for slam
/// \param msg - contains joint angles and velocities
void joint_state_handler(const sensor_msgs::JointState& msg)
{
    if (!msg.velocity.empty())
    {
        //Parse updates
        for (unsigned int i = 0; i < msg.name.size(); i++)
        {
            if (msg.name.at(i) == wheel_left_name)
            {
                new_left_wheel += msg.velocity.at(i)/static_cast<double>(DEFAULT_RATE);
            } else if (msg.name.at(i) == wheel_right_name) {
                new_right_wheel += msg.velocity.at(i)/static_cast<double>(DEFAULT_RATE);
            }
        }
    }
}


/// \brief Odom Callback Fcn. This merely saves the transformation 
/// from odom to robot
void odom_handler(const nav_msgs::Odometry& msg)
{
    double odom_x = msg.pose.pose.position.x;
    double odom_y = msg.pose.pose.position.y;
    tf2::Quaternion q( 
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Tor = turtlelib::Transform2D{turtlelib::Vector2D{odom_x, odom_y}, yaw};
}

/// \brief Odom Callback 

//Slam's Main Function
int main(int argc, char *argv[])
{
    //Setup Node
    ros::init(argc, argv, "slam");
    ros::NodeHandle nh("~");
    ros::NodeHandle pub_nh;
    tf2_ros::TransformBroadcaster br;

    // Local Params
    double x_init = 0;
    double y_init = 0;
    double theta_init = 0;
    double wheel_radius = 0.0;
    double track_width = 0.0;

    // Get Params
    nh.param("x0", x_init, DEFAULT_X);
    nh.param("y0", y_init, DEFAULT_Y);
    nh.param("theta0", theta_init, DEFAULT_THETA);
    nh.param("sim_slam", sim_slam, true);

    if (!pub_nh.getParam("wheel_radius", wheel_radius))
    {
        ROS_ERROR_STREAM("Slam: Cannot find wheel radius for robot");
        return(1);
    }

    if (!pub_nh.getParam("track_width", track_width))
    {
        ROS_ERROR_STREAM("Slam: Cannot find track width for robot");
        return(1);
    }

    if (!pub_nh.getParam("wheel_left", wheel_left_name))
    {
        ROS_ERROR_STREAM("Odometry: Cannot find wheel_left"); 
        return(1); //return 1 to indicate error
    }

    if (!pub_nh.getParam("wheel_right", wheel_right_name))
    {
        ROS_ERROR_STREAM("Odometry: Cannot find wheel_right"); 
        return(1); //return 1 to indicate error
    }

    //Build ROS Objects
    //Subscribers
    const auto fake_sensor_sub = pub_nh.subscribe("fake_sensor", QUEUE_SIZE, sim_sensor_handler);
    const auto joint_state_sub = pub_nh.subscribe("joint_states", QUEUE_SIZE, joint_state_handler);
    const auto odom_sub = pub_nh.subscribe("odom", QUEUE_SIZE, odom_handler);
    //Transforms
    geometry_msgs::TransformStamped Tmo_stamped;

    //Init DiffDrive
    slam_drive.set_wheel_config(track_width, wheel_radius);
    slam_drive.set_configuration(theta_init, x_init, y_init);
    new_left_wheel = slam_drive.left_wheel_pos();
    new_right_wheel = slam_drive.right_wheel_pos();
    
    //Init EKF filter
    ekf_filter.set_current_pose(theta_init, x_init, y_init);
    //Set up Rate object
    ros::Rate loop_rate(DEFAULT_RATE);

    //Set up broadcaster from map to odom
    Tmo_stamped.header.frame_id = "map";
    Tmo_stamped.child_frame_id = "odom";

    while(ros::ok())
    {
        //Grab data
        ros::spinOnce();

        //Calculate transform
        turtlelib::Transform2D Tmr = turtlelib::Transform2D{slam_drive.location(), slam_drive.theta()};
        turtlelib::Transform2D Tmo = Tmr*Tor.inv();

        //Broadcast transform
        Tmo_stamped.header.stamp = ros::Time::now();
        Tmo_stamped.transform.translation.x = Tmo.translation().x; 
        Tmo_stamped.transform.translation.y = Tmo.translation().y;
        Tmo_stamped.transform.translation.z =  0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, Tmo.rotation());
        Tmo_stamped.transform.rotation.x = q.x();
        Tmo_stamped.transform.rotation.y = q.y(); 
        Tmo_stamped.transform.rotation.z = q.z(); 
        Tmo_stamped.transform.rotation.w = q.w();

        br.sendTransform(Tmo_stamped);

        loop_rate.sleep();
    }
    return(0);
}

