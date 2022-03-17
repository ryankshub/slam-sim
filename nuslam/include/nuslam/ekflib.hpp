#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file
/// \brief Implementation of the Extended Kalman Filter for diff drive robot

// Project includes
#include "turtlelib/rigid2d.hpp"
// Standard C++ includes
#include <limits>
#include <map>
#include <tuple>
#include <vector>

//3rd Party includes
#include <armadillo>


namespace EKF_DD
{
    /// \brief Class representing the Extending Kalman Filter
    class EKF 
    {
    public:
        /// \brief Default EKF assumes the robot pose is (0, 0, 0) 
        /// with Q as Identity and R as 2x2 of 1
        EKF();

        /// \brief Create an EKF with given robot pose.
        /// with Q as Identity and R as 2x2 of 1
        /// \param theta - initial orientation of the robot
        /// \param x - initial x position of the robot
        /// \param y - initial y position of the robot
        EKF(double theta, double x, double y);

        /// \brief Create an EKF with no landmarks and given robot pose 
        /// w/ noise covariance
        /// \param theta - initial orientation of the robot
        /// \param x - initial x position of the robot
        /// \param y - initial y position of the robot
        /// \param Q - 3x3 noise covariance on robot's motion model
        /// \param R - 2x2 noise covariance on measurement model
        EKF(double theta, double x, double y, const arma::mat & Q, const arma::mat & R);

        /// \brief Set the robot's pose
        /// \param theta - orientation of the robot
        /// \param x - x position of the robot
        /// \param y - y position of the robot        
        void set_current_pose(double theta, double x, double y);

        /// \brief Set the robot's pose
        /// \param theta - orientation of the robot
        /// \param pt - x, y position of the robot
        /// \param y - y position of the robot        
        void set_current_pose(double theta, turtlelib::Vector2D pt);

        /// \brief Return the robot's pose
        /// \return A column vector of (theta, x, y) of the robot
        arma::colvec get_current_pose() const;

        /// \brief Create a state vector with robot's pose and 
        /// landmark's pose
        /// \param id - landmark's id
        /// \param predicted_pose - predicted pose
        /// \return column vector of [robot_pose landmark_pose]
        arma::colvec build_state_vector(int id, arma::colvec predicted_pose) const;

        /// \brief Return the landmark's position
        /// \return A column vector (mx1, my1, .... mxn, myn)
        std::map<int, std::tuple<double, double>> get_current_landmarks() const;

        /// \brief Check if landmark is a new landmark
        /// \return true if landmark is new.
        bool is_new_landmark(int id) const;

        /// \brief Update the pose vector for prediction
        /// \param odom_twist - input body twist of the robot
        arma::colvec predict_pose_vec(turtlelib::Twist2D odom_twist) const;

        /// \brief Update the convances for prediction
        /// \param odom_twist - input body twist of the robot
        arma::mat predict_pose_cov(turtlelib::Twist2D odom_twist) const;

        /// DATA ASSOCIATION HW4

        /// \brief Initiate Landmark if new landmark found
        /// This will add an entry for it's pose and covarance
        /// \param id - id of the Landmark
        /// \param range - range measurement from landmark
        /// \param bearing - bearing measurement from landmark
        /// \param predict_state - prediction robot's pose
        void initiate_landmark(int id, double range, double bearing, arma::colvec predict_state);

        /// \brief Calculate estimated measurement
        /// \param id - Landmark id
        /// \param predict_state - prediction robot's pose
        /// \return - column vector with estimated range and bearing
        arma::colvec compute_est_measurement(int id, arma::colvec predict_state) const;

        /// \brief Build sigma matrix for particular landmark
        /// \param id - Landmark id
        /// \param predicted_cov - predicted covariance
        /// \return - 5 by 5 sigma matrix with predicted robot's cov and landmark cov
        arma::mat build_sigma_mat(int id, arma::mat predicted_cov) const;

        /// \brief Build H matrix for particular landmark
        /// \param id - Landmark id
        /// \param predict_state - prediction robot's pose
        /// \return - 2 x 5 H matrix for kalman gain calculation
        arma::mat build_H_mat(int id, arma::colvec predict_state) const;

        /// \brief Compute the kalman for update step
        /// \param H_mat - H matrix for calculation
        /// \param sigma_mat - sigma matrix for calculation
        arma::mat compute_kalman_gain(arma::mat H_mat, arma::mat sigma_mat) const;

        /// \brief Update the pose of the robot and the landmark
        /// matching the measurement
        /// \param id - Landmark id
        /// \param predicted_pose - predicted pose of robot
        /// \param kalman_gain - Kalman gain
        /// \param est_measure - estimated measurement based on predicted state
        /// \param real_measure - measure from sensor data
        void update_poses(int id, arma::colvec predicted_pose, arma::mat kalman_gain, arma::colvec est_measure, arma::colvec real_measure);

        /// \brief Update the convariance of the robot and landmark
        /// matching the measurement
        /// \param id - Landmark id
        /// \param kalman_gain - Kalmen gain
        /// \param H_mat - Calculated H matrix
        /// \param sigma_mat - sigma matrix for calculation
        void update_covarances(int id, arma::mat kalman_gain, arma::mat H_mat, arma::mat sigma_mat);


    private:
        const int LANDMARK_X_IDX = 0;
        const int LANDMARK_Y_IDX = 1;
        const int POSE_THETA_IDX = 0;
        const int POSE_X_IDX = 1;
        const int POSE_Y_IDX = 2;
        arma::colvec mPose_vec;
        arma::mat mPose_cov;
        arma::mat mQ_cov;
        arma::mat mR_cov;
        std::map<int, std::tuple<double, double>> mLandmark_pose_map;
        std::map<int, std::tuple<double, double, double, double>> mLandmark_cov_map;
    };

    /// \brief Get range from relative x and y measure 
    /// \param rel_x - relative x coordinate of landmark (in robot's frame)
    /// \param rel_y - relative y coordinate of landmark (in robot's frame)
    /// \return range from landmark to robot
    double get_polar_range(double rel_x, double rel_y);

    /// \brief Get bearing from relative x and y measure 
    /// \param rel_x - relative x coordinate of landmark (in robot's frame)
    /// \param rel_y - relative y coordinate of landmark (in robot's frame)
    /// \return bearing from landmark to robot
    double get_polar_bearing(double rel_x, double rel_y);
        
}

#endif