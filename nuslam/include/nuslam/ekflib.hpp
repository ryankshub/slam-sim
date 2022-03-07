#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file
/// \brief Implementation of the Extended Kalman Filter for diff drive robot

// Project includes
#include "turtlelib/rigid2d.hpp"
// Standard C++ includes
#include <limits>
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
        /// \brief Default EKF with 3 landmarks at 3,3 and assumes the robot pose is
        /// (0, 0, 0)
        EKF();

        /// \brief Create an EKF with given number of landmarks at 3,3 and robot pose
        /// at origin (0, 0, 0)
        /// \param num_landmarks - Initial number of landmarks
        EKF(int num_landmarks);

        /// \brief Create an EKF with initial landmarks positions and robot
        /// at (0, 0, 0)
        /// \param landmarks_pts - list of landmark locations
        EKF(const std::vector<std::tuple<double, double>> & landmarks_pts);

        /// \brief Create an EKF with no landmarks and given robot pose.
        /// \param theta - initial orientation of the robot
        /// \param x - initial x position of the robot
        /// \param y - initial y position of the robot
        EKF(double theta, double x, double y);

        /// \brief Create an EKF with initial number of landmarks at 3,3 and given
        /// robot pose
        /// \param num_landmarks - Initial number of landmarks
        /// \param theta - initial orientation of the robot
        /// \param x - initial x position of the robot
        /// \param y - initial y position of the robot
        EKF(int num_landmarks, double theta, double x, double y);

        /// \brief Create an EKF with list of landmarks positions and given
        /// robot pose
        /// \param landmarks_pts - list of landmark locations
        /// \param theta - initial orientation of the robot
        /// \param x - initial x position of the robot
        /// \param y - initial y position of the robot
        EKF(const std::vector<std::tuple<double, double>> & landmarks_pts, 
            double theta, double x, double y);

        /// \brief Create an EKF with number of landmarks, list of landmarks positions, 
        /// and given robot pose
        /// \param landmarks_pts - list of landmark locations
        /// \param theta - initial orientation of the robot
        /// \param x - initial x position of the robot
        /// \param y - initial y position of the robot
        EKF(int num_landmarks, const std::vector<std::tuple<double, double>> & landmarks_pts, 
            double theta, double x, double y);

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
        arma::colvec get_current_pose() const;

        /// \brief Return the landmark's position
        arma::colvec get_current_landmarks() const;

        /// \brief Return the state vector
        arma::colvec get_current_state() const;

    private:
        const double DEFAULT_LANDMARK_POS_X = 3.0;
        const double DEFAULT_LANDMARK_POS_Y = 3.0;
        arma::colvec mPose_vec;
        arma::mat mPose_cov;
        int mNum_landmarks;
        arma::colvec mLandmark_vec;
        arma::mat mLandmark_cov;
        // arma::mat{2*num_landmarks, 2*num_landmarks, arma::fill::eye}*std::numeric_limits<double>::max();
    };
}

#endif