#include "nuslam/ekflib.hpp"
#include <cmath>

namespace EKF_DD
{
    //Empty EKF
    EKF::EKF()
        : EKF{0, std::vector<std::tuple<double,double>>{}, 0.0, 0.0, 0.0}
    {
    }

    //EKF with number of landmarks
    EKF::EKF(int num_landmarks)
        : EKF{num_landmarks, std::vector<std::tuple<double, double>>{}, 0.0, 0.0, 0.0}
    {
    }

    //EKF with landmark points
    EKF::EKF(const std::vector<std::tuple<double, double>> & landmarks_pts)
        : EKF{static_cast<int>(landmarks_pts.size()), landmarks_pts, 0.0, 0.0, 0.0}
    {
    }

    //EKF with robot pose
    EKF::EKF(double theta, double x, double y)
        : EKF{0, std::vector<std::tuple<double, double>>{}, theta, x, y}
    {      
    }

    //EKF with num landmarks and robot pose
    EKF::EKF(int num_landmarks, double theta, double x, double y)
        : EKF{static_cast<int>(num_landmarks), std::vector<std::tuple<double, double>>{}, theta, x, y}
    {
    }

    //EKF with landmarks_pt and robot pose
    EKF::EKF(const std::vector<std::tuple<double, double>> & landmarks_pts,
            double theta, double x, double y)
        : EKF{static_cast<int>(landmarks_pts.size()), landmarks_pts, theta, x, y}
    {
    }

    //EKF with num_landmarks, landmarks_pt, and robot pose
    EKF::EKF(int num_landmarks, const std::vector<std::tuple<double, double>> & landmarks_pts,
            double theta, double x, double y)
        : mPose_vec{turtlelib::normalize_angle(theta), x, y}
        , mPose_cov(3, 3, arma::fill::zeros)
        , mNum_landmarks{num_landmarks}
    {
        std::vector<double> landmarks_flat;
        if (landmarks_pts.empty() || static_cast<int>(landmarks_pts.size()) != num_landmarks)
        {
            for(int i = 0; i < num_landmarks; i++)
            {
                landmarks_flat.push_back(DEFAULT_LANDMARK_POS_X); //Yo, maybe these 
                landmarks_flat.push_back(DEFAULT_LANDMARK_POS_Y); //should be randomized
            }
        } else {
            for (std::tuple pt : landmarks_pts){
                landmarks_flat.push_back(std::get<0>(pt));
                landmarks_flat.push_back(std::get<1>(pt));
            }
        }
        mLandmark_vec = arma::colvec(landmarks_flat);
        mLandmark_cov = arma::mat(2*num_landmarks, 2*num_landmarks, arma::fill::eye)*std::numeric_limits<double>::max();
    }

    //Set current pose with 3 elem point
    void EKF::set_current_pose(double theta, double x, double y)
    {
        mPose_vec = arma::colvec{std::vector{turtlelib::normalize_angle(theta), x, y}};
        mPose_cov = arma::mat(3, 3, arma::fill::zeros);
    }

    //Sef current pose with angle and vector
    void EKF::set_current_pose(double theta, turtlelib::Vector2D pt)
    {
        mPose_vec = arma::colvec(std::vector{turtlelib::normalize_angle(theta), pt.x, pt.y});
        mPose_cov = arma::mat(3, 3, arma::fill::zeros);
    }

    //Getters

    arma::colvec EKF::get_current_pose() const
    {
        return mPose_vec;
    }

    arma::colvec EKF::get_current_landmarks() const
    {
        return mLandmark_vec;
    }

    arma::colvec EKF::get_current_state() const
    {
        return arma::join_cols(mPose_cov, mLandmark_vec);
    }

    // Prediciton Functions
    arma::colvec EKF::predict_pose_vec(turtlelib::Twist2D odom_twist) const
    {
        if(turtlelib::almost_equal(odom_twist.theta_dot, 0.0)){
            double x_delta = odom_twist.x_dot * std::cos(mPose_vec(POSE_THETA_IDX));
            double y_delta = odom_twist.x_dot * std::sin(mPose_vec(POSE_THETA_IDX));
            arma::colvec delta{0.0, x_delta, y_delta};

            return mPose_vec + delta;
        } else {
            double t_delta = odom_twist.theta_dot;
            double x_delta1 = -(odom_twist.x_dot/odom_twist.theta_dot)*std::sin(mPose_vec(POSE_THETA_IDX));
            double x_delta2 = (odom_twist.x_dot/odom_twist.theta_dot)*std::sin(mPose_vec(POSE_THETA_IDX) + odom_twist.theta_dot);
            double y_delta1 = (odom_twist.x_dot/odom_twist.theta_dot)*std::cos(mPose_vec(POSE_THETA_IDX));
            double y_delta2 = -(odom_twist.x_dot/odom_twist.theta_dot)*std::cos(mPose_vec(POSE_THETA_IDX) + odom_twist.theta_dot);
            arma::colvec delta{t_delta, x_delta1 + x_delta2, y_delta1 + y_delta2};

            arma::colvec predict_pose = mPose_vec + delta;
            predict_pose(POSE_THETA_IDX) = turtlelib::normalize_angle(predict_pose(POSE_THETA_IDX));
            return predict_pose;
        }
    }

    arma::mat EKF::predict_pose_cov(turtlelib::Twist2D odom_twist) const
    {
        arma::mat A;
        arma::colvec zero_col(3, arma::fill::zeros);
        if(turtlelib::almost_equal(odom_twist.theta_dot, 0.0)){
            double x_delta = -odom_twist.x_dot*std::sin(mPose_vec(POSE_THETA_IDX));
            double y_delta = odom_twist.x_dot*std::cos(mPose_vec(POSE_THETA_IDX));
            arma::colvec delta{0.0, x_delta, y_delta};
            arma::mat temp = join_rows(delta, zero_col, zero_col);
            A = temp + arma::mat(3, 3, arma::fill::eye);
        } else {
            double x_delta1 = -(odom_twist.x_dot/odom_twist.theta_dot)*std::cos(mPose_vec(POSE_THETA_IDX));
            double x_delta2 = (odom_twist.x_dot/odom_twist.theta_dot)*std::cos(mPose_vec(POSE_THETA_IDX) + odom_twist.theta_dot);
            double y_delta1 = -(odom_twist.x_dot/odom_twist.theta_dot)*std::sin(mPose_vec(POSE_THETA_IDX));
            double y_delta2 = (odom_twist.x_dot/odom_twist.theta_dot)*std::sin(mPose_vec(POSE_THETA_IDX) + odom_twist.theta_dot);
            arma::colvec delta{0.0, x_delta1 + x_delta2, y_delta1 + y_delta2};
            arma::mat temp = join_rows(delta, zero_col, zero_col);
            A = temp + arma::mat(3, 3, arma::fill::eye);
        }

        return A*mPose_vec*A.t(); //+ Q TODO
    }
}