#include "nuslam/ekflib.hpp"

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
        : mPose_vec(std::vector<double>{theta, x, y})
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
        mPose_vec = arma::colvec{std::vector{theta, x, y}};
        mPose_cov = arma::mat(3, 3, arma::fill::zeros);
    }

    //Sef current pose with angle and vector
    void EKF::set_current_pose(double theta, turtlelib::Vector2D pt)
    {
        mPose_vec = arma::colvec(std::vector{theta, pt.x, pt.y});
        mPose_cov = arma::mat(3, 3, arma::fill::zeros);
    }

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
}