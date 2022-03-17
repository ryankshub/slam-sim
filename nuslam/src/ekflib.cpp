#include "nuslam/ekflib.hpp"
#include <cmath>

namespace EKF_DD
{
    //Empty EKF
    EKF::EKF()
        : EKF{0.0, 0.0, 0.0, arma::mat(3,3,arma::fill::eye), arma::mat(2,2,arma::fill::eye)}
    {
    }

    //EKF with robot pose
    EKF::EKF(double theta, double x, double y)
        : EKF{theta, x, y, arma::mat(3,3,arma::fill::eye), arma::mat(2,2,arma::fill::eye)}
    {      
    }

    //EKF with robot pose w/ Q and Rcustom option
    EKF::EKF(double theta, double x, double y, const arma::mat & Q, const arma::mat & R)
        : mPose_vec{turtlelib::normalize_angle(theta), x, y}
        , mPose_cov(3, 3, arma::fill::zeros)
        , mQ_cov{Q}
        , mR_cov{R}
        , mLandmark_pose_map{std::map<int, std::tuple<double, double>>{}}
        , mLandmark_cov_map{std::map<int, std::tuple<double, double, double, double>>{}}
    {      
    }

    //Set current pose with 3 elem point
    void EKF::set_current_pose(double theta, double x, double y)
    {
        mPose_vec = arma::colvec{turtlelib::normalize_angle(theta), x, y};
        mPose_cov = arma::mat(3, 3, arma::fill::zeros);
    }

    //Sef current pose with angle and vector
    void EKF::set_current_pose(double theta, turtlelib::Vector2D pt)
    {
        mPose_vec = arma::colvec{turtlelib::normalize_angle(theta), pt.x, pt.y};
        mPose_cov = arma::mat(3, 3, arma::fill::zeros);
    }

    //Getters

    arma::colvec EKF::get_current_pose() const
    {
        return mPose_vec;
    }

    arma::colvec EKF::build_state_vector(int id, arma::colvec pose_vec) const
    {
        std::tuple<double, double> landmark_pt = mLandmark_pose_map.at(id);
        double mx = std::get<0>(landmark_pt); //0 is LANDMARK X IDX
        double my = std::get<1>(landmark_pt); //1 is LANDMARK Y IDX
        arma::colvec landcol{mx, my};
        arma::colvec statevec = arma::join_cols(pose_vec, landcol);
        return statevec;
    }

    std::map<int, std::tuple<double, double>> EKF::get_current_landmarks() const
    {
        return mLandmark_pose_map;
    }

    bool EKF::is_new_landmark(int id) const
    {
        return (mLandmark_pose_map.count(id) == 0) && (mLandmark_cov_map.count(id) == 0);
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

        return A*mPose_cov*A.t() + mQ_cov;
    }

    // Initiate Landmark
    void EKF::initiate_landmark(int id, double range, double bearing, arma::colvec predict_state)
    {
        double mx = predict_state(POSE_X_IDX) + range*std::cos(bearing + predict_state(POSE_THETA_IDX));
        double my = predict_state(POSE_Y_IDX) + range*std::sin(bearing + predict_state(POSE_THETA_IDX));

        std::tuple<double, double> landmark_pt = std::make_tuple(mx, my);
        mLandmark_pose_map[id] = landmark_pt;
        std::tuple<double, double, double, double> landmark_cov = std::make_tuple(100.0, 0.0, 0.0, 100.0);
        mLandmark_cov_map[id] = landmark_cov;
    }

    // Compute Est 
    arma::colvec EKF::compute_est_measurement(int id, arma::colvec predict_state) const
    {
        std::tuple<double, double> landmark_pt = mLandmark_pose_map.at(id);
        double mx = std::get<0>(landmark_pt); //0 is LANDMARK X IDX
        double my = std::get<1>(landmark_pt); //1 is LANDMARK Y IDX
        double x_comp = mx - predict_state(POSE_X_IDX);
        double y_comp = my - predict_state(POSE_Y_IDX);
        double est_range = std::sqrt(std::pow(x_comp, 2.0) + std::pow(y_comp, 2.0));
        double est_bearing = turtlelib::normalize_angle(std::atan2(y_comp, x_comp) - predict_state(POSE_THETA_IDX));
        arma::colvec est_meas{est_range, est_bearing};
        return est_meas;
    }

    // Build Matrix
    arma::mat EKF::build_sigma_mat(int id, arma::mat predicted_cov) const
    {
        arma::colvec three_zero_col(3, arma::fill::zeros);
        arma::colvec two_zero_col(2, arma::fill::zeros);
        arma::mat top_sigma = arma::join_rows(predicted_cov, three_zero_col, three_zero_col);
        std::tuple<double, double, double, double> landmark_cov = mLandmark_cov_map.at(id);
        arma::mat mat_cov{{std::get<0>(landmark_cov), std::get<1>(landmark_cov)}, {std::get<2>(landmark_cov), std::get<3>(landmark_cov)}};
        arma::mat bottom_sigma = arma::join_rows(two_zero_col, two_zero_col, two_zero_col,mat_cov);
        arma::mat sigma_mat = arma::join_cols(top_sigma, bottom_sigma);
        return sigma_mat;
    }

    arma::mat EKF::build_H_mat(int id, arma::colvec predict_state) const
    {
        std::tuple<double, double> landmark_pt = mLandmark_pose_map.at(id);
        double mx = std::get<0>(landmark_pt);//0 is LANDMARK X IDX
        double my = std::get<1>(landmark_pt);//1 is LANDMARK Y IDX
        double x_comp = mx - predict_state(POSE_X_IDX);
        double y_comp = my - predict_state(POSE_Y_IDX);
        double d = std::pow(x_comp, 2.0) + std::pow(y_comp, 2.0);
        double sqrt_d = std::sqrt(d);

        arma::rowvec top_H_mat {0, -x_comp/sqrt_d, -y_comp/sqrt_d, x_comp/sqrt_d, y_comp/sqrt_d};
        arma::rowvec bottom_H_mat {-1, y_comp/d, -x_comp/d, -y_comp/d, x_comp/d};
        arma::mat H_mat = arma::join_cols(top_H_mat, bottom_H_mat);
        return H_mat;
    }

    // Compute Kalman
    arma::mat EKF::compute_kalman_gain(arma::mat H_mat, arma::mat sigma_mat) const
    {
        arma::mat inner_prod = H_mat*sigma_mat*H_mat.t() + mR_cov;
        arma::mat kalman_gain = sigma_mat*H_mat.t()*inner_prod.i();
        return kalman_gain;
    }

    //Update functions
    void EKF::update_poses(int id, arma::colvec predicted_pose, arma::mat kalman_gain, arma::colvec est_measure, arma::colvec real_measure)
    {
        //Build state vector
        arma::colvec state_vec = build_state_vector(id, predicted_pose);
        //Get updated 
        arma::colvec updated_state_vec = state_vec + (kalman_gain*(real_measure - est_measure));
        //Parse it out
        // Update state pose
        mPose_vec = updated_state_vec.subvec(0,2);
        mPose_vec(POSE_THETA_IDX) = turtlelib::normalize_angle(mPose_vec(POSE_THETA_IDX));
        double mx = updated_state_vec(3);
        double my = updated_state_vec(4);
        //Update landmark pose
        mLandmark_pose_map[id] = std::make_tuple(mx, my);

    }

    void EKF::update_covarances(int id, arma::mat kalman_gain, arma::mat H_mat, arma::mat sigma_mat)
    {
        arma::mat identity(5,5, arma::fill::eye);
        arma::mat updated_cov = (identity - (kalman_gain*H_mat))*sigma_mat;
        //Parse it out
        //Update state covariance
        mPose_cov = updated_cov.submat(0, 0, 2, 2);
        double elem0 = updated_cov(3, 3);
        double elem1 = updated_cov(3, 4);
        double elem2 = updated_cov(4, 3);
        double elem3 = updated_cov(4, 4);
        //Update landmark covarience
        mLandmark_cov_map[id] = std::make_tuple(elem0, elem1, elem2, elem3);
    }

    // Polar conversions
    double get_polar_range(double rel_x, double rel_y)
    {
        return std::sqrt(std::pow(rel_x, 2.0) + std::pow(rel_y, 2.0));
    }

    double get_polar_bearing(double rel_x, double rel_y)
    {
        return std::atan2(rel_y, rel_x);
    }
   
}