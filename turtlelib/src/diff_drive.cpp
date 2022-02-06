#include "turtlelib/diff_drive.hpp"
#include <stdexcept>
#include <iostream>

namespace turtlelib {

    ////////////////////// DIFF DRIVE /////////////////////

    //Empty DiffDrive
    DiffDrive::DiffDrive()
        : DiffDrive{0.1, 0.1, 0.0, 0.0, 0.0}
    {
    }

    //DiffDrive with wheel configuration
    DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
        : DiffDrive{wheel_track, wheel_radius, 0.0, 0.0, 0.0}
    {
    }

    //DiffDrive with init location
    DiffDrive::DiffDrive(double x, double y, double theta)
        : DiffDrive{0.1, 0.1, x, y, theta}
    {
    }

    //DiffDrive with wheel configuration and init location
    DiffDrive::DiffDrive(double wheel_track, double wheel_radius, 
        double theta, double x, double y)
        : mWheel_track{wheel_track}
        , mWheel_rad{wheel_radius}
        , mAng_rad{normalize_angle(theta)}
        , mX_m{x}
        , mY_m{y}
        , mLw_rad{0.0}
        , mRw_rad{0.0}
    {
    }

    //DiffDrive get theta
    double DiffDrive::theta() const
    {
        return mAng_rad;
    }

    //DiffDrive get location
    Vector2D DiffDrive::location() const
    {
        return {mX_m, mY_m};
    }

    //DiffDrive get wheel track
    double DiffDrive::wheel_track() const
    {
        return mWheel_track;
    }

    //DiffDrive get wheel radius
    double DiffDrive::wheel_radius() const
    {
        return mWheel_rad;
    }

    //DiffDrive get left wheel position
    double DiffDrive::left_wheel_pos() const
    {
        return mLw_rad;
    }

    //DiffDrive get right wheel position
    double DiffDrive::right_wheel_pos() const
    {
        return mRw_rad;
    }

    //DiffDrive set_location
    DiffDrive & DiffDrive::set_configuration( const double theta, const double x, const double y)
    {
        mAng_rad = normalize_angle(theta);
        mX_m = x;
        mY_m = y;

        return *this;
    }

    //DiffDrive apply fw kinematics
    DiffDrive & DiffDrive::apply_fw_kin(const double left_pos, const double right_pos)
    {
        // Get wheel velocities
        double left_vel = left_pos - mLw_rad;
        double right_vel = right_pos - mRw_rad;
        std::cout << "NEW TEST!!!!!!" << std::endl;
        std::cout << "LEFT POS " << left_pos << std::endl;
        std::cout << "RIGHT POS " << right_pos << std::endl;
        std::cout << "LEFT WHEEL " << mLw_rad << std::endl;
        std::cout << "RIGHT WHEEL " << mRw_rad << std::endl;
        std::cout << "LEFT VEL " << left_vel << std::endl;
        std::cout << "RIGHT VEL " << right_vel << std::endl;
        // Get Body Twist
        double theta_dot = (mWheel_rad/mWheel_track)*(right_vel - left_vel);
        double x_dot = (0.5*mWheel_rad)*(left_vel + right_vel);
        std::cout << "THETA_DOT " << theta_dot << std::endl;
        std::cout << "X_DOT " << x_dot << std::endl;
        Twist2D body_twist{theta_dot, x_dot, 0.0};

        //Integrate Twist
        Transform2D motion = integrate_twist(body_twist);
        std::cout << "MOTION " << motion << std::endl;
        //Convert motion to world frame
        Twist2D qb{motion.rotation(), motion.translation().x, motion.translation().y};
        std::cout << "QB " << qb << std::endl;
        Twist2D q = Transform2D{mAng_rad}(qb); //Apply Adjunct
        std::cout << "Q " << q <<std::endl;
        //Update configuration
        mLw_rad = normalize_angle(left_pos);
        mRw_rad = normalize_angle(right_pos);
        mAng_rad = mAng_rad + q.theta_dot;
        mX_m = mX_m + q.x_dot;
        mY_m = mY_m + q.y_dot;
        std::cout << std::endl;
        return *this;
    }

    //DiffDrive calculate inv kinematics
    std::vector<double> DiffDrive::cal_inv_kin(const Twist2D & twist_b) const
    {
        if (!almost_equal(twist_b.y_dot, 0.0))
        {
            throw std::logic_error("Invalid twist has been provided");
        } 
        double left_vel = (-0.5*mWheel_track*twist_b.theta_dot + twist_b.x_dot)/mWheel_rad;
        double right_vel = (0.5*mWheel_track*twist_b.theta_dot + twist_b.x_dot)/mWheel_rad;
        return std::vector<double>{left_vel, right_vel};
    }

    //DiffDrive apply twist
    DiffDrive & DiffDrive::apply_twist(const Twist2D & twist_b, const double dt)
    {
        if (!almost_equal(twist_b.y_dot, 0.0))
        {
            throw std::logic_error("Invalid twist has been provided");
        }
        //Get wheel velocities
        std::vector<double> velocities = cal_inv_kin(twist_b);
        
        //Integrate Twist
        Transform2D motion = integrate_twist(twist_b);

        //Convert motion to world frame
        Twist2D qb{motion.rotation(), motion.translation().x, motion.translation().y};
        Twist2D q = Transform2D{mAng_rad}(qb); //Apply Adjunct

        //Update configuration
        mLw_rad = normalize_angle(mLw_rad + velocities.at(0)*dt);
        mRw_rad = normalize_angle( mRw_rad + velocities.at(1)*dt);
        mAng_rad = mAng_rad + q.theta_dot;
        mX_m = mX_m + q.x_dot;
        mY_m = mY_m + q.y_dot;

        return *this;
    }

    //DiffDrive print fcn
    std::ostream & operator<<(std::ostream & os, const DiffDrive & dd)
    {
        os << "loc (" << dd.mX_m << ", " << dd.mY_m << ", " << dd.mAng_rad << ") ";
        os << "radius: " << dd.mWheel_rad << " track: " << dd.mWheel_track;
        return os;
    }
}