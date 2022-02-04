#include "turtlelib/diff_drive.hpp"


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
    double DiffDrive::get_theta()
    {
        return mAng_rad;
    }

    //DiffDrive get location
    Vector2D DiffDrive::get_location()
    {
        return {mX_m, mY_m};
    }

    //DiffDrive get wheel track
    double DiffDrive::get_wheel_track()
    {
        return mWheel_track;
    }

    //DiffDrive get wheel radius
    double DiffDrive::get_wheel_radius()
    {
        return mWheel_rad;
    }

    //DiffDrive get left wheel position
    double DiffDrive::get_left_wheel_pos()
    {
        return mLw_rad;
    }

    //DiffDrive get right wheel position
    double DiffDrive::get_right_wheel_pos()
    {
        return mRw_rad;
    }

    //DiffDrive set_location
    DiffDrive & DiffDrive::set_location( const double theta, const double x, const double y)
    {
        mAng_rad = normalize_angle(theta);
        mX_m = x;
        mY_m = y;

        return *this;
    }

    //DiffDrive set wheel config
    DiffDrive & DiffDrive::set_wheel_config( const double wheel_track, const double wheel_radius)
    {
        mWheel_track = wheel_track;
        mWheel_rad = wheel_radius;
        return *this;
    }

    //DiffDrive apply fw kinematics
    DiffDrive & apply_fw_kinematics(const double left_pos, const double right_pos)
    {
        return *this;
    }

    //DiffDrive calculate inv kinematics
    std::vector<double> DiffDrive::cal_inv_kinematics(const Twist2D & twist_b)
    {
        return std::vector<double>{};
    }

    //DiffDrive apply twist
    DiffDrive & DiffDrive::apply_twist(const Twist2D & twist_b, const double dt = 1.0)
    {
        return *this;
    }

    //DiffDrive print fcn
    std::ostream & operator<<(std::ostream & os, const DiffDrive & dd)
    {
        os << "loc (" << mX_m << ", " << mY_m << ", " << mAng_rad << ") ";
        os << "radius: " << mWheel_rad << " track: " << mWheel_track;
        return os;
    }
}