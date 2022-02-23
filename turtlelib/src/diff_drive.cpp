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
        if(almost_equal(wheel_track, 0.0) || 
           almost_equal(wheel_radius, 0.0) || 
           wheel_track < 0.0 || 
           wheel_radius < 0.0)
        {
            throw std::invalid_argument("Wheel Track and Wheel radius must be greater than 0.0");
        }
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
    DiffDrive & DiffDrive::set_configuration(const double theta, const double x, const double y)
    {
        mAng_rad = normalize_angle(theta);
        mX_m = x;
        mY_m = y;

        return *this;
    }

    //DiffDrive set wheel config
    DiffDrive & DiffDrive::set_wheel_config(const double wheel_track, const double wheel_radius)
    {
        if(almost_equal(wheel_track, 0.0) || 
           almost_equal(wheel_radius, 0.0) || 
           wheel_track < 0.0 || 
           wheel_radius < 0.0)
        {
            throw std::invalid_argument("Wheel Track and Wheel radius must be greater than 0.0");
        }

        mWheel_track = wheel_track;
        mWheel_rad = wheel_radius;
        return *this;
    }

    //DiffDrive apply fw kinematics
    DiffDrive & DiffDrive::apply_fw_kin(const double left_pos, const double right_pos)
    {
        // Get Body Twist
        Twist2D body_twist = cal_fw_kin(left_pos, right_pos);

        //Integrate Twist
        Transform2D motion = integrate_twist(body_twist);

        //Convert motion to world frame
        Twist2D qb{motion.rotation(), motion.translation().x, motion.translation().y};

        Twist2D q = Transform2D{mAng_rad}(qb); //Apply Adjunct

        //Update configuration
        mLw_rad = normalize_angle(left_pos);
        mRw_rad = normalize_angle(right_pos);
        //Appling Eq 10 from Kinematics.pdf
        mAng_rad = mAng_rad + q.theta_dot;
        mX_m = mX_m + q.x_dot;
        mY_m = mY_m + q.y_dot;

        return *this;
    }

    //DiffDrive apply fw kinematics (velocity input)
    DiffDrive & DiffDrive::apply_fw_kin_vel(const double left_vel, const double right_vel)
    {
        // Get Body Twist
        Twist2D body_twist = cal_fw_kin_vel(left_vel, right_vel);

        //Integrate Twist
        Transform2D motion = integrate_twist(body_twist);

        //Convert motion to world frame
        Twist2D qb{motion.rotation(), motion.translation().x, motion.translation().y};
        Twist2D q = Transform2D{mAng_rad}(qb); //Apply Adjunct

        //Update configuration
        mLw_rad = normalize_angle(mLw_rad + left_vel);
        mRw_rad = normalize_angle(mRw_rad + right_vel);
        mAng_rad = mAng_rad + q.theta_dot;
        mX_m = mX_m + q.x_dot;
        mY_m = mY_m + q.y_dot;

        return *this;
    }

    //DiffDrive cal fw kinematics
    Twist2D DiffDrive::cal_fw_kin(const double left_pos, const double right_pos)
    {
        // Get wheel velocities
        double left_vel = left_pos - mLw_rad;
        double right_vel = right_pos - mRw_rad;
        // Get Body Twist
        //Eqs 7 and 8 from Kinematics.pdf
        double theta_dot = (mWheel_rad/mWheel_track)*(right_vel - left_vel);
        double x_dot = (0.5*mWheel_rad)*(left_vel + right_vel);
        return Twist2D{theta_dot, x_dot, 0.0};
    }

    //DiffDrive cal fw kinematics(velocity input)
    Twist2D DiffDrive::cal_fw_kin_vel(const double left_vel, const double right_vel)
    {
        // Get Body Twist
        double theta_dot = (mWheel_rad/mWheel_track)*(right_vel - left_vel);
        double x_dot = (0.5*mWheel_rad)*(left_vel + right_vel);
        return Twist2D{theta_dot, x_dot, 0.0};
    }

    //DiffDrive calculate inv kinematics
    std::vector<double> DiffDrive::cal_inv_kin(const Twist2D & twist_b) const
    {
        if (!almost_equal(twist_b.y_dot, 0.0))
        {
            throw std::logic_error("Invalid twist has been provided");
        } 
        // Equation 5 from Kinematics.pdf
        double left_vel = (-0.5*mWheel_track*twist_b.theta_dot + twist_b.x_dot)/mWheel_rad;

        // Equation 6 from Kinematics.pdf
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

    //Resolving Collision Function
    bool resolve_collision(DiffDrive & dd, double robot_radius, Vector2D obs_pose, double obs_radius)
    {
        //Get the distance between robot and obstacle
        Vector2D robot_pose = dd.location();
        Vector2D dist_vect = robot_pose - obs_pose;
        double distance = magnitude(dist_vect);
        double threshold = robot_radius + obs_radius;
        //Check for collision
        if (distance < threshold)
        {
            //Collision occured, displace robot
            //If robot and obstacle are on top of each other, jettison the obs right
            double vect_angle = 0.0;
            if (!(almost_equal(dist_vect.x, 0.0) && almost_equal(dist_vect.y, 0.0)))
            {
                vect_angle = std::atan2(dist_vect.y, dist_vect.x);
            }
            double new_x = threshold*std::cos(vect_angle) + obs_pose.x;
            double new_y = threshold*std::sin(vect_angle) + obs_pose.y;
            dd.set_configuration(dd.theta(), new_x, new_y);

            return true;
        }
        //No collision; do nothing
        return false;
    }
}