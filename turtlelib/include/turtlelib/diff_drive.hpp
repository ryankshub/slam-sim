#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of a differential drive wheeled mobile robot

//Project Includes
#include "turtlelib/rigid2d.hpp"

// Standard Includes
#include <vector>
#include <iosfwd> // contains forward definitions for iostream objects

// 3rd-party Includes

namespace turtlelib {
/// \brief Kinematics of 2-wheeled diff drive mobile robot
    class DiffDrive
    {
    public:
        /// \brief Create the default robot with .1m track and .1m radius
        /// \brief At location (0,0,0),
        DiffDrive();

        /// \brief Create a robot with custom wheel radius with variable track
        /// \param wheel_track - length of wheel axis between the 2 wheel center points (must be > 0)
        /// \param wheel_radius - radius of the wheel (must be > 0).
        DiffDrive(double wheel_track, double wheel_radius);

        /// \brief Creates a robot at established location with .1m track and
        /// \brief .1m radius
        /// \param x - x position of the robot (meters)
        /// \param y - y position of the robot (meters)
        /// \param theta - angle of the robot (radians, in range (-pi, pi])
        ///              - If angle is outside of range, it will be normalized
        DiffDrive(double theta, double x, double y);

        /// \brief Creates a robot at established location with given wheel track and wheel radius
        /// \param wheel_track - length of wheel axis between the 2 wheel center points (must be > 0)
        /// \param wheel_radius - radius of the wheel (must be > 0).
        /// \param x - x position of the robot (meters)
        /// \param y - y position of the robot (meters)
        /// \param theta - angle of the robot (radians, in range (-pi, pi]). 
        ///              - If angle is outside of range, it will be normalized
        DiffDrive(double wheel_track, double wheel_radius, 
            double theta, double x, double y);

        /// \brief returns the current angle of the robot
        double theta() const;

        /// \brief returns the current x-y location of the robot
        Vector2D location() const;

        /// \brief returns the distance between the robot's wheels
        double wheel_track() const;

        /// \brief returns the robot's wheel radius
        double wheel_radius() const;

        /// \brief returns the angle of the robot's left wheel (in radians, in range (-pi, pi])
        double left_wheel_pos() const;

        /// \brief returns the angle of the robot's right wheel (in radians, in range (-pi, pi])
        double right_wheel_pos() const;

        /// \brief sets the configuration of the robot
        /// \param x - x position of the robot (meters)
        /// \param y - y position of the robot (meters)
        /// \param theta - angle of the robot (radians, in range (-pi, pi] )
        DiffDrive & set_configuration( const double theta, const double x, const double y);

        /// \brief Given new wheel positions, updates the robot's location and angle
        /// \param left_pos - new angle of left wheel
        /// \param right_pos - new angle of the right wheel
        DiffDrive & apply_fw_kinematics(const double left_pos, const double right_pos);

        /// \brief Given a body twist, determine the wheel velocity to follow the twist
        /// \param twist_b - body twist robot attempts to follow
        /// \throw - logic_error if twist cannot be followed by robot
        /// \return a vector containing the [left_vel, right_vel]
        std::vector<double> cal_inv_kinematics(const Twist2D & twist_b) const;

        /// \brief Applies a body Twist to the robot, updating the robot's location and angle
        /// \param twist_b - body twist robot attempts to follow
        /// \param dt - time step to apply twist
        DiffDrive & apply_twist(const Twist2D & twist_b, const double dt = 1.0);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const DiffDrive & dd);
    private:
        //Wheel Configuration
        const double mWheel_track;
        const double mWheel_rad;
        //Robot Configuration
        double mAng_rad;
        double mX_m;
        double mY_m;
        double mLw_rad;
        double mRw_rad;
    };

    /// \brief prints a human readable version of the diff drive:
    /// An example output:
    /// loc: (x, y, theta) radius: r track: t
    /// \param os - an output stream
    /// \param dd - diff drive to print
    std::ostream & operator<<(std::ostream & os, const DiffDrive & dd);
}

#endif