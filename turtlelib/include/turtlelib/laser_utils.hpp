#ifndef LASER_UTILS_INCLUDE_GUARD_HPP
#define LASER_UTILS_INCLUDE_GUARD_HPP
/// \file
/// \brief Utility functions for lidar sensor

//Project Includes
#include "turtlelib/rigid2d.hpp"

// Standard Includes

// 3rd-party Includes

namespace turtlelib {

    /// \brief Checks if line intersects cylinder obstacle
    /// If the line intersects, populates the nearest point
    /// to robot.
    /// \param x1 - x value of first point on line
    /// \param y1 - y value of first point on line
    /// \param x2 - x value of second point on line
    /// \param y2 - y value of second point on line
    /// \param x0 - x value of obstacle
    /// \param y0 - y value of obstacle
    /// \param rad0 - radius of the obstacle
    /// \param pt - pt to populate if line intersects
    /// \return True if line intersects obstacle
    bool check_obs_intersection(double x1, double y1, 
                                double x2, double y2,
                                double x0, double y0, double rad0,
                                Vector2D & pt);

    /// \brief Checks if line intersects wall
    /// If the line intersects, populates the nearest point
    /// to robot.
    /// \param x1 - x value of first point on line
    /// \param y1 - y value of first point on line
    /// \param x2 - x value of second point on line
    /// \param y2 - y value of second point on line
    /// \param ang_rad - angle of laser line (radians)
    /// \param sizeW - size of the wall (should be > 0)
    /// \param xW - x value of the wall
    /// \param yW - y value of the wall
    /// \param pt - pt to populate if line intersects
    /// \return True if line intersects obstacle
    bool check_wall_intersection(double x1, double y1, 
                                 double x2, double y2,
                                 double ang_rad, double sizeW,
                                 double xW, double yW,
                                 Vector2D & pt); 
}

#endif