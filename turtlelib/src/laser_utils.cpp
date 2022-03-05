#include "turtlelib/laser_utils.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib {

///////////////////////LASER UTIL FCN//////////////////
    //Checks obstacles intersection
    bool check_obs_intersection(double x1, double y1, 
                                double x2, double y2,
                                double x0, double y0, double rad0,
                                Vector2D & pt)
    {
        //Remap points to obstacle's frame
        x1 = x1 - x0;
        x2 = x2 - x0;
        y1 = y1 - y0;
        y2 = y2 - y0;
        //Define factors
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr_sq = std::pow(dx, 2.0) + std::pow(dy, 2.0);
        double D = x1*y2 - x2*y1;

        //Calculate discriminant
        double discriminant = std::pow(rad0, 2.0)*dr_sq - std::pow(D, 2.0);
        //Check if not intersection
        if (discriminant <= 0)
        {
            return false;
        }

        // Calculate points
        double sgn_dy = 1.0;
        if (dy < 0)
        {
            sgn_dy = -1.0;
        }

        double first_x = (D*dy + sgn_dy * dx * std::sqrt(discriminant))/dr_sq;
        double second_x =  (D*dy - sgn_dy * dx * std::sqrt(discriminant))/dr_sq;

        double first_y = (-D*dx + std::abs(dy)*std::sqrt(discriminant))/dr_sq;
        double second_y = (-D*dx - std::abs(dy)*std::sqrt(discriminant))/dr_sq;

        //Determine which point is closest 
        double first_dist = std::sqrt(std::pow(first_x - x1, 2.0) + std::pow(first_y - y1, 2.0));
        double second_dist = std::sqrt(std::pow(second_x - x1, 2.0) + std::pow(second_y - y1, 2.0));

        //Populate closest point
        if (first_dist > second_dist){
            pt.x = second_x + x0;
            pt.y = second_y + y0;
        } else {
            pt.x = first_x + x0;
            pt.y = first_y + y0;
        }

        //Check new point is on line segment
        if (!((x1 <= pt.x) && (pt.x <= x2)) && !((x1 >= pt.x) && (pt.x >= x2)))
        {
            pt.x = 0.0;
            pt.y = 0.0;
            return false;
        }

        return true;
    }

    ///Checks Wall Intersection
    bool check_wall_intersection(double x1, double y1, 
                                 double x2, double y2,
                                 double xW1, double yW1,
                                 double xW2, double yW2,
                                 Vector2D & pt)
    {
        //Get cross product
        Vector2D line{x2-x1, y2-y1}; //p
        Vector2D wall{xW2-xW1, yW2-yW1}; //q
        double cross_linewall = cross(line, wall);
        // A cross product of 0 means the vectors
        // are either parallel or collinear
        // This function won't have a collinear case
        // so a 0 here, means no intersection
        if (almost_equal(cross_linewall, 0.0))
        {
            return false;
        }

        Vector2D diff{xW1 - x1, yW1 - y1};
        double line_scale = cross(diff, wall) / cross_linewall;
        double wall_scale = cross(diff, line) / cross_linewall;
        // Check if line_scale and wall_scale are in range [0,1]
        // if so, intersection! Populate pt p
        if (((almost_equal(line_scale, 0.0) || line_scale > 0.0) && 
            (almost_equal(line_scale, 1.0) || line_scale < 1.0)) &&
            ((almost_equal(wall_scale, 0.0) || wall_scale > 0.0) && 
            (almost_equal(wall_scale, 1.0) || wall_scale < 1.0)))
        {
            pt = line_scale*line + Vector2D{x1, y1};
            return true;
        }

        return false;
    }

}