#include "turtlelib/laser_utils.hpp"
#include <cmath>

namespace turtlelib {

///////////////////////LASER UTIL FCN//////////////////
    //Checks obstacles intersection
    bool check_obs_intersection(double x1, double x2, 
                                double y1, double y2,
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

        return true;
    }

    ///Checks Wall Intersection
    bool check_wall_intersection(double x1, double x2, 
                                 double y1, double y2,
                                 double ang_rad,
                                 double xW, double yW,
                                 Vector2D & pt)
    {
        // Processing y-wall
        if (almost_equal(xW, 0.0)) {
            //Check if intersection
            if ((std::abs(yW) >= std::abs(y1)) && (std::abs(yW) <= std::abs(y2)))
            {
                pt.x = yW/std::tan(ang_rad);
                pt.y = yW;
                return true;
            }

            return false;

        // Processing x-walls
        } else if (almost_equal(yW, 0.0)) {
            if ((std::abs(xW) >= std::abs(x1)) && (std::abs(xW) <= std::abs(x2)))
            {
                pt.x = xW;
                pt.y = xW*std::tan(ang_rad);
                return true;
            }

            return false;
        }
    }

}