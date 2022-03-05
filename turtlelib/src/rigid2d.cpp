#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <stdexcept>

namespace turtlelib
{

    /////////////////// UTILITY FUNCTION ////////////////
    double normalize_angle(double rad)
    {
        double new_ang;
        new_ang = fmod(rad + PI, 2*PI);
        if (new_ang < 0)
        {
            new_ang += 2*PI;
        }
        new_ang -= PI;
        if (almost_equal(new_ang, -PI))
        {
            return PI;
        }
        return new_ang;
    
    }

    /////////////////// VECTOR2D SECTION ////////////////

    //Adding += function for Vector2D
    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        double new_x = x + rhs.x;
        double new_y = y + rhs.y;

        x = new_x;
        y = new_y;

        return *this;
    }

    //Subtracting -= funciton for Vector2D
    Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        double new_x = x - rhs.x;
        double new_y = y - rhs.y;

        x = new_x;
        y = new_y;

        return *this;
        
    }

    //Multipling *= function for Vector2D
    Vector2D & Vector2D::operator*=(const double & rhs)
    {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    //Printing function for Vector2D
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    //Read function for Vector2D
    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        //Read from the input stream
        std::string user_input;
        std::getline(is, user_input);
        //Check for bracket syntax
        if (user_input.front() == '[')
        {
            if (user_input.back() != ']')
            {
                std::cout << "Improper Format: ";
                std::cout << "Format doubles as [x y] or x y" << std::endl;
                return is;
            } else {
                user_input.erase(0,1);
                user_input.pop_back();
            }
        }

        //Attempt to convert to a double.
        try 
        {
            std::string x_str = user_input.substr(0, user_input.find(" "));
            std::string y_str = user_input.substr(user_input.find(" "));
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            v.x = x;
            v.y = y;
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
            std::cout << "Improper Format: ";
            std::cout << "Format doubles as [x y] or x y" << std::endl;
        }

        return is;
    }

    //Normalize Vector function
    Vector2D normalize(const Vector2D & v)
    {
        double mag = std::sqrt(pow(v.x,2.0) + pow(v.y,2.0));
        Vector2D vec; 
        vec.x = v.x/mag;
        vec.y = v.y/mag;
        return vec;
    }

    //Adding+ Function
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        lhs += rhs;
        return lhs;
    }

    //Subtracting- Function
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    //Multiply* by scalar Function
    Vector2D operator*(const double lhs, Vector2D rhs)
    {
        rhs *= lhs;
        return rhs;
    }

    //Multiply* by scalar Function
    Vector2D operator*(Vector2D lhs, const double rhs)
    {
        lhs *= rhs;
        return lhs;
    }

    //dot function
    double dot(const Vector2D & lvec, const Vector2D & rvec)
    {
        return (lvec.x*rvec.x) + (lvec.y*rvec.y);
    }

    //cross product function
    double cross(const Vector2D & lvec, const Vector2D & rvec)
    {
        return lvec.x*rvec.y - lvec.y*rvec.x;
    }

    //magnitude function
    double magnitude(const Vector2D & vec)
    {
        return std::sqrt(pow(vec.x,2.0) + pow(vec.y,2.0));
    }

    //angle function
    double angle(const Vector2D & lvec, const Vector2D & rvec)
    {
        double lmag = magnitude(lvec);
        double rmag = magnitude(rvec);
        // Check if valid magnitudes
        if (almost_equal(lmag, 0.0))
        {
            throw std::invalid_argument{"First vector has a magnitude of zero"};
        } else if (almost_equal(rmag, 0.0)) {
            throw std::invalid_argument{"Second vector has a magnitude of zero"};
        }
        double dot_prod = dot(lvec, rvec);
        double angle = dot_prod/(lmag * rmag);
        angle = std::acos(angle);
        return angle;
    }


    ///////////////// TWIST 2D ////////////////////

    //Print function for Twist2D
    std::ostream & operator<<(std::ostream & os, Twist2D & t)
    {
        os << "[" << t.theta_dot;
        os << " " << t.x_dot;
        os << " " << t.y_dot;
        os << "]";
        return os;
    }

    //Read function for Twist2D
    std::istream & operator>>(std::istream & is, Twist2D & t)
    {
        //Read from the input stream
        std::string user_input;
        std::getline(is, user_input);

        //Check for bracket syntax
        if (user_input.front() == '[')
        {
            if (user_input.back() != ']')
            {
                std::cout << "Improper Format: ";
                std::cout << "Format doubles as [t x y] or t x y" << std::endl;
                return is;
            } else {
                user_input.erase(0,1);
                user_input.pop_back();
            }
        }

        //Attempt to convert to a double.
        try 
        {
            std::string theta_str = user_input.substr(0, user_input.find(" "));
            user_input.erase(0, user_input.find(" ")+1);
            std::string x_str = user_input.substr(0, user_input.find(" "));
            std::string y_str = user_input.substr(user_input.find(" "));
            double theta_dot = std::stod(theta_str);
            double x_dot = std::stod(x_str);
            double y_dot = std::stod(y_str);
            t.theta_dot = theta_dot;
            t.x_dot = x_dot;
            t.y_dot = y_dot;
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
            std::cout << "Improper Format: ";
            std::cout << "Format doubles as [t x y] or t x y" << std::endl;
        }

        return is;
    }

    ////////////// TRANSFORM2D SECTION //////////////////
    //Transform2D Identity 
    Transform2D::Transform2D() 
        : mVec()
        , mAng_rad(0)
    {     
    }

    //Transform2D with only trans
    Transform2D::Transform2D(Vector2D trans) 
        : mVec(trans)
        , mAng_rad(0)
    {
    }

    //Transform2D only rotation
    Transform2D::Transform2D(double radians)
        : mVec()
        , mAng_rad(radians)
    {
    }
    
    //Transform2 Full constrcutor
    Transform2D::Transform2D(Vector2D trans, double radians)
        : mVec(trans)
        , mAng_rad(radians)
    {
    }

    //Apply operation to vector
    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D new_vec;
        new_vec.x = (v.x*std::cos(mAng_rad)) - (v.y*std::sin(mAng_rad)) + mVec.x;
        new_vec.y = (v.x*std::sin(mAng_rad)) + (v.y*std::cos(mAng_rad)) + mVec.y;
        return new_vec;
    }

    //Apply adjoint to twist
    Twist2D Transform2D::operator()(const Twist2D & t) const
    {
        Twist2D new_twist;
        new_twist.theta_dot = t.theta_dot;
        new_twist.x_dot = mVec.y*t.theta_dot + std::cos(mAng_rad)*t.x_dot - std::sin(mAng_rad)*t.y_dot;
        new_twist.y_dot = -mVec.x*t.theta_dot + std::sin(mAng_rad)*t.x_dot + std::cos(mAng_rad)*t.y_dot;
        return new_twist;
    }

    //Return inverse of transform
    Transform2D Transform2D::inv() const
    {
        Vector2D vec;
        vec.x = -mVec.x*std::cos(mAng_rad) - mVec.y*std::sin(mAng_rad);
        vec.y = -mVec.y*std::cos(mAng_rad) + mVec.x*std::sin(mAng_rad);
        double inv_ang = -mAng_rad;
        Transform2D T{vec, inv_ang};
        return T;
    }

    //Transform operator for matrix multiplication
    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        double new_ang = mAng_rad + rhs.mAng_rad;
        double new_x = std::cos(mAng_rad)*rhs.mVec.x - std::sin(mAng_rad)*rhs.mVec.y + mVec.x;
        double new_y = std::sin(mAng_rad)*rhs.mVec.x + std::cos(mAng_rad)*rhs.mVec.y + mVec.y;

        mAng_rad = new_ang;
        mVec.x = new_x;
        mVec.y = new_y; 

        return *this;
    }

    //Get translation
    Vector2D Transform2D::translation() const
    {
        return mVec;
    }

    //Get rotation
    double Transform2D::rotation() const
    {
        return mAng_rad;
    }

    //Print function for transform2d
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        os << "deg: " << rad2deg(tf.mAng_rad);
        os << " x: " << tf.mVec.x;
        os << " y: " << tf.mVec.y;
        return os;
    }

    //Read function for transform2d
    std::istream & operator>>(std::istream & is, Transform2D & tf){
        // Get input
        std::string error_msg = R"(Format must be one of the following:
                                    'deg: deg_val x: x_val y: y_val'
                                    'deg_val x_val y_val
                                    'deg_val //seperated by newline
                                     x_val
                                     y_val)";
        std::string user_input;
        std::getline(is, user_input);

        //Determine format
        int format_mode = -1;
        if (user_input.find(' ') == std::string::npos){
            format_mode = 0; //Values are split by \n
        } else if (user_input.substr(0, user_input.find(' ')) == "deg:"){
            format_mode = 1; //Values are in program output
        } else {
            format_mode = 2; //Values are assumed to be split by ' '
        }

        // Parse format
        std::string input_d;
        std::string input_x;
        std::string input_y;

        switch(format_mode) {
            case(0):
                {
                    input_d = user_input;
                    std::getline(is, input_x);
                    std::getline(is, input_y);
                    break;
                }
            
            case(1):
                {
                    //Parse out degree val
                    int s_index = user_input.find(' ')+1;
                    int sublen = user_input.find(' ', s_index)-s_index;
                    input_d = user_input.substr(s_index, sublen);
                    user_input.erase(0, user_input.find(' ', s_index)+1);

                    //Parse out X val
                    s_index = user_input.find(' ')+1;
                    sublen = user_input.find(' ', s_index)-s_index;
                    input_x = user_input.substr(s_index, sublen);
                    user_input.erase(0, user_input.find(' ', s_index)+1);

                    //Parse out Y val
                    s_index = user_input.find(' ')+1;
                    sublen = user_input.find(' ', s_index)-s_index;
                    input_y = user_input.substr(s_index, sublen);
                    break;
                }
            
            case(2):
                {
                    //Parse out degree val
                    input_d = user_input.substr(0, user_input.find(' '));
                    user_input.erase(0, user_input.find(' ')+1);
                    //Parse out X val
                    input_x = user_input.substr(0, user_input.find(' '));
                    user_input.erase(0, user_input.find(' ')+1);
                    //Parse out Y val
                    input_y = user_input.substr(0, user_input.find(' '));
                    break;
                }
            
            default:
                std::cout << error_msg << std::endl;
                return is;

        }

        try{
            double angle = deg2rad(std::stod(input_d));
            double x = std::stod(input_x);
            double y = std::stod(input_y);
            Vector2D vec{x, y};
            Transform2D trans{vec, angle};
            tf = trans;
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
            std::cout << error_msg << std::endl;
        }

        return is;

    }

    //Transform multiplication
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        lhs *= rhs;
        return lhs;
    }

    //Transform integration
    Transform2D integrate_twist(const Twist2D & twist)
    {

        if (almost_equal(twist.theta_dot, 0.0))
        {
            return Transform2D{Vector2D{twist.x_dot, twist.y_dot}};
        }
        
        double ys = -twist.x_dot/twist.theta_dot;
        double xs = twist.y_dot/twist.theta_dot;
        Transform2D Tsb{Vector2D{xs, ys}};

        Transform2D Tss_p{twist.theta_dot};

        return Tsb.inv() * Tss_p * Tsb;
    }
}
