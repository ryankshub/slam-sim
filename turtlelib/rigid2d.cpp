#include "rigid2d.hpp"
#include <iostream>
#include <string>
#include <cmath>

namespace turtlelib
{

    //overloading ostream
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "Vector: X " << v.x << " Y " << v.y;
        return os;
    }

    //over istream 
    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        //Read from the input stream
        std::string user_input;
        std::getline(std::cin, user_input);

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
        double new_x = std::cos(mAng_rad)*rhs.mVec.x + std::sin(mAng_rad)*rhs.mVec.y + mVec.x;
        double new_y = -std::sin(mAng_rad)*rhs.mVec.x + std::cos(mAng_rad)*rhs.mVec.y + mVec.y;

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
        std::getline(std::cin, user_input);

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
                    std::getline(std::cin, input_x);
                    std::getline(std::cin, input_y);
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

}
