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
        std::string x_str;
        std::string y_str;
        is >> x_str;
        is >> y_str;

        //Check for bracket syntax
        if (x_str.front() == '[')
        {
            if (y_str.back() != ']')
            {
                std::cout << "Improper Format: ";
                std::cout << "Format doubles as [x y] or x y" << std::endl;
            } else {
                x_str.erase(0,1);
                y_str.pop_back();
            }
        }

        //Attempt to convert to a double.
        try 
        {
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            v.x = x;
            v.y = y;
        } catch (const std::invalid_argument& e) {
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

    //Print function for translation
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.mAng_rad);
        os << " x: " << tf.mVec.x;
        os << " y: " << tf.mVec.y;
        return os;
    }

}
