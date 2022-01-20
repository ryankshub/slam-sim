#include "rigid2d.hpp"
#include <iostream>
#include <string>

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
        std::string x_str;
        std::string y_str;
        is >> x_str;
        is >> y_str;

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

        try 
        {
            double x = std::stod(x_str);
            v.x = x;
            double y = std::stod(y_str);
            v.y = y;
        } catch (const std::invalid_argument& e) {
            std::cout << e.what() << std::endl;
            std::cout << "Improper Format: ";
            std::cout << "Format doubles as [x y] or x y" << std::endl;
        }

        return is;
    }

}
