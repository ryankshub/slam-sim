#include "include/rigid2d.hpp"
#include <iostream>
// Frame main
// Main file to test rigid2D library

using namespace turtlelib;

int main()
{
    //Create Transfromation T_ab and T_bc
    Transform2D T_ab;
    Transform2D T_bc;

    //Get user input
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;

    //Produce transformation matrix output
    std::cout << "T_{a,b}: " << T_ab << std::endl;
    Transform2D T_ba = T_ab.inv();
    std::cout << "T_{b,a}: " << T_ba << std::endl;
    std::cout << "T_{b,c}: " << T_bc << std::endl;
    Transform2D T_cb = T_bc.inv();
    std::cout << "T_{c,b}: " << T_cb << std::endl;
    Transform2D T_ac = T_ab*T_bc;
    std::cout << "T_{a,c}: " << T_ac << std::endl;
    Transform2D T_ca = T_ac.inv();
    std::cout << "T_{c,a}: " << T_ca << std::endl;

    //Create Vector2D B
    Vector2D v_b;

    //Get user input
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v_b;

    // Produce vector output
    Vector2D v_bhat = normalize(v_b);
    std::cout << "v_bhat: " << v_bhat << std::endl;
    Vector2D v_a = T_ab(v_b);
    std::cout << "v_a: " << v_a << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    Vector2D v_c = T_cb(v_b);
    std::cout << "v_c: " << v_c << std::endl;

    //Create Twist2D B
    Twist2D V_b;

    //Get user input
    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;

    // Produce twist output
    Twist2D V_a = T_ab(V_b);
    std::cout << "V_a " << V_a << std::endl;
    std::cout << "V_b " << V_b << std::endl;
    Twist2D V_c = T_cb(V_b);
    std::cout << "V_c " << V_c << std::endl;



    return (0);
}