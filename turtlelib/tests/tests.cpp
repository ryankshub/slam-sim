// Turtlelib Testfile
// This file test function in the turtlelib namespace
// RKS

#define CATCH_CONFIG_MAIN //Tells Catch to provide a main()-only do in one cpp
#include "catch.hpp"
#include "include/rigid2d.hpp"
#include <cmath>

using namespace turtlelib;
static const double epsilon = 1.0e-12;


////////////////// TRANSFORM2D ////////////

/// TEST operator*= ///
TEST_CASE("Transform2D::operator*=, Self Reference", "[Transform2D]")
{
    //Build objects
    Transform2D T_id{};
    Vector2D vec_id{0,0};

    //Perform functions
    T_id *= T_id;

    //Grab results
    Vector2D res_vec = T_id.translation();
    double res_ang = T_id.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Identity Mul", "[Transform2D]")
{
    //Build objects
    Transform2D T_id{};
    Transform2D T_id2{};
    Vector2D vec_id{0,0};

    //Perform functions
    T_id *= T_id2;

    //Grab results
    Vector2D res_vec = T_id.translation();
    double res_ang = T_id.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Cancelling Rotations", "[Transform2D]")
{
    //Build objects
    Transform2D T_a{-PI/4};
    Transform2D T_b{PI/4};
    Vector2D vec_id{0,0};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Cancelling Transitions", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{1,2};
    Vector2D vec_b{-1,-2};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{0,0};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Only Transitions", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{2,2};
    Vector2D vec_b{1,3};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{3,5};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Basic Transformation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = PI/4;
    Vector2D vec_b{2,3};
    double ang_b = PI/3;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{0.292893,5.53553};
    double ang_ans = 1.8326;

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Big Rotation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = 4.88692;
    Vector2D vec_b{-2,3};
    double ang_b = 1.74533;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{3.60713, 4.49056};
    double ang_ans = 0.349066;

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = std::fmod(T_a.rotation(), 2.0*PI);

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}


/// TEST operator* ///

TEST_CASE("Transform2D::operator*, Identity Mul", "[Transform2D]")
{
    //Build objects
    Transform2D T_id{};
    Transform2D T_id2{};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_id3 = T_id * T_id2;

    //Grab results
    Vector2D res_vec = T_id3.translation();
    double res_ang = T_id3.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Cancelling Rotations", "[Transform2D]")
{
    //Build objects
    Transform2D T_a{-PI/4};
    Transform2D T_b{PI/4};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Cancelling Transitions", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{1,2};
    Vector2D vec_b{-1,-2};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Only Transitions", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{2,2};
    Vector2D vec_b{1,3};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{3,5};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Basic Transformation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = PI/4;
    Vector2D vec_b{2,3};
    double ang_b = PI/3;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{0.292893,5.53553};
    double ang_ans = 1.8326;

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Big Rotation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = 4.88692;
    Vector2D vec_b{-2,3};
    double ang_b = 1.74533;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{3.60713, 4.49056};
    double ang_ans = 0.349066;

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = std::fmod(T_c.rotation(), 2.0*PI);

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    REQUIRE(res_ang == Approx( ang_ans ).margin(epsilon));
}

/// TEST operator()(Vector2D) ///

TEST_CASE("Transform2D::operator(Vector), Idenity Transform", "[Transform2D]")
{
    //Build objects
    Transform2D T_ab{};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{1.0, 1.0};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);

    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Rotation", "[Transform2D]")
{
    //Build objects
    Transform2D T_ab{PI/4};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{0.0, 1.414213};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Translation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{3.0, -1.0};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Transformation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a, PI/4};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{2., -0.58578};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

/// TEST operator()(Twist2D) ///

TEST_CASE("Transform2D::operator(Twist2D), Idenity Transform", "[Transform2D]")
{
    //Build objects
    Transform2D T_ab{};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, 1.0, 2.0};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(epsilon));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(epsilon));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Rotation", "[Transform2D]")
{
    //Build objects
    Transform2D T_ab{PI/4};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, -0.70710678, 2.12132034};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(epsilon));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(epsilon));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Translation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, -0.04719755, 0.95280245};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(epsilon));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(epsilon));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Transformation", "[Transform2D]")
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a, PI/4};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{0.52359878, -1.75430433, 1.07412279};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(epsilon));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(epsilon));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(epsilon));
}