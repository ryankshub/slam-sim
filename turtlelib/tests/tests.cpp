// Turtlelib Testfile
// This file test function in the turtlelib namespace
// RKS

#include "catch_ros/catch.hpp"
#include "turtlelib/rigid2d.hpp"
#include <cmath>
#include <sstream>

using namespace turtlelib;
static const double EPSILON = 1.0e-12;

////////////////// VECTOR2D ////////////

/// TEST Vector2D Read
/// \brief Test Reading into Vector2D
TEST_CASE("istream Vector input","[Vector2D]") // James Avtges
{
    std::stringstream bracket;
    turtlelib::Vector2D bracketV;
    bracket.str("[1 1]");
    std::stringstream number;
    turtlelib::Vector2D numberV;
    number.str("1 1");
    bracket >> bracketV;
    number >> numberV;
    REQUIRE(bracketV.x == Approx(1.0).margin(EPSILON));
    REQUIRE(bracketV.y == Approx(1.0).margin(EPSILON));
    REQUIRE(numberV.x == Approx(1.0).margin(EPSILON));
    REQUIRE(numberV.y == Approx(1.0).margin(EPSILON));
}


/// TEST Vector2D Print
/// \brief Test Printing from Vector2D
TEST_CASE("ostream Vector output","[Vector2D]") // James Avtges
{
    std::stringstream vectorOut;
    turtlelib::Vector2D vector;
    vector.x = 9.0;
    vector.y = 1.0;

    vectorOut << vector;

    REQUIRE(vectorOut.str() == "[9 1]");
}

/// TEST Vector2D normalize
/// \brief Test Normalizing a Vector2D
TEST_CASE("normalize Vector2D","[Vector2D]") 
{
    Vector2D vec{5,5};
    Vector2D norm = normalize(vec);
    double x_ans = 0.707106;
    double y_ans = 0.707106;


    REQUIRE(norm.x == Approx(x_ans).margin(EPSILON));
    REQUIRE(norm.y == Approx(y_ans).margin(EPSILON));
}

////////////////// TWIST2D ////////////

/// TEST Twist2D ReadPrinting from Vector2D
/// \brief Test Reading into Twist2D
TEST_CASE("istream Twist input","[Twist2D]") // James Avtges
{
    std::stringstream bracket;
    turtlelib::Twist2D bracketT;
    bracket.str("[1 2 3]");
    std::stringstream number;
    turtlelib::Twist2D numberT;
    number.str("1 2 3");
    bracket >> bracketT;
    number >> numberT;
    REQUIRE(bracketT.theta_dot == Approx(1.0).margin(EPSILON));
    REQUIRE(bracketT.x_dot == Approx(2.0).margin(EPSILON));
    REQUIRE(bracketT.y_dot == Approx(3.0).margin(EPSILON));
    REQUIRE(numberT.theta_dot == Approx(1.0).margin(EPSILON));
    REQUIRE(numberT.x_dot == Approx(2.0).margin(EPSILON));
    REQUIRE(numberT.y_dot == Approx(3.0).margin(EPSILON));
}


/// TEST Twist2D Print
/// \brief Test Printing from Twist2D
TEST_CASE("ostream Twist output","[Twist2D]")// James Avtges
{
    std::stringstream twistOut;
    turtlelib::Twist2D twist;
    twist.theta_dot = 10.0;
    twist.x_dot = 5.0;
    twist.y_dot = 4.0;

    twistOut << twist;

    REQUIRE(twistOut.str() == "[10 5 4]");
}


////////////////// TRANSFORM2D ////////////

/// TEST Transform2D::Transform2D()
/// \brief Test Empty Constructor
TEST_CASE("Transform2D::Transform2D(), Empty Constructor", "[Transform2D]") 
{
    Transform2D T{};

    Vector2D t_ans = T.translation();
    double ang_ans = T.rotation();

    REQUIRE(t_ans.x == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(t_ans.y == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 0.0 ).margin(EPSILON));
} 


/// TEST Transform2D::Transform2D(Vector2D)
/// \brief Test Constructor with Vector2D input
TEST_CASE("constructor_trans", "[Transform2D]") // Anna Garverick
{
    Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    Transform2D T(v);

    Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = rad2deg(r_out);

    REQUIRE(t_out.x == Approx( 1.0 ).margin(EPSILON));
    REQUIRE(t_out.y == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(d == Approx( 0.0 ).margin(EPSILON));
}


/// TEST Transform2D::Transform2D(double)
/// \brief Test Constructor with angle input
TEST_CASE("constructor_rot", "[Transform2D]") // Anna Garverick
{
    double r = deg2rad(90.0);

    Transform2D T(r);

    Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = rad2deg(r_out);

    REQUIRE(t_out.x == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(t_out.y == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(d == Approx( 90.0 ).margin(EPSILON));
}


/// TEST Transform2D::Transform2D(Vector2D, double)
/// \brief Test Constructor with Vector2D and angle input
TEST_CASE("constructor_all", "[Transform2D]") // Anna Garverick
{
    Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    double r = deg2rad(90.0);

    Transform2D T(v, r);

    Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = rad2deg(r_out);

    REQUIRE(t_out.x == Approx( 1.0 ).margin(EPSILON));
    REQUIRE(t_out.y == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(d == Approx( 90.0 ).margin(EPSILON));
} 


/// TEST Transform2D::inv
/// \brief Test inverse function
TEST_CASE("inv", "[Transform2D]") // Anna Garverick
{
    Vector2D v;
    v.x = 1.0;
    v.y = 2.0;

    double r = PI/4;

    Transform2D T(v,r);

    Transform2D T_inv(0);
    T_inv = T.inv();

    Vector2D t_out = T_inv.translation();
    double r_out = T_inv.rotation();

    REQUIRE(t_out.x == Approx(-2.12132).margin(EPSILON));
    REQUIRE(t_out.y == Approx(-0.7071).margin(EPSILON));
    REQUIRE(r_out == Approx(-PI/4).margin(EPSILON));
}


/// TEST Transform2D::translation
/// \brief Test getter translation function
TEST_CASE("trans", "[Transform2D]") //Anna Garverick
{
    Vector2D v;
    v.x = 5.0;
    v.y = 10.0;

    Transform2D T(v);

    Vector2D t_out = T.translation();

    REQUIRE(t_out.x == Approx(5.0).margin(EPSILON));
    REQUIRE(t_out.y == Approx(10.0).margin(EPSILON));
}


/// TEST Transform2D::rotation
/// \brief Test setter rotation function
TEST_CASE("rot", "[Transform2D")  //Anna Garverick
{    
    double r = deg2rad(33.0);

    Transform2D T(r);

    double r_out = T.rotation();

    double d = rad2deg(r_out);

    REQUIRE(d == Approx(33.0).margin(EPSILON));
}

/// TEST operator*= ///
/// \brief Test self referencing multiplcation
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test identity multiplaction
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test two transformation whose rotations should cancel
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test two transformation whose transitions should cancel
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test two transformation with only transitions
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test basic multiplication
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( ang_ans ).margin(EPSILON));
}

/// \brief Test two transformation whose combined rotations are greater than 2PI
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( ang_ans ).margin(EPSILON));
}


/// TEST operator* ///
/// \brief Test identity multiplaction
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test two transformation whose rotations should cancel
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test two transformation whose transitions should cancel
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test two transformation with only transitions
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
    REQUIRE(res_vec.x == Approx( vec_id.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_id.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test basic multiplication
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( ang_ans ).margin(EPSILON));
}

/// \brief Test two transformation whose combined rotations are greater than 2PI
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
    REQUIRE(res_ang == Approx( ang_ans ).margin(EPSILON));
}


/// TEST operator()(Vector2D) ///
/// \brief Applying Identity transform to a point
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
}

/// \brief Applying only rotation to a point
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
}

/// \brief Applying only translation to a point
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
}

/// \brief Applying basic transformation to a point
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
    REQUIRE(res_vec.x == Approx( vec_ans.x ).margin(EPSILON));
    REQUIRE(res_vec.y == Approx( vec_ans.y ).margin(EPSILON));
}


/// TEST operator()(Twist2D) ///
/// \brief Applying Identity transform to a twist
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
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(EPSILON));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(EPSILON));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(EPSILON));
}

/// \brief Applying only rotation to a twist
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
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(EPSILON));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(EPSILON));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(EPSILON));
}

/// \brief Applying only translation to a twist
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
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(EPSILON));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(EPSILON));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(EPSILON));
}

/// \brief Applying basic transformation to a twist
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
    REQUIRE(res_twt.theta_dot == Approx ( twt_ans.theta_dot).margin(EPSILON));
    REQUIRE(res_twt.x_dot == Approx( twt_ans.x_dot ).margin(EPSILON));
    REQUIRE(res_twt.y_dot == Approx( twt_ans.y_dot ).margin(EPSILON));
}


/// TEST Transform2D Read
/// \brief Test Reading into Transform2D
TEST_CASE("istream Transform input","[Transform2D]") // James Avtges
{
    std::stringstream transform;
    turtlelib::Transform2D T(0);
    transform.str("deg: 80 x: 2 y: 4\n");
    transform >> T;

    turtlelib::Vector2D translation = T.translation();
    double rotation = turtlelib::rad2deg(T.rotation());
    REQUIRE(translation.x == Approx(2.0).margin(EPSILON));
    REQUIRE(translation.y == Approx(4.0).margin(EPSILON));
    REQUIRE(rotation == Approx(80.0).margin(EPSILON));
}


/// TEST Transform2D Print
/// \brief Test Printing from Transform2D
TEST_CASE("ostream Transform output","[Transform2D]")// James Avtges
{
    std::stringstream transformOut;
    turtlelib::Vector2D trans;
    trans.x = 3.2;
    trans.y = 4.0;
    double rot = PI/3;
    turtlelib::Transform2D T(trans,rot);

    transformOut << T;

    REQUIRE(transformOut.str() == "deg: 60 x: 3.2 y: 4");
}