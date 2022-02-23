// Turtlelib Testfile
// This file test function in the turtlelib namespace
// RKS

#include "catch_ros/catch.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>

using namespace turtlelib;
static const double EPSILON = 1.0e-12;

////////////////// UTILITY FUNCTIONS /////

/// normalize_angle tests
/// TEST zero test normalize_angle 
/// \brief Test normalizing 0
TEST_CASE("normalize_angle zero test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double reg_ang = 0;
    // Normalized angle
    double ang_ans = 0;

    double test_ang = normalize_angle(reg_ang);
    REQUIRE(test_ang == Approx( ang_ans ).margin(EPSILON));
}

/// TEST limits test normalize_angle 
/// \brief Test normalizing pi and -pi
TEST_CASE("normalize_angle -pi/pi test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double pi_ang = PI;
    double neg_pi_ang = -PI;
    // Normalized angle
    double ang_ans = PI;

    double test1_ang = normalize_angle(pi_ang);
    double test2_ang = normalize_angle(neg_pi_ang);
    REQUIRE(test1_ang == Approx( ang_ans ).margin(EPSILON));
    REQUIRE(test2_ang == Approx( ang_ans ).margin(EPSILON));
}

/// \brief Test normalizing 2*pi and -2*pi
TEST_CASE("normalize_angle -2pi/2pi test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double pi_ang = 2.0*PI;
    double neg_pi_ang = -2.0*PI;
    // Normalized angle
    double ang_ans = 0;

    double test1_ang = normalize_angle(pi_ang);
    double test2_ang = normalize_angle(neg_pi_ang);
    REQUIRE(test1_ang == Approx( ang_ans ).margin(EPSILON));
    REQUIRE(test2_ang == Approx( ang_ans ).margin(EPSILON));
}

/// \brief Test normalizing 3*pi
TEST_CASE("normalize_angle 3pi test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double reg_ang = 3*PI;
    // Normalized angle
    double ang_ans = PI;

    double test_ang = normalize_angle(reg_ang);
    REQUIRE(test_ang == Approx( ang_ans ).margin(EPSILON));
}

/// TEST quadrants normalize_angle 
/// \brief Test positive quadraints conversion
TEST_CASE("normalize_angle positive quadrant test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double quad1_ang = PI/3.0;
    double quad2_ang = (3.0*PI)/4.0;
    double quad3_ang = (4.0*PI)/3.0;
    double quad4_ang = (7.0*PI)/4.0;
    // Normalized angle
    double quad1_ans = PI/3.0;
    double quad2_ans = (3.0*PI)/4.0;
    double quad3_ans = -(2.0*PI)/3.0;
    double quad4_ans = -PI/4.0;

    double quad1_test = normalize_angle(quad1_ang);
    double quad2_test = normalize_angle(quad2_ang);
    double quad3_test = normalize_angle(quad3_ang);
    double quad4_test = normalize_angle(quad4_ang);

    REQUIRE(quad1_test == Approx( quad1_ans ).margin(EPSILON));
    REQUIRE(quad2_test == Approx( quad2_ans ).margin(EPSILON));
    REQUIRE(quad3_test == Approx( quad3_ans ).margin(EPSILON));
    REQUIRE(quad4_test == Approx( quad4_ans ).margin(EPSILON));
}
 
/// \brief Test negative quadraints conversion
TEST_CASE("normalize_angle negative quadraints test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double quad1_ang = -PI/3.0;
    double quad2_ang = -(3.0*PI)/4.0;
    double quad3_ang = -(4.0*PI)/3.0;
    double quad4_ang = -(7.0*PI)/4.0;
    // Normalized angle
    double quad1_ans = -PI/3.0;
    double quad2_ans = -(3.0*PI)/4.0;
    double quad3_ans = (2.0*PI)/3.0;
    double quad4_ans = PI/4.0;

    double quad1_test = normalize_angle(quad1_ang);
    double quad2_test = normalize_angle(quad2_ang);
    double quad3_test = normalize_angle(quad3_ang);
    double quad4_test = normalize_angle(quad4_ang);

    REQUIRE(quad1_test == Approx( quad1_ans ).margin(EPSILON));
    REQUIRE(quad2_test == Approx( quad2_ans ).margin(EPSILON));
    REQUIRE(quad3_test == Approx( quad3_ans ).margin(EPSILON));
    REQUIRE(quad4_test == Approx( quad4_ans ).margin(EPSILON));
}

/// TEST basic cases
/// \brief Test 3PI/2
TEST_CASE("normalize_angle 3pi/2 test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double reg_ang = (3.0*PI)/2.0;
    // Normalized angle
    double ang_ans = -PI/2.0;

    double test_ang = normalize_angle(reg_ang);
    REQUIRE(test_ang == Approx( ang_ans ).margin(EPSILON));
}

/// \brief Test -5PI/2
TEST_CASE("normalize_angle -5pi/2 test", "[normalize_angle]")
{
    // Unnormalize regular angle
    double reg_ang = -(5.0*PI)/2.0;
    // Normalized angle
    double ang_ans = -PI/2.0;

    double test_ang = normalize_angle(reg_ang);
    REQUIRE(test_ang == Approx( ang_ans ).margin(EPSILON));
}


/// resolve_collision tests
/// TEST Clear no collision, no contact 
/// \brief Test no collision when robot is 
/// not near obstacle
TEST_CASE("resolve_collision clear no collision", "[resolve_collision]")
{
    DiffDrive D{0.3, 0.4, 0.0, 2.0, 3.0};
    double robot_radius = 2.0;
    Vector2D obs{10.0, 8.0};
    double obs_radius = 1.0;

    bool collided = resolve_collision(D, robot_radius, obs, obs_radius);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 3.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(!collided);
}

/// TEST Clear no collision, kiss contact
/// \brief Test no collision when robot is 
/// barely tapping obstacle
TEST_CASE("resolve_collision kiss no collision", "[resolve_collision]")
{
    DiffDrive D{0.3, 0.4, 0.0, 2.0, 3.0};
    double robot_radius = 2.0;
    Vector2D obs{5.0, 3.0};
    double obs_radius = 1.0;

    bool collided = resolve_collision(D, robot_radius, obs, obs_radius);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 3.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(!collided);
}

/// TEST Straight on Collision,
/// \brief Test head-on collision 
TEST_CASE("resolve_collision head-on collision", "[resolve_collision]")
{
    DiffDrive D{0.3, 0.4, 0.0, 3.0, 3.0};
    double robot_radius = 2.0;
    Vector2D obs{5.0, 3.0};
    double obs_radius = 1.0;

    bool collided = resolve_collision(D, robot_radius, obs, obs_radius);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 3.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(collided);
}

/// TEST Angled Collision,
/// \brief Test collision from a angle
TEST_CASE("resolve_collision collision from an angle", "[resolve_collision]")
{
    DiffDrive D{0.3, 0.4, PI/4.0, 3.0, 1.0};
    double robot_radius = 2.0;
    Vector2D obs{5.0, 3.0};
    double obs_radius = 1.0;

    bool collided = resolve_collision(D, robot_radius, obs, obs_radius);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();

    REQUIRE(l_ans.x == Approx( 2.87867 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 0.87867  ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI/4.0 ).margin(EPSILON));
    REQUIRE(collided);
}

/// TEST Skid Collision,
/// \brief Test collision from swiping obs
TEST_CASE("resolve_collision collision from swiping obs", "[resolve_collision]")
{
    DiffDrive D{0.3, 0.4, PI/4.0, 3.0, 3.0};
    double robot_radius = 2.0;
    Vector2D obs{5.0, 3.0};
    double obs_radius = 1.0;

    bool collided = resolve_collision(D, robot_radius, obs, obs_radius);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 3.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI/4.0 ).margin(EPSILON));
    REQUIRE(collided);
}



////////////////// VECTOR2D /////////////////////////////////////////////////////////

/// TEST Vector2D ADD ASSIGN +=
/// \brief Test adding empty vectors
TEST_CASE("Vector2D::operator+= summing zeros", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D empty{0.0, 0.0};

    // Answer vector
    Vector2D ans{0.0, 0.0};

    // Operation
    Vector2D result{0.0, 0.0};
    result += empty;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test adding zeros to non-zeros
TEST_CASE("Vector2D::operator+= add zero", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{0.0, 0.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{1.0, 2.0};

    // Operation
    Vector2D resA = A;
    resA += B;
    Vector2D resB = B;
    resB += A;

    REQUIRE(resA.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Basic adding case
TEST_CASE("Vector2D::operator+= basic add", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{-3.0, 4.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{-2.0, 6.0};

    // Operation
    Vector2D resA = A;
    resA += B;
    Vector2D resB = B;
    resB += A;

    REQUIRE(resA.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( ans.y ).margin(EPSILON));
}


/// TEST Vector2D SUBTRACT ASSIGN -=
/// \brief Test subtracting empty vectors
TEST_CASE("Vector2D::operator-= summing zeros", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D empty{0.0, 0.0};

    // Answer vector
    Vector2D ans{0.0, 0.0};

    // Operation
    Vector2D result{0.0, 0.0};
    result -= empty;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test subtract zeros from non-zeros
TEST_CASE("Vector2D::operator-= subtract zero", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{0.0, 0.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{1.0, 2.0};

    // Operation
    Vector2D resA = A;
    resA -= B;
    Vector2D resB = B;
    resB -= A;

    REQUIRE(resA.x == Approx( -ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( -ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Basic subtracting case
TEST_CASE("Vector2D::operator-= basic subtract", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{-3.0, 4.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{-4.0, 2.0};

    // Operation
    Vector2D resA = A;
    resA -= B;
    Vector2D resB = B;
    resB -= A;

    REQUIRE(resA.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( -ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( -ans.y ).margin(EPSILON));
}

/// TEST Vector2D SCALE ASSIGN *=
/// \brief Test scaling vectors by zero
TEST_CASE("Vector2D::operator*= scale by zero", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = 0;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{0.0, 0.0};

    // Operation
    Vector2D result = vec;
    result *= scalar;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test scaling vectors by one
TEST_CASE("Vector2D::operator*= scale by one", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = 1;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans = vec;

    // Operation
    Vector2D result = vec;
    result *= scalar;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test flipping vectors
TEST_CASE("Vector2D::operator*= flip vector", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = -1;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{-1.0, 2.0};

    // Operation
    Vector2D result = vec;
    result *= scalar;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test positive scaling 
TEST_CASE("Vector2D::operator*= positive scaling", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = 3;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{3.0, -6.0};

    // Operation
    Vector2D result = vec;
    result *= scalar;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test negative scaling
TEST_CASE("Vector2D::operator*= negative scaling", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = -4.3;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{-4.3, 8.6};

    // Operation
    Vector2D result = vec;
    result *= scalar;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

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


/// TEST Vector2D ADD +
/// \brief Test adding empty vectors
TEST_CASE("Vector2D::operator+ summing zeros", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D empty{0.0, 0.0};

    // Answer vector
    Vector2D ans{0.0, 0.0};

    // Operation
    Vector2D result = empty + empty;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test adding zeros to non-zeros
TEST_CASE("Vector2D::operator+ add zero", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{0.0, 0.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{1.0, 2.0};

    // Operation
    Vector2D resA = A + B;
    Vector2D resB = B + A;

    REQUIRE(resA.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Basic adding case
TEST_CASE("Vector2D::operator+ basic add", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{-3.0, 4.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{-2.0, 6.0};

    // Operation
    Vector2D resA = A + B;
    Vector2D resB = B + A;

    REQUIRE(resA.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( ans.y ).margin(EPSILON));
}


/// TEST Vector2D SUBTRACT -
/// \brief Test subtracting empty vectors
TEST_CASE("Vector2D::operator- summing zeros", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D empty{0.0, 0.0};

    // Answer vector
    Vector2D ans{0.0, 0.0};

    // Operation
    Vector2D result = empty - empty;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test subtract zeros from non-zeros
TEST_CASE("Vector2D::operator- subtract zero", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{0.0, 0.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{1.0, 2.0};

    // Operation
    Vector2D resA = A - B;
    Vector2D resB = B - A;

    REQUIRE(resA.x == Approx( -ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( -ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Basic subtracting case
TEST_CASE("Vector2D::operator- basic subtract", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{-3.0, 4.0};
    Vector2D B{1.0, 2.0};

    // Answer vector
    Vector2D ans{-4.0, 2.0};

    // Operation
    Vector2D resA = A - B;
    Vector2D resB = B - A;

    REQUIRE(resA.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(resA.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(resB.x == Approx( -ans.x ).margin(EPSILON));
    REQUIRE(resB.y == Approx( -ans.y ).margin(EPSILON));
}


/// TEST Vector2D SCALE  *
/// \brief Test scaling vectors by zero
TEST_CASE("Vector2D::operator* scale by zero", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = 0;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{0.0, 0.0};

    // Operation
    Vector2D result = vec * scalar;
    Vector2D result2 = scalar * vec;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(result2.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result2.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test scaling vectors by one
TEST_CASE("Vector2D::operator* scale by one", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = 1;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans = vec;

    // Operation
    Vector2D result = vec * scalar;
    Vector2D result2 = scalar * vec;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(result2.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result2.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test flipping vectors
TEST_CASE("Vector2D::operator* flip vector", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = -1;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{-1.0, 2.0};

    // Operation
    Vector2D result = vec * scalar;
    Vector2D result2 = scalar * vec;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(result2.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result2.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test positive scaling 
TEST_CASE("Vector2D::operator* positive scaling", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = 3;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{3.0, -6.0};

    // Operation
    Vector2D result = vec * scalar;
    Vector2D result2 = scalar * vec;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(result2.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result2.y == Approx( ans.y ).margin(EPSILON));
}

/// \brief Test negative scaling
TEST_CASE("Vector2D::operator* negative scaling", "[Vector2D]")
{
    // Initialize test vectors
    double scalar = -4.3;
    Vector2D vec{1.0, -2.0};

    // Answer vector
    Vector2D ans{-4.3, 8.6};

    // Operation
    Vector2D result = vec * scalar;
    Vector2D result2 = scalar * vec;

    REQUIRE(result.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result.y == Approx( ans.y ).margin(EPSILON));
    REQUIRE(result2.x == Approx( ans.x ).margin(EPSILON));
    REQUIRE(result2.y == Approx( ans.y ).margin(EPSILON));
}


/// TEST Vector2D DOT PRODUCT
/// \brief Test dot product of parallel vectors
TEST_CASE("Vector2D dot, parallel vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{3.0, 3.0};

    // Dot product result
    double ans = 12.0;

    // Operation
    double result = dot(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test dot product of basic vectors
TEST_CASE("Vector2D dot, basic vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{0.0, 2.0};

    // Dot product result
    double ans = 4.0;

    // Operation
    double result = dot(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test dot product of perpendicular vectors
TEST_CASE("Vector2D dot, perpendicular vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{-2.0, 2.0};

    // Dot product result
    double ans = 0.0;

    // Operation
    double result = dot(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test dot product of opposite vectors
TEST_CASE("Vector2D dot, opposite vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{-2.0, -2.0};

    // Dot product result
    double ans = -8.0;

    // Operation
    double result = dot(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}


/// TEST Vector2D CALCULATE MAGNITUDE
/// \brief Test calculate magnitude of zero vector
TEST_CASE("Vector2D magnitude, zero vector", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{0.0, 0.0};

    // Dot product result
    double ans = 0.0;

    // Operation
    double result = magnitude(A);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test calculate magnitude of vectors
TEST_CASE("Vector2D magnitude, basic vector", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{3.0, 4.0};
    Vector2D B{3.0, -4.0};
    Vector2D C{-3.0, 4.0};
    Vector2D D{-3.0, -4.0};

    // Dot product result
    double ans = 5.0;

    // Operation
    double resultA = magnitude(A);
    double resultB = magnitude(B);
    double resultC = magnitude(C);
    double resultD = magnitude(D);

    REQUIRE(resultA == Approx( ans ).margin(EPSILON));
    REQUIRE(resultB == Approx( ans ).margin(EPSILON));
    REQUIRE(resultC == Approx( ans ).margin(EPSILON));
    REQUIRE(resultD == Approx( ans ).margin(EPSILON));
}

/// TEST Vector2D CALCULATE ANGLE
/// \brief Test calculate angle of parallel vectors
TEST_CASE("Vector2D angle, parallel vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{3.0, 3.0};

    // Dot product result
    double ans = 0.0;

    // Operation
    double result = angle(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test calculate angle of basic vectors
TEST_CASE("Vector2D angle, basic vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{0.0, 2.0};

    // Dot product result
    double ans = PI/4.0;

    // Operation
    double result = angle(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test calculate angle of perpendicular vectors
TEST_CASE("Vector2D angle, perpendicular vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{-2.0, 2.0};

    // Dot product result
    double ans = PI/2.0;

    // Operation
    double result = angle(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test calculate angle of opposite vectors
TEST_CASE("Vector2D angle, opposite vectors", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{-2.0, -2.0};

    // Dot product result
    double ans = PI;

    // Operation
    double result = angle(A, B);

    REQUIRE(result == Approx( ans ).margin(EPSILON));
}

/// \brief Test calculate angle exception with zero mag left vector
TEST_CASE("Vector2D angle, zero mag left vector", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{0.0, 0.0};
    Vector2D B{-2.0, -2.0};

    // Operation
    REQUIRE_THROWS(angle(A,B));
}

/// \brief Test calculate angle exception with zero mag right vector
TEST_CASE("Vector2D angle, zero mag right vector", "[Vector2D]")
{
    // Initialize test vectors
    Vector2D A{2.0, 2.0};
    Vector2D B{0.0, 0.0};

    // Operation
    REQUIRE_THROWS(angle(A,B));
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


/// TEST Transform2D Integrate
/// \brief Test Zero Twist
TEST_CASE("integration zero Twist2D", "[Transform2D]")
{
    //Test twist
    Twist2D twist{0.0, 0.0, 0.0};

    //Answers components
    Vector2D ans_trans{0.0, 0.0};
    double ans_angle = 0.0;

    //Results
    Transform2D res = integrate_twist(twist);
    Vector2D res_trans = res.translation();
    double res_angle = res.rotation();

    REQUIRE(res_trans.x == Approx(ans_trans.x).margin(EPSILON));
    REQUIRE(res_trans.y == Approx(ans_trans.y).margin(EPSILON));
    REQUIRE(res_angle == Approx(ans_angle).margin(EPSILON));
}

/// \brief Test Translation of 2D Twist
TEST_CASE("integrate translation Twist2D", "[Transform2D]")
{
    //Test twist
    Twist2D twist{0.0, 5.0, 2.0};

    //Answers components
    Vector2D ans_trans{5.0, 2.0};
    double ans_angle = 0.0;

    //Results
    Transform2D res = integrate_twist(twist);
    Vector2D res_trans = res.translation();
    double res_angle = res.rotation();

    REQUIRE(res_trans.x == Approx(ans_trans.x).margin(EPSILON));
    REQUIRE(res_trans.y == Approx(ans_trans.y).margin(EPSILON));
    REQUIRE(res_angle == Approx(ans_angle).margin(EPSILON));
}

/// \brief Test Rotation of 2D Twist
TEST_CASE("integrate rotation Twist2D", "[Transform2D]")
{
    //Test twist
    Twist2D twist{PI/4.0, 0.0, 0.0};

    //Answers components
    Vector2D ans_trans{0.0, 0.0};
    double ans_angle = PI/4.0;

    //Results
    Transform2D res = integrate_twist(twist);
    Vector2D res_trans = res.translation();
    double res_angle = res.rotation();

    REQUIRE(res_trans.x == Approx(ans_trans.x).margin(EPSILON));
    REQUIRE(res_trans.y == Approx(ans_trans.y).margin(EPSILON));
    REQUIRE(res_angle == Approx(ans_angle).margin(EPSILON));
}

/// \brief Test Full Motion of 2D Twist
TEST_CASE("integrate full motion Twist2D", "[Transform2D]")
{
    //Test twist
    Twist2D twist{PI/4.0, 5.0, 2.0};

    //Answers components
    Vector2D ans_trans{3.75574, 3.66525};
    double ans_angle = PI/4.0;

    //Results
    Transform2D res = integrate_twist(twist);
    Vector2D res_trans = res.translation();
    double res_angle = res.rotation();

    REQUIRE(res_trans.x == Approx(ans_trans.x).margin(EPSILON));
    REQUIRE(res_trans.y == Approx(ans_trans.y).margin(EPSILON));
    REQUIRE(res_angle == Approx(ans_angle).margin(EPSILON));
}


////////////////// DIFFDRIVE ////////////

/// TEST DiffDrive::DiffDrive()
/// \brief Test Empty Constructor
TEST_CASE("DiffDrive Empty Constructor", "[DiffDrive]") 
{
    DiffDrive D{};

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double track_ans = D.wheel_track();
    double radius_ans = D.wheel_radius();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(track_ans == Approx( 0.1 ).margin(EPSILON));
    REQUIRE(radius_ans == Approx( 0.1 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( 0.0 ).margin(EPSILON));
}


/// TEST DiffDrive::DiffDrive(double, double)
/// \brief Test Wheel Configuration Constructor
TEST_CASE("DiffDrive Wheel Configuration Constructor", "[DiffDrive]") 
{
    DiffDrive D{.3, 3.5};

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double track_ans = D.wheel_track();
    double radius_ans = D.wheel_radius();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(track_ans == Approx( 0.3 ).margin(EPSILON));
    REQUIRE(radius_ans == Approx( 3.5 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( 0.0 ).margin(EPSILON));
}


/// TEST DiffDrive::DiffDrive(double, double, double, double, double)
/// \brief Test Full Constructor w/ normalization
TEST_CASE("DiffDrive Full Constructor w/ normalization", "[DiffDrive]") 
{
    DiffDrive D{0.3, 0.4, (7*PI)/3.0, 3.5, -0.8};

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double track_ans = D.wheel_track();
    double radius_ans = D.wheel_radius();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 3.5 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( -0.8 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI/3.0 ).margin(EPSILON));
    REQUIRE(track_ans == Approx( 0.3 ).margin(EPSILON));
    REQUIRE(radius_ans == Approx( 0.4 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test Full Constructor
TEST_CASE("DiffDrive Full Constructor", "[DiffDrive]") 
{
    DiffDrive D{0.3, 0.4, PI/3.0, 3.5, -0.8};

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double track_ans = D.wheel_track();
    double radius_ans = D.wheel_radius();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 3.5 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( -0.8 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI/3.0 ).margin(EPSILON));
    REQUIRE(track_ans == Approx( 0.3 ).margin(EPSILON));
    REQUIRE(radius_ans == Approx( 0.4 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test Full Constructor w/ invalid wheel track
TEST_CASE("DiffDrive Constructor w/invalid wheel track", "[DiffDrive]") 
{
    bool test_passed = false;
    try
    {
        DiffDrive D{0.0, 0.4, PI/3.0, 3.5, -0.8};
    } catch(const std::exception& e) {
        test_passed = true;
    }
    REQUIRE(test_passed);
}

/// \brief Test Full Constructor w/ invalid wheel radius
TEST_CASE("DiffDrive Constructor w/ invalid wheel radius", "[DiffDrive]") 
{
    bool test_passed = false;
    try
    {
        DiffDrive D{0.3, -0.4, PI/3.0, 3.5, -0.8};
    } catch(const std::exception& e) {
        test_passed = true;
    }
    REQUIRE(test_passed);
}

/// TEST DiffDrive::set_configuration
/// \brief Test changing setting robot's configuration at origin
TEST_CASE("DiffDrive set zero configuration", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{0.3, 0.4, PI/3.0, 3.5, -0.8};
    //Move robot to origin
    D.set_configuration(0.0, 0.0, 0.0);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();


    REQUIRE(l_ans.x == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 0.0 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 0.0 ).margin(EPSILON));
}

/// \brief Test changing setting robot's configuration to non-point
TEST_CASE("DiffDrive set non-zero configuration", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{0.3, 0.4, PI/3.0, 3.5, -0.8};
    //Move robot to origin
    D.set_configuration(PI/5.0, 3.0, -4.0);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();


    REQUIRE(l_ans.x == Approx( 3.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( -4.0).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI/5.0 ).margin(EPSILON));
}

/// \brief Test angle is normalized when changing setting robot's configuration
TEST_CASE("DiffDrive normalized angle when setting configuration", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{0.3, 0.4, PI/3.0, 3.5, -0.8};
    //Move robot to origin
    D.set_configuration(-PI, 3.0, -4.0);

    Vector2D l_ans = D.location();
    double ang_ans = D.theta();


    REQUIRE(l_ans.x == Approx( 3.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( -4.0).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI ).margin(EPSILON));
}

/// TEST DiffDrive::set_wheel_config
/// \brief Test changing setting robot's wheel config 
TEST_CASE("DiffDrive set wheel configuration", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{0.3, 0.4, PI/3.0, 3.5, -0.8};
    //Move robot to origin
    D.set_wheel_config(0.7, 1.4);

    double track_ans = D.wheel_track();
    double radius_ans = D.wheel_radius();


    REQUIRE(track_ans == Approx( 0.7 ).margin(EPSILON));
    REQUIRE(radius_ans == Approx( 1.4 ).margin(EPSILON));
    //Check invalid cases
    REQUIRE_THROWS(D.set_wheel_config(0.0, 1.3));
    REQUIRE_THROWS(D.set_wheel_config(0.5, -1.3));

}

/// TEST DiffDrive::apply_fw_kin
/// \brief Test Forward Kinematics on Forward path
TEST_CASE("DiffDrive FW KIN w/ Forward motion", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    //Fw Kin
    D.apply_fw_kin((5.0*PI)/2.0, (5.0*PI)/2.0);
    //Get answers
    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 7.55360 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 6.55360 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI/4.0).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( PI/2.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( PI/2.0 ).margin(EPSILON));
}

/// \brief Test Forward Kinematics with rotate in place
TEST_CASE("DiffDrive FW KIN w/ rotation", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    //Fw Kin
    D.apply_fw_kin(-PI/2.0, PI/2.0);
    //Get answers
    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 1.0).margin(EPSILON));
    REQUIRE(ang_ans == Approx( (3.0*PI)/4.0 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( -PI/2.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( PI/2.0 ).margin(EPSILON));
}

/// \brief Test Forward Kinematics with circle arc motion
TEST_CASE("DiffDrive FW KIN w/ circle arc", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};

    //Fw Kin
    D.apply_fw_kin(0, PI);
    //Get answers
    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 2.41421).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 3.0*PI/4.0 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( 0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( PI ).margin(EPSILON));
}


/// TEST DiffDrive::cal_inv_kin
/// \brief Test Inverse Kinematics on Forward path
TEST_CASE("DiffDrive INV KIN w/ Forward motion", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{0.0, 7.85398, 0.0};
    //Inv Kin
    std::vector<double> ans_vels = D.cal_inv_kin(B);

    REQUIRE(ans_vels.at(0) == Approx( (5.0*PI)/2.0 ).margin(EPSILON));
    REQUIRE(ans_vels.at(1) == Approx( (5.0*PI)/2.0 ).margin(EPSILON));
}

/// \brief Test Inverse Kinematics with rotate in place
TEST_CASE("DiffDrive INV KIN w/ rotation", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{PI/2.0, 0.0, 0.0};
    //Inv Kin
    std::vector<double> ans_vels = D.cal_inv_kin(B);

    REQUIRE(ans_vels.at(0) == Approx( -PI/2.0 ).margin(EPSILON));
    REQUIRE(ans_vels.at(1) == Approx( PI/2.0 ).margin(EPSILON));
}

/// \brief Test Inverse Kinematics with circle arc motion
TEST_CASE("DiffDrive INV KIN w/ circle arc", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{PI/2.0, PI/2.0, 0.0};
    //Inv Kin
    std::vector<double> ans_vels = D.cal_inv_kin(B);

    REQUIRE(ans_vels.at(0) == Approx( 0 ).margin(EPSILON));
    REQUIRE(ans_vels.at(1) == Approx( PI ).margin(EPSILON));
}

/// \brief Test Inverse Kinematics with invalid twist
TEST_CASE("DiffDrive INV KIN w/ invalid twist", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{PI/2.0, 3.0, 3.0};
    //Inv Kin
    REQUIRE_THROWS(D.cal_inv_kin(B));
}


/// TEST DiffDrive::apply_twist
/// \brief Test appling twist for Forward path
TEST_CASE("DiffDrive apply twist w/ Forward motion", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{0.0, 7.85398, 0.0};
    //Apply Twist
    D.apply_twist(B);
    //Get answers
    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 7.55360 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 6.55360 ).margin(EPSILON));
    REQUIRE(ang_ans == Approx( PI/4.0).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( PI/2.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( PI/2.0 ).margin(EPSILON));
}

/// \brief Test apply twist with rotate in place
TEST_CASE("DiffDrive apply twist w/ rotation", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{PI/2.0, 0.0, 0.0};
    //Apply Twist
    D.apply_twist(B);
    //Get answers
    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 1.0).margin(EPSILON));
    REQUIRE(ang_ans == Approx( (3.0*PI)/4.0 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( -PI/2.0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( PI/2.0 ).margin(EPSILON));
}

/// \brief Test apply twist with circle arc motion
TEST_CASE("DiffDrive apply twist w/ circle arc", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{PI/2.0, PI/2.0, 0.0};
    //Apply Twist
    D.apply_twist(B);
    //Get answers
    Vector2D l_ans = D.location();
    double ang_ans = D.theta();
    double left_wheel_ans = D.left_wheel_pos();
    double right_wheel_ans = D.right_wheel_pos();

    REQUIRE(l_ans.x == Approx( 2.0 ).margin(EPSILON));
    REQUIRE(l_ans.y == Approx( 2.41421).margin(EPSILON));
    REQUIRE(ang_ans == Approx( 3.0*PI/4.0 ).margin(EPSILON));
    REQUIRE(left_wheel_ans == Approx( 0 ).margin(EPSILON));
    REQUIRE(right_wheel_ans == Approx( PI ).margin(EPSILON));
}

/// \brief Test apply twist with invalid twist
TEST_CASE("DiffDrive apply twist w/ invalid twist", "[DiffDrive]") 
{
    //Init object
    DiffDrive D{2.0, 1.0, PI/4.0, 2.0, 1.0};
    Twist2D B{PI/2.0, 3.0, 3.0};
    //Inv Kin
    REQUIRE_THROWS(D.apply_twist(B));
}

