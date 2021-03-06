#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        double diff = d1 - d2;
        diff = std::abs(diff);
        if (diff <= epsilon)
        {
            return true;
        }
        return false;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        double rad = (deg * PI)/ (double)180.0;
        return rad;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        double deg = (rad * (double)180.0)/ PI;
        return deg;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(3.65, 3.650), "comparing doubles works");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "converting zero failed");

    static_assert(almost_equal(deg2rad(180.0), PI), "converting to pi failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "converting zero failed");

    static_assert(almost_equal(rad2deg(PI), 180.0), "converting from pi failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "double conversion failed");

    static_assert(almost_equal(rad2deg(deg2rad(2.1)), 2.1), "double conversion failed");

    /// Utility functions
    /// \brief Normalize the an input angle between (-pi,pi]
    /// \param rad - angle in radians
    /// \returns angle in range (-pi, pi]
    double normalize_angle(double rad);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief add the values of input vector into this vector
        /// \param rhs - Vector who's values to add
        /// \return - a reference to sum vector
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract the values of input vector from this vector
        /// \param rhs - Vector who's values to subtract
        /// \return - a reference to difference vector
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief Multiply vector by scalar
        /// \param rhs - Scalar multiple
        /// \return - a reference to the scaled vector
        Vector2D & operator*=(const double & rhs);
    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief produce normalize version of a vector
    /// \param v - vector to normalize
    /// \return normalized version of the vector
    Vector2D normalize(const Vector2D & v);

    /// \brief addes two vectors together
    /// \param lhs - left vector to add
    /// \param rhs - right vector to add
    /// \return sum of the two input vector
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief subtract two vectors together
    /// \param lhs - left vector in subtraction
    /// \param rhs - right vector in subtrcation
    /// \return difference of the two input vector
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply vector by scalar
    /// \param lhs - scalar value
    /// \param rhs - vector value
    /// \return scaled vector
    Vector2D operator*(const double lhs, Vector2D rhs);

    /// \brief multiply vector by scalar
    /// \param lhs - vector value
    /// \param rhs - scalar value
    /// \return scaled vector
    Vector2D operator*(Vector2D lhs, const double rhs);

    /// \brief compute the dot product of two vectors
    /// \param lvec - left 2D vector
    /// \param rvec - right 2D vector
    /// \return dot product of the vector
    double dot(const Vector2D & lvec, const Vector2D & rvec);

    /// \brief compute the cross product of two vectors
    /// \param lvec - left 2D vector
    /// \param rvec - right 2D vector
    /// \return cross product of the vector
    double cross(const Vector2D & lvec, const Vector2D & rvec);

    /// \brief compute a vector's magnitude
    /// \param vec - 2D vector
    /// \return magnitude of the vector
    double magnitude(const Vector2D & vec);

    /// \brief compute the angle between two vectors, in range [0, pi]
    /// \param lvec - left 2D vector
    /// \param rvec - right 2D vector
    /// \return angle of between the vectors, in range [0, pi]
    /// \throw Invalid argument if either vector has a magnitude of zero
    double angle(const Vector2D & lvec, const Vector2D & rvec);

    /// \brief a twist (velocity representation)
    struct Twist2D
    {
        /// \brief Rotational velocity 
        double theta_dot;

        /// \brief Velocity in the x-axis
        double x_dot;

        /// \brief Velocity in the y-axis
        double y_dot;
    };

    /// \brief Print a 2D twist
    /// \param os - stream to output ot
    /// \param t - twist to print out
    std::ostream & operator<<(std::ostream & os, Twist2D & t);

    /// \brief input a 2 dimensional twist 
    ///        Format should be [theta x y] or theta x y
    /// \param is - stream from which to read
    /// \param t [out] - output twist
    std::istream & operator>>(std::istream & is, Twist2D & t);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply the adjunct to change the twist's frame
        /// \param t - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(const Twist2D & t) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    private:
        Vector2D mVec;
        double mAng_rad;
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief integrate a 2D twist for one time-unit
    /// \param twist - body frame twist to integrate
    /// \return 2D transformation matrix representing the motion
    Transform2D integrate_twist(const Twist2D & twist);
}

#endif
