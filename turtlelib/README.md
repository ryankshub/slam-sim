# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
      1. Implement the Vector2D object as a class and create a normalize method.
      2. Implement a standalone function that takes in a Vector2D object as an input and return a unit vector.
      3. Make a derived class of Vector2D called UnitVector2D that would maintain the invariant of having a magnitude of one. The ~normalize~ functionality would be captured in the constructors of this class.   

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

      - The pros of method 3 is that the class encapsulation would allow functionality to maintain the invariant of a unit vector. Further, if the UnitVector2D is a subclass of Vector2D, then it would inherit plenty of the functionality. The con is that Vector2D would need to be turned into a class, which offers little benefit, but without the conversion every utility for Vector2D would be need to be rewritten for UnitVector2D.

      - Method 2 is excellent because it doesn't require any additional work for Vector2D; the function can simply produce a unit vector. The con is that the produced vector has no functionality to maintain magnitude invariant. 

      - Method 1 is the worst; it basically encapsulates Method 2 into a class with no invariant. 

   - Which of the methods would you implement and why?

      - I would implement method 2. It is the quickest method and requires no change to the Vector2D struct. There are also other ways to protect a Vector2D that must stay unit(e.g. const). 


2. What is the difference between a class and a struct in C++?
    - A struct is a collection of indepedent datum. A class is a collection of datum with method functions for manipulating, interfacing, and maintaining the datum and any invariants they must adhere to. 


3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?
    - `Guildline C.2: Use class if the class has an invariant.` Vector2D is a struct of two doubles that can represent a point or vector in 2-dimensional space. But there is no relationship between the two doubles; they are independent. Transform2D represents a homogeneous transformation matrix for 2-dimensional space; these matrices have to maintain a certain form in order for their properties to apply. In essence, Transform2D has invariants to maintain, Vector2D does not.

    - `Guideline C.8: Use class rather than struct if any member is non-public.` To ensure that Transform2D can maintain it's invariants, the class must protect it's members. Without that protection, any function could manipulate the class and break it's invariant. Vector2D's members do not need any protection. 

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
    - `Guideline C.46: by default, declare single-argument constructors explicit.` This is to avoid any unintended conversions. 

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not? Refer to [C++ Core Guidelines (Constants and Immutability)](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability) in your answer
    - `Guideline Con.2: by default, make member functions 'const'`. Any member function that does not need to manipulate member data should be declared `const` to enforce protections. The function `Transform2D::inv()` is declared `const` because calculating the inverse of a transformation matrix does not change the original matrix. Thus, the function should not manipulate any members. The function `*=` is updating the Transform2D instance by multiplying itself with another Transform2D; since the object's members are being updated, the function `*=` should not be declared `const`

# Sample Run of frame_main
```
Enter transform T_{a,b}:
90 0 1
Enter transform T_{b,c}:
90 1 0
T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b:
1 1
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b:
1 1 1
V_a [1 0 1]
V_b [1 1 1]
V_c [1 2 -1]
```