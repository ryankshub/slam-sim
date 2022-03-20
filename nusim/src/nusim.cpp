/// \file nusim
/// \brief The Nusim node maintains the state of the simulation
///
/// PARAMETERS:
///   private (parameter scoped to node name):
///     rate (int): The running rate of the node (Hz)
///     x0 (double): Initial x position of the robot (m)
///     y0 (double): Initial y position of the robot (m)
///     theta0 (double): Initial theta of the robot (rad)
///     obstacles/obs_x: List of x position for cylinder obstacles (m)
///     obstacles/obs_y: List of y position for cylinder obstacles (m)
///     obstacles/radius: Radius of the cylinder obstacles (m)
///     dist/vel_mean: Mean of Wheel Command noise. Simulates changes in velocity 
///         wheel vel despite commanded speed 
///     dist/vel_variance: Standard deviation of Wheel Command noise.
///     dist/slip_min: Minimum value of slip to apply to wheel position
///     dist/slip_max: Maximum value of slip to apply to wheel position
///     sensor/basic_sensor_variance: Standard deviation of sensor noise
///         for fake landmark sensor
///     sensor/max_range: Max detection range of fake landmark sensor. If 
///         a landmark is outside of this range, it will not be detected
///     laser/min_angle: Starting angle of lidar sensor
///     laser/max_angle: Ending angle of lidar sensor
///     laser/min_range: Minimum distance for lidar scan
///     laser/max_range: Maximum distance for lidar scan
///     laser/resolution_range: Resolution of laser scan
///     laser/angle_samples: Number of samples to make between min_angle and max_angle
///     laser/noise_mean: Mean of a detected obstacles noise
///     laser/noise_dev: Standard deviation of a detectd obstacle noise
///     red_space (bool): True if certain params are found in red namespace.
///     arena_x_length(double): X length of the arena
///     arena_y_length(double): Y length of the arena
///  public:
///     wheel_radius(double, required): radius of the robot's wheels
///     track_width (double, required): distance between the robot's wheels 
///     motor_cmd_to_rads(double, required): Conversion rate from ticks/secs to rad/sec  
///     collision_radius: Radius of robot's collision bubble
/// PUBLISHES:
///     nusim/timestep (std_msgs::UInt64): The timestep of the simulation
///     red/sensor_data (nuturtlebot_msgs::SensorData): 
///     nusim/obstacles (visualization_msgs::MarkerArray): The list of cylinder Marker obstacles to 
///         display in rviz
///     nusim/walls (visualization_msgs::MarkerArray): The walls of the arena
///     scan (sensor_msgs::LaserScan): Simulated lider data
///     nusim/sim_path (nav_msgs::Path): Path of the simulated robot
///     fake_sensor (visualization_msgs::MarkerArray): noisy simulated obstacles
/// SUBSCRIBER:
///     red/wheel_cmd (nuturtlebot_msgs::WheelCommands): Commanded wheel speeds
/// BROADCASTER:
///     broadcaster (non-static world to red-base_footprint): Transformation matrix from world frame
///         to red-base_footprint frame
/// SERVICES:
///     reset (std_srvs::Empty): Call this service to reset the simulation's timestep and
///         the position of the robot back to x0, y0, theta0
///     teleport (nusim::Teleport): Given a x, y, and theta, this service will teleport 
///         the robot to desired pose

//RKS

//Project include
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/laser_utils.hpp"
//C++ includes
#include <cmath>
#include <cstdint>
#include <random>
#include <limits>
#include <vector>
//3rd-party includes
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nusim/Teleport.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "nuturtlebot_msgs/WheelCommands.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/UInt64.h"
#include "std_srvs/Empty.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


//Nusim's Constants
static const int DEFAULT_RATE = 500;
static const std::uint32_t QUEUE_SIZE = 1000;
static const double DEFAULT_X = 0.0;
static const double DEFAULT_Y = 0.0;
static const double DEFAULT_THETA = 0.0;
static const double DEFAULT_RADIUS = 0.0;
static const std::vector<double> DEFAULT_OBS_LIST;
static const double CYLINDER_HEIGHT = 0.25;
static const double WALL_THICKNESS = 0.1;
static const double WALL_HEIGHT = 0.25;
static const bool DEFAULT_SIM_SPACE = true;
static const int ENCODER_RESOLUTION = 4096;
static const double DEFAULT_VEL_MEAN = 0.0;
static const double DEFAULT_VEL_VARIANCE = 0.0;
static const double DEFAULT_WHEEL_SLIP = 1.0;
static const double DEFAULT_OBS_RANGE = 100.0;
static const double DEFAULT_COLLISION_RADIUS = 0.5;
static const double DEFAULT_LASER_MIN_ANGLE = 0.0;
static const double DEFAULT_LASER_MAX_ANGLE = turtlelib::PI*2.0;
static const double DEFAULT_LASER_MIN_RANGE = 0.120;
static const double DEFAULT_LASER_MAX_RANGE = 3.5;
static const double DEFAULT_LASER_RESOLUTION = 0.015;
static const int DEFAULT_ANGLE_SAMPLES = 360;
static const double DEFAULT_LASER_NOISE_MEAN = 0.0;
static const double DEFAULT_LASER_NOISE_DEV = 0.01;

// Nusim Node's variables
static std::uint64_t timestep = 0;
static int rate;
static double x_init;
static double y_init;
static double theta_init;
static double motor_cmd_to_rads;
static double collision_radius;
static turtlelib::DiffDrive diff_drive;
static nuturtlebot_msgs::SensorData sensor_data;
static double arena_x_length;
static double arena_y_length; 
static double vel_mean;
static double vel_variance;
static double slip_min;
static double slip_max;
static double basic_sensor_variance;
static double max_range;
static visualization_msgs::MarkerArray cylinders;
static visualization_msgs::MarkerArray walls;  
static bool publish_fake_sensor = false;
static visualization_msgs::MarkerArray fake_sensor_readings;
static bool publish_laser_sensor = false;
static double laser_min_angle;
static double laser_max_angle;
static double laser_min_range;
static double laser_max_range;
static double laser_resolution;
static int laser_angle_samples;
static double laser_noise_mean;
static double laser_noise_dev;
static std::vector<float> laser_ranges;


// Nusim Node's functions

/// \brief Create a random number device for C++ distribution functions
/// Originally by Matt Elwin for SLAM 495
/// \return Return random number device
std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}


/// \brief Cycle through known objects to determine if the robot has collided
/// If the robot has collided, adjust robot's position
void collision_detection(turtlelib::DiffDrive & dd, double robot_radius)
{
    for (visualization_msgs::Marker cylin : cylinders.markers)
    { 
        turtlelib::Vector2D obs_pose{cylin.pose.position.x, cylin.pose.position.y};
        bool collision = turtlelib::resolve_collision(dd, 
                                                      robot_radius, 
                                                      obs_pose,
                                                      cylin.scale.x);
        if (collision)
        {
            break; //We only consider one collision per call
        }
    }
}


//Nusim Node's callbacks

/// \brief Callback fcn for reset service. Reset simulation
///
/// \param req - Empty Request
/// \param res - Empty Response 
/// \returns true if service completes successfully
bool reset(std_srvs::Empty::Request &,
           std_srvs::Empty::Response &)
{
    timestep = 0;
    diff_drive.set_configuration(theta_init, x_init, y_init);
    return true;
}


/// \brief Callback fcn teleport service. Teleports robot
///
/// \param req - Teleport Request
/// \param res - Teleport Response 
/// \returns true if service completes successfully
bool teleport(nusim::Teleport::Request &req,
              nusim::Teleport::Response &)
{
    diff_drive.set_configuration(req.theta, req.x, req.y);
    return true;
}


/// \brief Callback fcn to wheel_cmd subscriber. Grabs the current
/// wheel velocity commands in ticks and updates the internal position
/// of the robots and encoder angles
void wheel_cmd_handler(const nuturtlebot_msgs::WheelCommands& msg)
{
    //Convert ticks to rads
    double new_left_rads = static_cast<double>(msg.left_velocity)/static_cast<double>(rate);
    double new_right_rads = static_cast<double>(msg.right_velocity)/static_cast<double>(rate);

    new_left_rads *= motor_cmd_to_rads;
    new_right_rads *= motor_cmd_to_rads;

    // Add noisy component to velocity in rad/sec
    if (!(turtlelib::almost_equal(new_left_rads, 0.0) && turtlelib::almost_equal(new_right_rads, 0.0)))
    {
        std::normal_distribution<double> gauss(vel_mean, vel_variance);
        double vel_noise = gauss(get_random());
        new_left_rads += vel_noise;
        new_right_rads += vel_noise;
    }

    //Update configuration
    diff_drive.apply_fw_kin_vel(new_left_rads, new_right_rads);

    //Resolve any collisions
    collision_detection(diff_drive, collision_radius);

    //Calculate wheel slip 
    std::uniform_real_distribution<double> uni(slip_min, slip_max);
    //Convert new velocity to ticks
    double left_vel_ticks = new_left_rads / motor_cmd_to_rads;
    double right_vel_ticks = new_right_rads / motor_cmd_to_rads;

    //Update sensor_data angle (add slip noise)
    sensor_data.left_encoder += static_cast<int>(left_vel_ticks*uni(get_random()));
    sensor_data.right_encoder += static_cast<int>(right_vel_ticks*uni(get_random()));
    //Normalize encoder angle between 0 ~ 4095
    sensor_data.left_encoder = sensor_data.left_encoder % ENCODER_RESOLUTION;
    sensor_data.right_encoder = sensor_data.right_encoder % ENCODER_RESOLUTION;

}


/// \brief Timer callback to publish fake_sensor_readings. These readings provide
/// noisy landmark locations. 
/// \param TimerEvent - ros TimerEvent required for function, not used.
void prep_fake_sensor(const ros::TimerEvent&)
{
    //Set up Transformation for world-robot
    turtlelib::Transform2D Twr {diff_drive.location(), diff_drive.theta()};
    turtlelib::Transform2D Trw = Twr.inv();
    //Set up normal distribution
    std::normal_distribution<double> gauss(0.0, basic_sensor_variance);

    // Read from obstacle array cylinder
    for (visualization_msgs::Marker cylin: cylinders.markers)
    {
        //Fill out basic infomation
        visualization_msgs::Marker reading;
        reading.header.stamp = ros::Time::now();
        reading.header.frame_id = "red-base_footprint";
        reading.ns = "fake_sensor"; //Namespace in fake_sensor
        reading.id = cylin.id;
        reading.type = cylin.type;
        reading.pose.position.z = cylin.pose.position.z;
        reading.pose.orientation.x = cylin.pose.orientation.x;
        reading.pose.orientation.y = cylin.pose.orientation.y;
        reading.pose.orientation.z = cylin.pose.orientation.z;
        reading.pose.orientation.w = cylin.pose.orientation.w;
        reading.scale.x = cylin.scale.x;
        reading.scale.y = cylin.scale.y;
        reading.scale.z = cylin.scale.z;
        // Make sensor reading green for visualization
        reading.color.r = 1;
        reading.color.g = 1;
        reading.color.b = 0;
        reading.color.a = 1.0;
        // Update location of marker, make position relative, and add noise
        turtlelib::Transform2D Two {turtlelib::Vector2D{cylin.pose.position.x, cylin.pose.position.y}, 0};
        turtlelib::Transform2D Tro = Trw * Two;
        reading.pose.position.x = (Tro.translation().x) + gauss(get_random());
        reading.pose.position.y = (Tro.translation().y) + gauss(get_random());
        double distance = std::sqrt(pow(reading.pose.position.x,2.0) + pow(reading.pose.position.y,2.0));
        //Check if marker in range
        if (distance <= max_range) 
        {
            reading.action = visualization_msgs::Marker::ADD;
        } else {
            reading.action = visualization_msgs::Marker::DELETE;
        }
        //Update reading
        fake_sensor_readings.markers.at(reading.id) = reading;
    }

    //Set publish signal high
    publish_fake_sensor = true;
}


/// \brief Populates the LaserScan lidar topic with information
/// about the robot's environment 
/// \param TimerEvent - ros TimerEvent required for function, not used.
void prep_laser_sensor(const ros::TimerEvent&)
{

    //Set Transformation frame for world-robot
    turtlelib::Transform2D Twr {diff_drive.location(), diff_drive.theta()};
    turtlelib::Transform2D Trw = Twr.inv();
    
    //Set up normal distribution
    std::normal_distribution<double> gauss(laser_noise_mean, laser_noise_dev);

    // Built empty ranges
    std::vector<float> range_values;
    double current_angle = 0.0;
    double angle_increment = (laser_max_angle - laser_min_angle)/laser_angle_samples;
    // Loop through lasers
    for (int i = 0; i < laser_angle_samples; i++) ////LOOP
    {
        // Set up collision checkers
        bool collision = false;
        turtlelib::Vector2D collision_pt {0.0, 0.0};

        // Get points for laser scanning segment
        double x_min = laser_min_range*std::cos(current_angle);
        double y_min = laser_min_range*std::sin(current_angle);
        double x_max = laser_max_range*std::cos(current_angle);
        double y_max = laser_max_range*std::sin(current_angle);

        //Check obstacles
        for (visualization_msgs::Marker cylin: cylinders.markers)
        {
            if (collision)
            {
                break; //If we found a collision, end loop
            }
            turtlelib::Vector2D cylin_r = Trw(turtlelib::Vector2D{cylin.pose.position.x, cylin.pose.position.y});
            collision = turtlelib::check_obs_intersection(x_min, y_min, x_max, y_max,
                                                            cylin_r.x, cylin_r.y, cylin.scale.x,
                                                            collision_pt);
            
        }

        // Check walls
        // Check if previous collision
        if (!collision)
        {
            for (visualization_msgs::Marker wall: walls.markers)
            {
                if (collision)
                {
                    break; //If we found a collision, end loop  
                } 

                //Check for Front/West Wall
                if (turtlelib::almost_equal(wall.pose.position.y, 0.0) && wall.pose.position.x > 0)
                {
                    turtlelib::Vector2D wall_pt1_r = Trw(turtlelib::Vector2D{arena_x_length/2.0, -arena_y_length/2.0});
                    turtlelib::Vector2D wall_pt2_r = Trw(turtlelib::Vector2D{arena_x_length/2.0, arena_y_length/2.0});
                    collision = turtlelib::check_wall_intersection(x_min, y_min, x_max, y_max,
                                                                    wall_pt1_r.x, wall_pt1_r.y,
                                                                    wall_pt2_r.x, wall_pt2_r.y,
                                                                    collision_pt);
                } 
                // Check for Back/East Wall
                else if (turtlelib::almost_equal(wall.pose.position.y, 0.0) && wall.pose.position.x < 0)
                {
                    turtlelib::Vector2D wall_pt1_r = Trw(turtlelib::Vector2D{-arena_x_length/2.0, -arena_y_length/2.0});
                    turtlelib::Vector2D wall_pt2_r = Trw(turtlelib::Vector2D{-arena_x_length/2.0, arena_y_length/2.0});
                    collision = turtlelib::check_wall_intersection(x_min, y_min, x_max, y_max,
                                                                    wall_pt1_r.x, wall_pt1_r.y,
                                                                    wall_pt2_r.x, wall_pt2_r.y,
                                                                    collision_pt);
                }

                // Check for Right/North Wall
                else if (turtlelib::almost_equal(wall.pose.position.x, 0.0) && wall.pose.position.y < 0)
                {
                    turtlelib::Vector2D wall_pt1_r = Trw(turtlelib::Vector2D{arena_x_length/2.0, -arena_y_length/2.0});
                    turtlelib::Vector2D wall_pt2_r = Trw(turtlelib::Vector2D{-arena_x_length/2.0, -arena_y_length/2.0});
                    collision = turtlelib::check_wall_intersection(x_min, y_min, x_max, y_max,
                                                                    wall_pt1_r.x, wall_pt1_r.y,
                                                                    wall_pt2_r.x, wall_pt2_r.y,
                                                                    collision_pt);
                }

                // Check for Right/North Wall
                else if (turtlelib::almost_equal(wall.pose.position.x, 0.0) && wall.pose.position.y > 0)
                {
                    turtlelib::Vector2D wall_pt1_r = Trw(turtlelib::Vector2D{arena_x_length/2.0, arena_y_length/2.0});
                    turtlelib::Vector2D wall_pt2_r = Trw(turtlelib::Vector2D{-arena_x_length/2.0, arena_y_length/2.0});
                    collision = turtlelib::check_wall_intersection(x_min, y_min, x_max, y_max,
                                                                    wall_pt1_r.x, wall_pt1_r.y,
                                                                    wall_pt2_r.x, wall_pt2_r.y,
                                                                    collision_pt);
                }   
            }
        }

        //Get range within laser resolution
        double range_value = turtlelib::magnitude(collision_pt);
        if (!turtlelib::almost_equal(range_value, 0.0))
        {
            range_value += laser_resolution/2.0;
            range_value -= std::fmod(range_value, laser_resolution);
            
            // Apply noise
            range_value += gauss(get_random()); 
            
            //Cap range within min and max value
            if (range_value > laser_max_range) 
            { 
                range_value = laser_max_range;
            }
            else if (range_value < laser_min_range) 
            { 
                range_value = laser_min_range;
            }
        } else {
            range_value = std::numeric_limits<sensor_msgs::LaserScan::_range_max_type>::infinity();
        }

        //append it
        range_values.push_back(range_value);

        //Update angle
        current_angle = turtlelib::normalize_angle(current_angle + angle_increment);
    }

    //Update ranges
    laser_ranges = range_values;
    // Set pubish flag to true
    publish_laser_sensor = true;
}


//Nusim Main Function
int main(int argc, char *argv[])
{
    //Setup node
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~"); //Private scoping node handler
    ros::NodeHandle pub_nh;
    tf2_ros::TransformBroadcaster br; 

    //Local obstacle params
    std::vector<double> obs_x;
    std::vector<double> obs_y;
    double obs_radius;

    //Local diff_drive params
    double wheel_radius = 0.0;
    double track_width = 0.0;
    
    //Local namespacing params
    bool red_space = true;

    //Get Params
    //Rate
    nh.param("rate", rate, DEFAULT_RATE);
    //Robot position
    nh.param("x0", x_init, DEFAULT_X);
    nh.param("y0", y_init, DEFAULT_Y);
    nh.param("theta0", theta_init, DEFAULT_THETA);
    //Obstacale position
    nh.param("obstacles/obs_x", obs_x, DEFAULT_OBS_LIST);
    nh.param("obstacles/obs_y", obs_y, DEFAULT_OBS_LIST);
    nh.param("obstacles/radius", obs_radius, DEFAULT_RADIUS);
    //Distribution params
    nh.param("dist/vel_mean", vel_mean, DEFAULT_VEL_MEAN);
    nh.param("dist/vel_variance", vel_variance, DEFAULT_VEL_VARIANCE);
    nh.param("dist/slip_min", slip_min, DEFAULT_WHEEL_SLIP);
    nh.param("dist/slip_max", slip_max, DEFAULT_WHEEL_SLIP);
    //Sensor params
    nh.param("sensor/basic_sensor_variance", basic_sensor_variance, DEFAULT_VEL_VARIANCE);
    nh.param("sensor/max_range", max_range, DEFAULT_OBS_RANGE);
    //Namespace params
    nh.param("red_space", red_space, DEFAULT_SIM_SPACE);
    //Laser sensor params
    nh.param("laser/min_angle", laser_min_angle, DEFAULT_LASER_MIN_ANGLE);
    nh.param("laser/max_angle", laser_max_angle, DEFAULT_LASER_MAX_ANGLE);
    nh.param("laser/min_range", laser_min_range, DEFAULT_LASER_MIN_RANGE);
    nh.param("laser/max_range", laser_max_range, DEFAULT_LASER_MAX_RANGE);
    nh.param("laser/resolution_range", laser_resolution, DEFAULT_LASER_RESOLUTION);
    nh.param("laser/angle_samples", laser_angle_samples, DEFAULT_ANGLE_SAMPLES);
    nh.param("laser/noise_mean", laser_noise_mean, DEFAULT_LASER_NOISE_MEAN);
    nh.param("laser/noise_dev", laser_noise_dev, DEFAULT_LASER_NOISE_DEV);

    //Get Arena Param
    if(!nh.getParam("arena/x_length", arena_x_length))
    {
        ROS_ERROR_STREAM("Nusim: Cannot find x_length for arena");
        return(1);
    }

    if(!nh.getParam("arena/y_length", arena_y_length))
    {
        ROS_ERROR_STREAM("Nusim: Cannot find y_length for arena");
        return(1);
    }

    //Get DiffDrive params
    if (red_space) //If our parameters are in red namespace
    {
        if (!pub_nh.getParam("red/wheel_radius", wheel_radius))
        {
        ROS_ERROR_STREAM("Nusim: Cannot find wheel_radius"); 
        return(1); //return 1 to indicate error
        }

        if (!pub_nh.getParam("red/track_width", track_width))
        {
        ROS_ERROR_STREAM("Nusim: Cannot find track_width"); 
        return(1); //return 1 to indicate error
        }

        if (!pub_nh.getParam("red/motor_cmd_to_rads", motor_cmd_to_rads))
        {
        ROS_ERROR_STREAM("Nusim: Cannot find motor_cmd_to_rads"); 
        return(1); //return 1 to indicate error
        } 

        pub_nh.param("red/collision_radius", collision_radius, DEFAULT_COLLISION_RADIUS);

    } else { //If parameters are not in red space

        if (!pub_nh.getParam("wheel_radius", wheel_radius))
        {
        ROS_ERROR_STREAM("Nusim: Cannot find wheel_radius"); 
        return(1); //return 1 to indicate error
        }

        if (!pub_nh.getParam("track_width", track_width))
        {
        ROS_ERROR_STREAM("Nusim: Cannot find track_width"); 
        return(1); //return 1 to indicate error
        }

        if (!pub_nh.getParam("motor_cmd_to_rads", motor_cmd_to_rads))
        {
        ROS_ERROR_STREAM("Nusim: Cannot find motor_cmd_to_rads"); 
        return(1); //return 1 to indicate error
        } 

        pub_nh.param("collision_radius", collision_radius, DEFAULT_COLLISION_RADIUS);
    }


    //Build ROS Objects
    //Publishers
    const auto timestep_pub = nh.advertise<std_msgs::UInt64>("timestep", QUEUE_SIZE);
    const auto encoder_pub = pub_nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", QUEUE_SIZE);
    const auto cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", QUEUE_SIZE, true);
    const auto walls_pub = nh.advertise<visualization_msgs::MarkerArray>("walls", QUEUE_SIZE, true);
    const auto laser_pub = pub_nh.advertise<sensor_msgs::LaserScan>("scan", 100, true);
    const auto path_pub = nh.advertise<nav_msgs::Path>("sim_path",QUEUE_SIZE, true);
    //Subscribers
    const auto wheel_cmd_sub = pub_nh.subscribe("red/wheel_cmd", QUEUE_SIZE, wheel_cmd_handler);
    //Services
    const auto reset_srv = nh.advertiseService("reset", reset);
    const auto teleport_srv = nh.advertiseService("teleport", teleport);
    //Timers
    const auto laser_time = nh.createTimer(ros::Duration(0.2), prep_laser_sensor);
    geometry_msgs::TransformStamped ts;

    //Set up Rate object
    ros::Rate loop_rate(rate);

    //Init diff drive
    diff_drive.set_wheel_config(track_width, wheel_radius);
    diff_drive.set_configuration(theta_init, x_init, y_init);

    //Publish Obstacle Markers
    if (obs_radius > 0.0 && obs_x.size() > 0.0 && obs_x.size() == obs_y.size())
    {
        for(long unsigned int i=0;i < obs_x.size(); i++)
        {
            visualization_msgs::Marker cylin;
            cylin.header.stamp = ros::Time::now();
            cylin.header.frame_id = "world";
            cylin.ns = "obstacles";
            cylin.id = i;
            cylin.type = visualization_msgs::Marker::CYLINDER;
            cylin.action = visualization_msgs::Marker::ADD;
            cylin.pose.position.x = obs_x[i];
            cylin.pose.position.y = obs_y[i];
            cylin.pose.position.z = CYLINDER_HEIGHT/2.0;
            cylin.pose.orientation.x = 0.0;
            cylin.pose.orientation.y = 0.0;
            cylin.pose.orientation.z = 0.0;
            cylin.pose.orientation.w = 1.0;
            cylin.scale.x = obs_radius;
            cylin.scale.y = obs_radius;
            cylin.scale.z = CYLINDER_HEIGHT;
            cylin.color.r = 1.0;
            cylin.color.g = 0.0;
            cylin.color.b = 0.0;
            cylin.color.a = 1.0;
            //Populate static cylinders and noisy fake_sensor_readings
            cylinders.markers.push_back(cylin);
            fake_sensor_readings.markers.push_back(cylin);
        }
    }
    //Publish static obstacles
    cylinder_pub.publish(cylinders);

    //Create fake_sensor_publisher: this relies on cylinders so let cylinders be init
    const auto fake_sensor_pub = pub_nh.advertise<visualization_msgs::MarkerArray>("fake_sensor", 100, true); 
    const auto fake_sensor_time = nh.createTimer(ros::Duration(0.2), prep_fake_sensor);

    //Publish Wall Markers      
    //Front Wall (West Wall)
    visualization_msgs::Marker front_wall;
    front_wall.header.stamp = ros::Time::now();
    front_wall.header.frame_id = "world";
    front_wall.ns = "walls";
    front_wall.id = 0;
    front_wall.type = visualization_msgs::Marker::CUBE;
    front_wall.action = visualization_msgs::Marker::ADD;
    front_wall.pose.position.x = arena_x_length/2.0 + WALL_THICKNESS/2.0;
    front_wall.pose.position.y = 0;
    front_wall.pose.position.z = WALL_HEIGHT/2.0;
    front_wall.pose.orientation.x = 0.0;
    front_wall.pose.orientation.y = 0.0;
    front_wall.pose.orientation.z = 0.0;
    front_wall.pose.orientation.w = 1.0;
    front_wall.scale.x = WALL_THICKNESS;
    front_wall.scale.y = arena_y_length + 2.0*WALL_THICKNESS;
    front_wall.scale.z = WALL_HEIGHT;
    front_wall.color.r = 1.0;
    front_wall.color.g = 0.0;
    front_wall.color.b = 0.0;
    front_wall.color.a = 1.0;
    walls.markers.push_back(front_wall);

    //Back Wall (East Wall)
    visualization_msgs::Marker back_wall;
    back_wall.header.stamp = ros::Time::now();
    back_wall.header.frame_id = "world";
    back_wall.ns = "walls";
    back_wall.id = 1;
    back_wall.type = visualization_msgs::Marker::CUBE;
    back_wall.action = visualization_msgs::Marker::ADD;
    back_wall.pose.position.x = -arena_x_length/2.0 - WALL_THICKNESS/2.0;
    back_wall.pose.position.y = 0;
    back_wall.pose.position.z = WALL_HEIGHT/2.0;
    back_wall.pose.orientation.x = 0.0;
    back_wall.pose.orientation.y = 0.0;
    back_wall.pose.orientation.z = 0.0;
    back_wall.pose.orientation.w = 1.0;
    back_wall.scale.x = WALL_THICKNESS;
    back_wall.scale.y = arena_y_length + 2.0*WALL_THICKNESS;
    back_wall.scale.z = WALL_HEIGHT;
    back_wall.color.r = 1.0;
    back_wall.color.g = 0.0;
    back_wall.color.b = 0.0;
    back_wall.color.a = 1.0;
    walls.markers.push_back(back_wall);

    //Right Wall (North Wall)
    visualization_msgs::Marker right_wall;
    right_wall.header.stamp = ros::Time::now();
    right_wall.header.frame_id = "world";
    right_wall.ns = "walls";
    right_wall.id = 2;
    right_wall.type = visualization_msgs::Marker::CUBE;
    right_wall.action = visualization_msgs::Marker::ADD;
    right_wall.pose.position.x = 0;
    right_wall.pose.position.y = -arena_y_length/2.0 - WALL_THICKNESS/2.0;
    right_wall.pose.position.z = WALL_HEIGHT/2.0;
    right_wall.pose.orientation.x = 0.0;
    right_wall.pose.orientation.y = 0.0;
    right_wall.pose.orientation.z = 0.0;
    right_wall.pose.orientation.w = 1.0;
    right_wall.scale.x = arena_x_length + 2.0*WALL_THICKNESS;
    right_wall.scale.y = WALL_THICKNESS;
    right_wall.scale.z = WALL_HEIGHT;
    right_wall.color.r = 1.0;
    right_wall.color.g = 0.0;
    right_wall.color.b = 0.0;
    right_wall.color.a = 1.0;
    walls.markers.push_back(right_wall);

    //Left Wall (South Wall)
    visualization_msgs::Marker left_wall;
    left_wall.header.stamp = ros::Time::now();
    left_wall.header.frame_id = "world";
    left_wall.ns = "walls";
    left_wall.id = 3;
    left_wall.type = visualization_msgs::Marker::CUBE;
    left_wall.action = visualization_msgs::Marker::ADD;
    left_wall.pose.position.x = 0;
    left_wall.pose.position.y = arena_y_length/2.0 + WALL_THICKNESS/2.0;
    left_wall.pose.position.z = WALL_HEIGHT/2.0;
    left_wall.pose.orientation.x = 0.0;
    left_wall.pose.orientation.y = 0.0;
    left_wall.pose.orientation.z = 0.0;
    left_wall.pose.orientation.w = 1.0;
    left_wall.scale.x = arena_x_length + 2.0*WALL_THICKNESS;
    left_wall.scale.y = WALL_THICKNESS;
    left_wall.scale.z = WALL_HEIGHT;
    left_wall.color.r = 1.0;
    left_wall.color.g = 0.0;
    left_wall.color.b = 0.0;
    left_wall.color.a = 1.0;
    walls.markers.push_back(left_wall);

    //Pub Walls
    walls_pub.publish(walls);

    //Init Laser Scanner
    sensor_msgs::LaserScan laserscan;
    laserscan.header.frame_id = "red-base_footprint";
    laserscan.angle_min = laser_min_angle;
    laserscan.angle_max = laser_max_angle;
    laserscan.angle_increment = (laser_max_angle - laser_min_angle)/laser_angle_samples;
    laserscan.time_increment = 1.73e-5;
    laserscan.scan_time = laserscan.time_increment * laser_angle_samples;
    laserscan.range_min = laser_min_range;
    laserscan.range_max = laser_max_range;
    
    //Init Sim Path
    nav_msgs::Path sim_path;
    sim_path.header.frame_id = "world";
    double past_x = diff_drive.location().x;
    double past_y = diff_drive.location().y;
    double past_theta = diff_drive.theta();
    //Main loop
    while(ros::ok())
    {
        //Update data
        timestep++; //this means we start at T = 1

        //Populate transform
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "red-base_footprint";
        ts.transform.translation.x = diff_drive.location().x;
        ts.transform.translation.y = diff_drive.location().y;
        ts.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, diff_drive.theta());
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        br.sendTransform(ts);

        //Package and send data
        std_msgs::UInt64 msg;
        msg.data = timestep;
        timestep_pub.publish(msg);
        encoder_pub.publish(sensor_data);

        //Fake sensor publishing
        if (publish_fake_sensor)
        {
            publish_fake_sensor = false;
            fake_sensor_pub.publish(fake_sensor_readings);
        }

        // LIDAR publishing
        if (publish_laser_sensor)
        {
            publish_laser_sensor = false;
            laserscan.header.stamp = ros::Time::now();
            laserscan.ranges = laser_ranges;
            laser_pub.publish(laserscan);     
        }

        // Sim path publishing
        if(!turtlelib::almost_equal(past_x, diff_drive.location().x) ||
            !turtlelib::almost_equal(past_y, diff_drive.location().y) ||
            !turtlelib::almost_equal(past_theta, diff_drive.theta()))
        {
            //Update past reference pts
            past_x = diff_drive.location().x;
            past_y = diff_drive.location().y;
            //Create new pose
            geometry_msgs::Pose new_pose{};
            new_pose.position.x = diff_drive.location().x;
            new_pose.position.y = diff_drive.location().y;
            tf2::Quaternion qp;
            qp.setRPY(0, 0, diff_drive.theta());
            new_pose.orientation.x = qp.x();
            new_pose.orientation.y = qp.y();
            new_pose.orientation.z = qp.z();
            new_pose.orientation.w = qp.w();
            
            //Create new pose stamped
            geometry_msgs::PoseStamped new_pose_stamped{};
            new_pose_stamped.pose = new_pose;
            sim_path.header.stamp = ros::Time::now();
            new_pose_stamped.header = sim_path.header;
            sim_path.poses.push_back(new_pose_stamped);
            path_pub.publish(sim_path);
        }

        //Grab data
        ros::spinOnce();

        //Sleep 
        loop_rate.sleep();
        
    }

    return(0);

}
