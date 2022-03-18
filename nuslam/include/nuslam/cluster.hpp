#ifndef CLUSTER_DETECTION_INCLUDE_GUARD_HPP
#define CLUSTER_DETECTION_INCLUDE_GUARD_HPP
/// \file 
/// \brief Implementation of the clustering algorithm in slam_ml namespace

//Project
#include "turtlelib/rigid2d.hpp"
// Standard C++ includes
#include <vector>

//3rd Party includes


namespace slam_ml 
{
    /// \brief Produce clusters from ranges 
    /// \param ranges - a list of ranges and corresponding to angles 
    /// \param angle_increment - angle increment per each point [radians]
    /// \param dist_thres - distance threshold for clusterin [meters]
    /// \return clusters of points
    std::vector<std::vector<turtlelib::Vector2D>> produce_clusters(std::vector<float> ranges, 
                                                                            double angle_increment,
                                                                            double dist_thres);
    
    /// \brief determines if the cluster is an circle/arc
    /// \param cluster - collection of points represented as Vector2D
    /// \param mean_lower_bound - lower bound of angle mean value
    /// \param mean_higher_bound - higher bound of angle mean value
    /// \param std_dev_thres - upper bound of std dev for angles
    /// \return True if cluster is identified as circle or arc
    bool classify_circle(std::vector<turtlelib::Vector2D> cluster,
                        double mean_lower_bound,
                        double mean_higher_bound,
                        double std_dev_thres);
}

#endif //CLUSTER_DETECTION_INCLUDE_GUARD_HPP