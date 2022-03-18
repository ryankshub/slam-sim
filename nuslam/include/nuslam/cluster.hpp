#ifndef CLUSTER_DETECTION_INCLUDE_GUARD_HPP
#define CLUSTER_DETECTION_INCLUDE_GUARD_HPP
/// \file 
/// \brief Implementation of the clustering algorithm in slam_ml namespace

//Project

// Standard C++ includes
#include <vector>
#include <utility>
//3rd Party includes


namespace slam_ml 
{
    /// \brief Produce clusters from ranges 
    /// \param ranges - a list of ranges and corresponding to angles 
    /// \param angle_increment - angle increment per each point
    /// \param dist_thres - distance threshold for clusterin
    /// \return clusters of points
    std::vector<std::vector<std::pair<double, double>>> produce_clusters(std::vector<double> ranges, 
                                                                            double angle_increment,
                                                                            double dist_thres);
}

#endif //CLUSTER_DETECTION_INCLUDE_GUARD_HPP