#include "nuslam/cluster.hpp"
#include "turtlelib/rigid2d.hpp"
#include <cmath>
#include <iostream>

namespace slam_ml
{
    std::vector<std::vector<std::pair<double, double>>> produce_clusters(std::vector<double> ranges, 
                                                                        double angle_increment,
                                                                        double dist_thres)
    {
        //init
        unsigned long int max_index = ranges.size()-1;
        std::vector<std::vector<std::pair<double, double>>> clusters{};
        double prev_x = ranges.at(0);
        double prev_y = 0;
        std::vector<std::pair<double, double>> cluster{};
        cluster.push_back(std::pair<double, double>{prev_x, prev_y});

        //Loop
        for (unsigned long int i = 1; i < ranges.size(); i++)
        {
            //Get x and y
            double x = ranges.at(i)*std::cos(turtlelib::normalize_angle(i*angle_increment));
            double y = ranges.at(i)*std::sin(turtlelib::normalize_angle(i*angle_increment));
            //Vector of prev_x,y to x,y
            turtlelib::Vector2D vec{prev_x - x, prev_y - y};
            //Feed current cluster
            if (turtlelib::magnitude(vec) <= dist_thres)
            {
                cluster.push_back(std::pair<double, double>{x,y});
            //Get new cluster
            } else {
                clusters.push_back(cluster);
                cluster = std::vector<std::pair<double, double>>{};
                cluster.push_back(std::pair<double, double>{x,y});
            }
            prev_x = x;
            prev_y = y;
            
            if (i == max_index){
                double first_x = ranges.at(0);
                double first_y = 0;
                turtlelib::Vector2D{first_x - x, first_y - y};
                if (turtlelib::magnitude(vec) <= dist_thres)
                {
                    if (cluster.size() > 1) //last point part of a cluster
                    {
                        //add the cluster to first cluster
                        clusters.front().insert(clusters.front().end(), cluster.begin(), cluster.end());
                    } else { // last point by itself
                        clusters.front().push_back(std::pair<double, double>{x,y});
                    }
                }
            } 
        }

        //Filter small clusters
        std::vector<std::vector<std::pair<double, double>>> rtn_clusters{};
        for (auto curr_cluster : clusters)
        {
            if (curr_cluster.size() > 3)
            {
                rtn_clusters.push_back(curr_cluster);
            }
        }

        return rtn_clusters;
    }
}