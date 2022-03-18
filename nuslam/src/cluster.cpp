#include "nuslam/cluster.hpp"
#include "turtlelib/rigid2d.hpp"
#include <cmath>
#include <iostream>

namespace slam_ml
{
    std::vector<std::vector<turtlelib::Vector2D>> produce_clusters(std::vector<float> ranges, 
                                                                        double angle_increment,
                                                                        double dist_thres)
    {
        //init
        unsigned long int max_index = ranges.size()-1;
        std::vector<std::vector<turtlelib::Vector2D>> clusters{};
        double prev_x = static_cast<double>(ranges.at(0));
        double prev_y = 0.0;
        std::vector<turtlelib::Vector2D> cluster{};
        cluster.push_back(turtlelib::Vector2D{prev_x, prev_y});

        //Loop
        for (unsigned long int i = 1; i < ranges.size(); i++)
        {
            //Get x and y
            double x = static_cast<double>(ranges.at(i)*std::cos(turtlelib::normalize_angle(i*angle_increment)));
            double y = static_cast<double>(ranges.at(i)*std::sin(turtlelib::normalize_angle(i*angle_increment)));
            //Vector of prev_x,y to x,y
            turtlelib::Vector2D vec{prev_x - x, prev_y - y};
            //Feed current cluster
            if (turtlelib::magnitude(vec) <= dist_thres)
            {
                cluster.push_back(turtlelib::Vector2D{x,y});
            //Get new cluster
            } else {
                clusters.push_back(cluster);
                cluster = std::vector<turtlelib::Vector2D>{};
                cluster.push_back(turtlelib::Vector2D{x,y});
            }
            prev_x = x;
            prev_y = y;
            
            if (i == max_index){
                double first_x = static_cast<double>(ranges.at(0));
                double first_y = 0.0;
                turtlelib::Vector2D{first_x - x, first_y - y};
                if (turtlelib::magnitude(vec) <= dist_thres)
                {
                    if (cluster.size() > 1) //last point part of a cluster
                    {
                        //add the cluster to first cluster
                        clusters.front().insert(clusters.front().end(), cluster.begin(), cluster.end());
                    } else { // last point by itself
                        clusters.front().push_back(turtlelib::Vector2D{x,y});
                    }
                }
            } 
        }

        //Filter small clusters
        std::vector<std::vector<turtlelib::Vector2D>> rtn_clusters{};
        for (auto curr_cluster : clusters)
        {
            if (curr_cluster.size() > 3)
            {
                rtn_clusters.push_back(curr_cluster);
            }
        }

        return rtn_clusters;
    }

    bool classify_circle(std::vector<turtlelib::Vector2D> cluster, 
                        double mean_lower_bound, 
                        double mean_higher_bound,
                        double std_dev_thres)
    {
        //Get endpoints
        double first_x = cluster.front().x;
        double first_y = cluster.front().y;
        double last_x = cluster.back().x;
        double last_y = cluster.back().y;
        std::vector<double> angles{};
        double angles_sum = 0.0;

        //Get angles
        for (unsigned long int i = 1; i < cluster.size()-1; i++)
        {
            turtlelib::Vector2D P1{first_x - cluster.at(i).x, first_y - cluster.at(i).y};
            turtlelib::Vector2D P2{last_x - cluster.at(i).x, last_y - cluster.at(i).y};
            double angle = turtlelib::angle(P1, P2);
            angles_sum += angle;
            angles.push_back(angle);
        }
        //Get mean
        double angles_mean = angles_sum/static_cast<double>(cluster.size());

        double angles_var_sum = 0.0;
        //Get standard devation
        for (auto angle : angles)
        {
            angles_var_sum += std::pow(angle - angles_mean, 2.0);
        }
        angles_var_sum = angles_var_sum / static_cast<double>(cluster.size());
        double angles_std_dev = std::sqrt(angles_var_sum);

        //Identify if we're a circle/arc
        bool rtn_val = (angles_std_dev <= std_dev_thres) && ((angles_mean > mean_lower_bound) && (angles_mean < mean_higher_bound));
        return rtn_val;
    }
}