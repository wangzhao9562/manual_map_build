/*
 * @Author: Zhao Wang
 * @Date: 2020-05-27 18:38:41
 * @LastEditTime: 2020-05-28 22:16:46
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of RosMapBuilder
 * @FilePath: /manual_map_build/src/ros_map_builder.cpp
 */ 

#include<manual_map_build/ros_map_builder.h>
#include<algorithm>
#include<functional>

namespace manual_map_build{

RosMapBuilder::RosMapBuilder(unsigned int width, unsigned int height, double resolution, Origin origin,
        const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes) : width_(width), height_(height), res_(resolution)
{
    // Load parameters
    ros::NodeHandle nh;
    nh.param("map_frame", map_frame_, std::string("/map"));
    nh.param("inscribed_distance", inscribed_dist_, 2.0);
    nh.param("circumscribed_distance", circumscribed_dist_, 5.0);
    nh.param("factor", factor_, 1.0);
    nh.param("occ_th", occ_th_, 0.65);
    nh.param("free_th", free_th_, 0.196);

    // Create two value map 
    tv_map_ = new TwoValueMap(width, height, resolution, ur_vertexes, ll_vertexes);
    occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("occ_grid", 1);

    nav_msgs::OccupancyGrid occ_grid;
    // Transform costs
    twoValueToMapCosts(occ_grid, origin, ur_vertexes, ll_vertexes, tv_map_->getCosts());
    // Publish occupancy grid
    while(ros::ok()){
        occGridPub(occ_grid);
    }
}

RosMapBuilder::RosMapBuilder(const std::string& file_path)
{
    /* Reserved for implemention */
}

RosMapBuilder::~RosMapBuilder()
{
    if(tv_map_)
    {
        delete tv_map_;
        tv_map_ = nullptr;
    }
}

void RosMapBuilder::twoValueToMapCosts(nav_msgs::OccupancyGrid& occ_grid, Origin origin,
    const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes, 
    const std::vector<unsigned int>& two_value_cost)
{
    // Compute minimum distance between free grids and obstacle grids
    std::vector<double> min_dist_vec = std::move(minObsDist(two_value_cost));
    // Compute cost of each grid
    std::vector<unsigned int> costs = std::move(projectCosts(min_dist_vec));

    // Create occupied grid object
    occ_grid.header.frame_id = map_frame_;
    occ_grid.header.stamp = ros::Time::now();
    occ_grid.info.map_load_time = ros::Time::now();
    occ_grid.info.resolution = res_;
    occ_grid.info.width = width_;
    occ_grid.info.height = height_;
    occ_grid.info.origin.position.x = origin.first;
    occ_grid.info.origin.position.y = origin.second;
    occ_grid.info.origin.position.z = 0.0;
    occ_grid.info.origin.orientation.w = 1.0;

    // Add cost value
    for(auto cost : costs)
    {
        double occ = cost / 255.0;
        if(occ > occ_th_)
        {
            occ_grid.data.push_back(100);
        }
        else if(occ < free_th_)
        {
            occ_grid.data.push_back(0);
        }
        else
        {
            double ratio = (occ - free_th_) / (occ_th_ - free_th_);
            occ_grid.data.push_back(ratio * 99);
        }
    }
}

std::vector<double> RosMapBuilder::minObsDist(const std::vector<unsigned int>& cost_vec)
{
    std::vector<double> min_dist_vec(cost_vec.size(), -1);
    for(unsigned int vec_id = 0; vec_id < cost_vec.size(); ++vec_id)
    {
        if(cost_vec[vec_id] != 1)
        {
            min_dist_vec[vec_id] = getMinObsDistance(vec_id, cost_vec);
        }
    }
    return min_dist_vec;
}

double RosMapBuilder::getMinObsDistance(unsigned int index, const std::vector<unsigned int>& obs_vec)
{
    std::vector<double> dist_vec;
    unsigned int x = index % width_;
    unsigned int y = index / width_;
    for(int vec_id = 0; vec_id < obs_vec.size(); ++vec_id)
    {
        if(obs_vec[vec_id] == 1)
        {
            unsigned int obs_x = vec_id % width_;
            unsigned int obs_y = vec_id / width_;
            double dist = distance(x, y, obs_x, obs_y);
            dist_vec.push_back(dist);
        }
    }
    std::make_heap(dist_vec.begin(), dist_vec.end(), std::greater<int>());
    std::pop_heap(dist_vec.begin(), dist_vec.end());
    // dist_vec.pop_back();
    return dist_vec.at(dist_vec.size() - 1);
}

std::vector<unsigned int> RosMapBuilder::projectCosts(const std::vector<double>& dist_vec)
{
    std::vector<unsigned int> costs(dist_vec.size(), 0);
    // Compute cost, cost = (253 - 1) * e(k), k = -f * (distance - inscribed_distance)
    for(int vec_id = 0; vec_id < dist_vec.size(); ++vec_id)
    {
        double dist = dist_vec.at(vec_id);
        if(dist != -1)
        {
            if(dist > inscribed_dist_)
            {
                double k = -factor_ * (dist - inscribed_dist_);
                costs[vec_id] = (INSCRIBED_COST - 1) * std::exp(k);
            }
            if(dist > 0 && dist <= inscribed_dist_)
            {
                costs[vec_id] = INSCRIBED_COST;
            }
        }
        else{
            costs[vec_id] = LETHAL_COST;
        }
    }
    return costs;
}
  
    
void RosMapBuilder::occGridPub(const nav_msgs::OccupancyGrid& occ_grid)
{
    occ_grid_pub_.publish(occ_grid);
}

template<typename T>
void RosMapBuilder::testPrintVec(const std::vector<T>& vec){
    for(auto ele : vec)
    {
        std::cout << ele << " ";
    }
    std::cout;
}

} // end of ns