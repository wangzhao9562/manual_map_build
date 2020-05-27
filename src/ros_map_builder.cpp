/*
 * @Author: Zhao Wang
 * @Date: 2020-05-27 18:38:41
 * @LastEditTime: 2020-05-27 22:47:12
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
    nh.param("inscribed_distance", inscribed_dist_, 5.0);
    nh.param("circumscribed_distance", circumscribed_dist_, 10.0);
    nh.param("factor", factor_, 1.0);

    // Create two value map 
    tv_map_ = new TwoValueMap(width, height, resolution, ur_vertexes, ll_vertexes);
    occ_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("occ_grid", 1);

    nav_msgs::OccupancyGrid occ_grid;
    // Transform costs
    twoValueToMapCosts(occ_grid, origin, ur_vertexes, ll_vertexes, tv_map_->getCosts());
    // Publish occupancy grid
    occGridPub(occ_grid);
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
    // Compute new costs
    std::vector<unsigned int> obs_vec = std::move(getObsIndex(ur_vertexes, ll_vertexes));
    std::vector<double> min_dist_vec = std::move(minObsDist(two_value_cost, obs_vec));
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
        occ_grid.data.push_back(cost);
    }
}

std::vector<unsigned int> RosMapBuilder::getObsIndex(const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes)
{
    int size = ur_vertexes.size();
    std::vector<unsigned int> obs_vec(width_ * height_, 0);
    for(unsigned int row = ll_vertexes[0].first; row < ur_vertexes[size - 1].first; ++row)
    {
        for(unsigned int col = ll_vertexes[0].second; col < ur_vertexes[size - 1].second; ++col)
        {
            if(row >= width_ || col >= height_)
            {
                exit(1); // if index is overflow, exit
            }
            unsigned int index = col * width_ + row;
            obs_vec[index] = 1;
        }
    }
    return obs_vec;
}

std::vector<double> RosMapBuilder::minObsDist(const std::vector<unsigned int>& cost_vec, const std::vector<unsigned int>& obs_vec)
{
    if(cost_vec.size() != obs_vec.size())
    {
        exit(1);
    }
    std::vector<double> min_dist_vec(cost_vec.size(), -1);
    for(unsigned int vec_id = 0; vec_id < cost_vec.size(); ++vec_id)
    {
        if(obs_vec[vec_id] != 1)
        {
            min_dist_vec[vec_id] = getMinObsDistance(vec_id, obs_vec);
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
        unsigned int obs_x = obs_vec[vec_id] % width_;
        unsigned int obs_y = obs_vec[vec_id] / width_;
        double dist = distance(x, y, obs_x, obs_y); 
        dist_vec.push_back(dist);
    }
    std::make_heap(dist_vec.begin(), dist_vec.end(), std::greater<int>());
    dist_vec.pop_back();
    return dist_vec.at(dist_vec.size() - 1);
}

std::vector<unsigned int> RosMapBuilder::projectCosts(const std::vector<double>& dist_vec)
{
    std::vector<unsigned int> costs(dist_vec.size(), 0);
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

} // end of ns