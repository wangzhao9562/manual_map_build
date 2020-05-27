/*
 * @Author: Zhao Wang
 * @Date: 2020-05-26 15:56:02
 * @LastEditTime: 2020-05-27 22:53:44
 * @LastEditors: Please set LastEditors
 * @Description: Definition of RosMapBuilder class which transfrom two value map into occupancy grid
 * @FilePath: /manual_map_build/include/manual_map_build/ros_map_builder.h
 */ 
#ifndef ROS_MAP_BUILDER_H_
#define ROS_MAP_BUILDER_H_

#include<ros/ros.h>
#include<manual_map_build/two_value_map.h>
#include<nav_msgs/OccupancyGrid.h>
#include<string>
#include<math.h>

namespace manual_map_build{

unsigned int UNKNOWN = 255;
unsigned int LETHAL_COST = 254;
unsigned int INSCRIBED_COST = 253;

/**
 * @brief Build ros type map manually
 */
class RosMapBuilder{
typedef std::pair<double, double> Origin;
public:
    /**
     * @brief Constructor
     * @param width Map width
     * @param height Map height
     * @param resolution Resolution of map
     * @param origin Origin point of map
     * @param ur_vertexes Upper right vertexes of obstacle areas 
     * @param ll_vertexes Lower left vertexes of obstacle areas
     */
    RosMapBuilder(unsigned int width, unsigned int height, double resolution, Origin origin,
        const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes);

    /**
     * @brief Constructor
     * @param filename Parameter file
     */    
    RosMapBuilder(const std::string& filename);

    /**
     * @brief Deconstructor
     */
    ~RosMapBuilder();

private:
    /**
     * @brief Transform two value cost into occupancy grid cost
     * @param occ_grid Returned occupancy grid type object
     * @param origin Origin point of map
     * @param ur_vertexes Upper right vertexes of obstacle areas 
     * @param ll_vertexes Lower left vertexes of obstacle areas
     * @param two_value_cost Cost of transformed two value map
     */
    void twoValueToMapCosts(nav_msgs::OccupancyGrid& occ_grid, Origin origin,
        const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes, 
        const std::vector<unsigned int>& two_value_cost);
    
    /**
     * @brief Get index vector of obstacles
     * @param ur_vertexes Vector of upper right vertexes of obstacle
     * @param ll_vertexes Vector of lower left vertexes of obstacle
     * @param return Vector index of all obstacle in map
     */
    std::vector<unsigned int> getObsIndex(const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes);

    /**
     * @brief Get minimum distance between all free grids and occupied grids
     * @param cost_vec Cost of two value map
     * @param obs_vec Index vector of obstacle
     * @param return Minimum distance vector
     */
    std::vector<double> minObsDist(const std::vector<unsigned int>& cost_vec, const std::vector<unsigned int>& obs_vec);

    /**
     * @brief Compute minimum distance between grid with input index and occupied grids
     * @param index Index of ordered grid
     * @param obs_vec Index vector of occupied grids
     * @return The minimum distance 
     */
    double getMinObsDistance(unsigned int index, const std::vector<unsigned int>& obs_vec);

    /**
     * @brief Project two value costs into costmap 
     * @param dist_vec Minimum distance vector between free grids and occupied grids
     * @param return Costs of transformed costmap
     */
    std::vector<unsigned int> projectCosts(const std::vector<double>& dist_vec);

    /**
     * @brief Euclidean distance
     */
    constexpr double distance(const unsigned int x1, const unsigned int y1, const unsigned int x2, const unsigned y2)
    {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    /**
     * @brief Publish occupancy grid information
     */ 
    void occGridPub(const nav_msgs::OccupancyGrid& occ_grid);

private:
    TwoValueMap* tv_map_; // two value map object

    unsigned int width_, height_;
    double res_;

    double inscribed_dist_, circumscribed_dist_;
    double factor_;

    std::string map_frame_; 
    ros::Publisher occ_grid_pub_; // occupancy grid publisher
}; // end of class

} // end of ns

#endif
