/*
 * @Author: Zhao Wang
 * @Date: 2020-05-22 15:49:57
 * @LastEditTime: 2020-05-22 17:09:29
 * @LastEditors: Please set LastEditors
 * @Description: Map with two types of value structure
 * @FilePath: /manual_map_build/include/manual_map_build/two_value_map.h
 */ 

#ifndef TWO_VALUE_MAP_H_
#define TWO_VALUE_MAP_H_

#include <vector>
#include <utility>

namespace two_value_map{
static const unsigned int OCCUPIED = 1;
static const unsigned int FREE = 0;

typedef std::pair<unsigned int, unsigned int> Vertex;

/**
 * @brief Map structure with two types of costs: 0 or 1
 */
class TwoValueMap{
public:
    /**
     * @param width Map width
     * @param height Map height
     * @param resolution Map resolution
     * @param ur_vertexes Upper right vertexes integration of obstacles
     * @param ll_vertexes Lower left vertexes integration of obstacles
     */
    TwoValueMap(unsigned int width, unsigned int height, double resolution, const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes);
    
private:
    /**
     * @param ur_vertexes Upper right vertexes integration of obstacles
     * @param ll_vertexes Lower left vertexes integration of obstacles
     */
    void mapBuild(const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes);

private:
    unsigned int w_, h_; // height and width of map
    double res_; // resolution
    std::vector<unsigned int> costs_; // costs of map
}; // end of class
}; // end of ns

#endif
