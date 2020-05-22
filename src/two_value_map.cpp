/*
 * @Author: Zhao Wang
 * @Date: 2020-05-22 16:10:36
 * @LastEditTime: 2020-05-22 17:09:55
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of TwoValueMap
 * @FilePath: /manual_map_build/src/two_value_map.cpp
 */ 
#include<include/manual_map_build.h>

namespace two_value_map{
    TwoValueMap::TwoValueMap(unsigned int width, unsigned int height, double resolution, 
        const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes) : w_(width), h_(height), res_(resolution)
    {
        costs_(w_ * h_, FREE); // initialize map costs
        mapBuild(ur_vertexes, ll_vertexes);
    }

    void TwoValueMap::mapBuild(const std::vector<Vertex>& ur_vertexes, const std::vector<Vertex>& ll_vertexes)
    {
        static_assert(ur_vertexes.size() == ll_vertexes.size(), "Input two vectors should have same length!");

        int len = ur_vertexes.size();
        for(int ind = 0; i < len; ++ind)
        {
            Vertex upper_right, lower_left;
            upper_right = ur_vertexes.at(ind); // get upper right vertex of obstacle region
            lower_left = ll_vertexes.at(ind); // get lower left vertex of obstacle region
            for(unsigned int row = lower_left.first; row < upper_right.first; ++row)
            {
                for(unsigned int col = lower_left.second; col < upper_right.second; ++col)
                {
                    unsigned int index = col * w_ + row;
                    costs_.at(index) = OCCUPIED;
                }
            }
        }
    }

}; // end of ns
