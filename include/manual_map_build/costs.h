/*
 * @Author: Zhao Wang
 * @Date: 2020-05-28 13:15:03
 * @LastEditTime: 2020-05-28 13:20:20
 * @LastEditors: Please set LastEditors
 * @Description: Define the value of costs
 * @FilePath: /manual_map_build/include/manual_map_build/cost.h
 */ 
#ifndef COSTS_H_
#define COSTS_H_

namespace manual_map_build{
    // For costmap
    static unsigned int UNKNOWN = 255;
    static unsigned int LETHAL_COST = 254;
    static unsigned int INSCRIBED_COST = 253;
    // For two value map
    static unsigned int OCCUPIED_COST = 1;
    static unsigned int FREE_COST = 0;
} // end of ns

#endif
