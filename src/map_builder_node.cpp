/*
 * @Author: Zhao Wang
 * @Date: 2020-05-28 14:27:38
 * @LastEditTime: 2020-05-28 22:01:51
 * @LastEditors: Please set LastEditors
 * @Description: Ros node
 * @FilePath: /manual_map_build/src/map_builder_node.cpp
 */ 
#include <manual_map_build/ros_map_builder.h>
#include <string.h>
#include <vector>
#include <utility>
#include <stdlib.h>

typedef std::pair<unsigned int, unsigned int> POINT;

int main(int argc, char** argv){
    ros::init(argc, argv, "manual_map_builder_node");
    ros::NodeHandle nh;

    // Prepare parameteres for map builder
    double origin_x, origin_y;
    double resolution;
    unsigned int width, height;
    std::vector<POINT> ur_points, ll_points;

    ROS_INFO("Start parameter parsing");
    ROS_INFO_STREAM("Num of command: " << argc);
    ROS_INFO_STREAM("First command: " << argv[1]);

    /* Reserved for implementation of parameter loading from file */

    int arg_id = 1;
    while(arg_id < argc)
    {
        // Parse origin point 
        if(std::strcmp(argv[arg_id], "-o") == 0)
        {
            ROS_INFO("Parse origin point");
            if(arg_id + 1 < argc)
            {
                origin_x = std::atof(argv[arg_id + 1]);
                ROS_INFO_STREAM("origin x: " << origin_x);
            }
            else
            {
                ROS_ERROR("Missing parameter for origin point");
                exit(1);
            }
            if(arg_id + 2 < argc)
            {
                origin_y = std::atof(argv[arg_id + 2]);
                ROS_INFO_STREAM("origin_y: " << origin_y);
            }
            else
            {
                ROS_ERROR("Missing parameter for origin point");
                exit(1);
            }
            arg_id += 2;
        }
        // Parse width
        if(std::strcmp(argv[arg_id], "-w") == 0)
        {
            ROS_INFO("Parse width");
            if(arg_id + 1 < argc)
            {
                width = static_cast<unsigned int>(std::atoi(argv[arg_id + 1]));
                ROS_INFO_STREAM("width: " << width);
            }
            else
            {
                ROS_ERROR("Missing parameter for width");
                exit(1);
            }
            ++arg_id;
        }
        // Parse height
        if(std::strcmp(argv[arg_id], "-h") == 0)
        {
            ROS_INFO("Parse height");
            if(arg_id + 1 < argc)
            {
                height = static_cast<unsigned int>(std::atoi(argv[arg_id + 1]));
                ROS_INFO_STREAM("height: " << height);
            }
            else
            {
                ROS_ERROR("Missing parameter for height");
                exit(1);
            }
            ++arg_id;
        }
        // Parse resolution 
        if(std::strcmp(argv[arg_id], "-r") == 0)
        {
            ROS_INFO("Parse resolution");
            if(arg_id + 1 < argc)
            {
                resolution = std::atof(argv[arg_id + 1]);
                ROS_INFO_STREAM("resolution: " << resolution);
            }
            else
            {
                ROS_ERROR("Missing parameter for resolution");
                exit(1);
            }
            ++arg_id;
        }
        // Parse upper right points
        if(std::strcmp(argv[arg_id], "-ur") == 0)
        {
            ROS_INFO("Parse upper right points");
            int ur_id = arg_id;
            ROS_INFO_STREAM("ur_id: " << ur_id);
            while(std::strcmp(argv[ur_id + 1], "-ll") != 0)
            {
                // Ensure there are at least two parameters for upper right points
                if(ur_id + 2 < argc)
                {
                    POINT p;
                    p.first = static_cast<unsigned int>(std::atoi(argv[ur_id + 1]));
                    p.second = static_cast<unsigned int>(std::atoi(argv[ur_id + 2]));
                    ur_points.push_back(p);
                    ur_id += 2;
                }
                else{
                    break;
                }
            }
            ++ur_id;
            ROS_INFO_STREAM("Number of upper right points: " << ur_points.size());
            arg_id = ur_id;
            ROS_INFO_STREAM("arg_id: " << arg_id);
        }
        // Parse lower left points
        if(std::strcmp(argv[arg_id], "-ll") == 0)
        {
            ROS_INFO("Parse lower left points");
            int ll_id = arg_id;
            while(ll_id < argc)
            {
                if(ll_id + 2 < argc)
                {
                    POINT p;
                    p.first = static_cast<unsigned int>(std::atoi(argv[ll_id + 1]));
                    p.second = static_cast<unsigned int>(std::atoi(argv[ll_id + 2]));
                    ll_points.push_back(p);
                    ll_id += 2;
                }
                else{
                    break;
                }
                // ++ll_id;
            }
            ++ll_id;
            ROS_INFO_STREAM("Number of lower left points: " << ll_points.size());
            arg_id = ll_id;
        }
        ++arg_id;
    }

    // Initialize map builder
    try{
        ROS_INFO("Create costmap builder");
        manual_map_build::RosMapBuilder(width, height, resolution, std::make_pair(origin_x, origin_y), ur_points, ll_points);
    }
    catch(std::exception& e)
    {
        ROS_ERROR("Error in map builder initialization");
        exit(1);
    }
    return 0;
}
