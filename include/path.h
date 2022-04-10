#ifndef PathATH_H_
#define PathATH_H_
#include <iostream>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace RL_PLANNER_NS
{
    class Path  /// 路径类 由一系列的点组成
    {
    private:
        ros::NodeHandle n;
        /// Publisher for the path as 
        ros::Publisher pubPath;
        /// Publisher for the nodes on the path
        ros::Publisher pubPathNodes;
        /// Publisher for the vehicle along the path
        ros::Publisher pubPathVehicles;
        /// Path data structure for visualization
        nav_msgs::Path path;
        /// Nodes data structure for visualization
        visualization_msgs::MarkerArray pathNodes;
        /// Vehicle data structure for visualization
        visualization_msgs::MarkerArray pathVehicles;
        /// Value that indicates that the path is smoothed/post processed
        bool smoothed = false;
    public:
        Path(/* args */);
        ~Path();
    };
    
    Path::Path(/* args */)
    {
    }
    
    Path::~Path()
    {
    }
    
}


#endif