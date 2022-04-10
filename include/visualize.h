#ifndef VISUALIZE_H_
#define VISUALIZE_H_

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

namespace RL_PLANNER_NS {

class Visualize 
{
    public:
        Visualize() 
        {
            pubPath = n.advertise<visualization_msgs::MarkerArray>("/planned_path", 10);
        }

        void pub_path();
        void pack_marker_msg(std::vector<std::vector<double>> &pathVector_nx3); // 打包成可视化用的msg

    private:
        ros::NodeHandle n;
        ros::Publisher pubPath; // 发布器
        geometry_msgs::PoseArray msgPath;
        visualization_msgs::MarkerArray msgVisPath; // 用于可视化的路径

};
}
#endif // VISUALIZE_H
