#include "visualize.h"

namespace RL_PLANNER_NS 
{
    void Visualize:: pub_path()
    {
        pubPath.publish(msgVisPath);
    }

    void Visualize::pack_marker_msg(std::vector<std::vector<double>> &pathVector_nx3) // 打包成可视化用的msg
    {
        msgVisPath.markers.clear();
        visualization_msgs::Marker costCube;
        ros::Time t = ros::Time::now();
        for (size_t i = 0; i < pathVector_nx3.size(); i++)
        {
                costCube.header.frame_id = "slot";
                costCube.header.stamp = t;
                costCube.id = i; // 用id区分开各个marker
                costCube.type = visualization_msgs::Marker::CUBE; // 形状
                costCube.pose.position.x = pathVector_nx3[i][0];
                costCube.pose.position.y = pathVector_nx3[i][1];
                costCube.pose.position.z = pathVector_nx3[i][2];
                costCube.scale.x = 0.1;
                costCube.scale.y = 0.1;
                costCube.scale.z = 0.1;
                costCube.color.a = 0.6;
                costCube.color.r = 255;
                costCube.color.g = 0;
                costCube.color.b = 0;
                costCube.lifetime = ros::Duration(0); // 0代表永远
                msgVisPath.markers.push_back(costCube);
        }

        std::cout << msgVisPath.markers.size() << std::endl;
    }
}
