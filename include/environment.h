#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <iostream>
// #include <ctime>
#include <vector>
// #include <ros/ros.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include "visualize.h"


namespace RL_PLANNER_NS
{
    class Node2D
    {
        public:
            double x;
            double y;
            double th;

            Node2D()
            { x = 0.0;
                y = 0.;
                th = 0.;
            };
            Node2D(double x_, double y_, double th_) : x(x_), y(y_), th(th_){};

            // Node2D(Node2D &node) 
            // { 
            //     x = node.x;
            //     y = node.y;
            //     th = node.th;
            // }
            // Node2D& operator=(Node2D &node) // 拷贝初始化返回引用
            // {
            //     this->x = node.x;
            //    this-> y = node.y;
            //     this-> th = node.th;
            // }
            ~Node2D(){};
    };

    class Obstacle // 车辆障碍物类
    {
    private:
        std::vector< Node2D> obs;

    public:
        Obstacle()
        {
        };
        ~Obstacle(){};
    };

   class Environment // 描述泊车环境
    {
        private:
            std::vector<Obstacle> env;

        public:
            Environment(/* args */)
            {
              //  env = std::vector<Obstacle>(2, Obstacle());
            };
            ~Environment(){};

            double slotLen = -1.0;
    };
}
#endif