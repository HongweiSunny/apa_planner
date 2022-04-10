#ifndef PLANNER_H_
#define PLANNER_H_

#include <iostream>
#include <ctime>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "visualize.h"
#include "vehicle.h"
#include "environment.h"
#include "NN.h"
#include "MCTS.h"
#include  <Eigen/Eigen>
#include  <string>
using namespace Eigen;

namespace RL_PLANNER_NS
 {
    class Planner
    {
        public:
            Planner();
            virtual ~Planner(){};
            void set_start();
            void set_slot();
            void plan();
            void visualize();

        private:
            ros::NodeHandle n;
            ros::Publisher pubStart;   // 成员驼峰 类首字母大写的驼峰  函数下划线分割
            ros::Publisher pubRes; 
            ros::Publisher pubEnd; 
            ros::Subscriber subStart;
            ros::Subscriber subSlot;
            tf::TransformListener listener;
            tf::StampedTransform transform;
            
            Vehicle car; // mcts里面也有一个vehicle
            
            Environment env;

            Visualize vis; // 用于可视化的类  发布topic到rviz

            
            MCTS mcts; // 内部含有NN的成员  或者 NN作为一个参数给MCTS使用


            std::vector<std::vector<double>> res; // x y th 存储计算得到的结果

            geometry_msgs::PoseWithCovarianceStamped start;
            double slotLen = 0.0;
            geometry_msgs::PoseStamped end;
            
            bool validStart = false; /// Flags for allowing the planner to plan
            bool validSlot = false;
            bool validSuccess = false;

            // callback func
            void set_start(const geometry_msgs::PoseWithCovarianceStampedConstPtr& start);
            void set_slot(const geometry_msgs::PoseWithCovarianceStampedConstPtr& slot);


    };
}; // namespace RL_PLANNER_NS

#endif
