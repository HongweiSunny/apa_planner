#ifndef MCTS_H_
#define MCTS_H_
#include <iostream>
#include <vector>
// #include "environment.h"
#include "vehicle.h"
#include "NN.h"
#include <Eigen/Eigen>
#include <cmath>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "tic_toc.h"
#include <tf/tf.h>
#include <algorithm>

using namespace std;
// MCTS是一个one-step的过程
// 其主要的功能是进行rollout
namespace RL_PLANNER_NS
{
// class ValueIndex // 用于排序并且返回索引
// {

// }

class MCTS
{
    public:
        MCTS()
        {
            pubTrajMCTS = nMCTS.advertise<nav_msgs::Path>("/MCTS/planned_traj", 1, true); // ros相关的在构造函数里面初始化一下
        };
        MCTS(double x, double y, double th, double v, double steer) : startNode(x, y, th), fcnn(), carInMCTS()
        {
            startSteer = steer;
            startVelo = v;
            slotLen = 4.57; // 默认4.57
            yIni = y;
            pubTrajMCTS = nMCTS.advertise<nav_msgs::Path>("/MCTS/planned_traj", 1, true);
        };
        void expand_one_layer(Node2D& old, Node2D& newnode, double angleOrder, double veloOrder); // 往下拓展一层
        double rollout(Node2D, double, double); // 返回一次rollout的回报值
        void one_step_MCTS(); // 返回多次rollout的回报值
        
        double get_reward(Node2D&, double); // 计算回报值
        bool reach_target(Node2D&);
        bool stop_judge_special(Node2D&, double);
        bool collision_detect(double x, double y, double th) ; // Collison Detection
        void set_yini(double);
        void set_slot(double);
        void set_startnode(Node2D &);
        void set_startangle(double);
        void set_startvelo(double);

        // 可视化结果
        void visualize_in_MCTS();

    public:
        NN fcnn;           // NN对象
        Vehicle carInMCTS; // 基本的参数和运动学模型

        TicToc tictoc;

        // private:
        int lenRollout = 0;
        double maxReward = -100000.0;
        std::vector<Node2D> traj;     // 最大回报对应的结果
        std::vector<Node2D> traj_now; // 临时 直接被rollout操作
        std::vector<double> trajVelo;
        std::vector<double> trajVelo_now;
        std::vector<double> trajSteer;
        std::vector<double> trajSteer_now;
        Node2D startNode;
        double startVelo;
        double startSteer;
        double slotLen = 4.57; // 默认4.57
        double yIni = 1.75;

        // ros 相关成员
        ros::NodeHandle nMCTS;
        ros::Publisher pubTrajMCTS;


};

} // END NS
#endif