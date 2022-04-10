#include "planner.h"

namespace  RL_PLANNER_NS
{
    // 两个回调函数
    void Planner::set_slot(const geometry_msgs::PoseWithCovarianceStampedConstPtr&  slot_msg)
    {
        slotLen = slot_msg->pose.pose.position.x;
        mcts.slotLen = slotLen;
        std::cout << "I am seeing a new slot :" << slotLen << std::endl;
    }
    void Planner::set_start(const geometry_msgs::PoseWithCovarianceStampedConstPtr&  slot_msg)
    {
        start.header = slot_msg->header;
        start.pose = slot_msg->pose;
        double x = slot_msg->pose.pose.position.x;
        double y = slot_msg->pose.pose.position.y;
        std::cout << "I am seeing a new start x:" << x << " y:" << y << std::endl;
    }

    // 构造函数
    Planner::Planner():mcts() // 调用MCTS的初始化
    {
        pubStart = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/parking_start_pose", 1);
        pubEnd = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/parking_end_pose", 1);
        subSlot = n.subscribe("/slot_detected", 1, &Planner::set_slot, this);
        subStart = n.subscribe("/initialpose", 1, &Planner::set_start, this);
    } 

   // 规划函数
    void Planner::plan() 
    {
        // 调用MCTS成员执行规划
        // 返回规划结果
        // 进行可视化
        
        // -----------------------------------------------------
        // 测试NN
        // Eigen::Matrix<double,5,1> tmp;
        // Eigen::Matrix<double,15,1> outtmp;
        // tmp << 2.5, 1.75, 0.0, 0.0, 0.0;
        //  mcts.fcnn.forward_func(tmp, outtmp );
        // for (size_t i = 0; i < outtmp.size(); i++)
        // {
        //     std::cout << outtmp[i] << " ";
        // }
        // std::cout << std::endl;
        // std::cout << outtmp.size()<<std::endl;
        // ------------------------------ 测试通过 和matlab的输出是一样的
        Node2D n_test(2.5, 1.75, 0.);
        // mcts.rollout(n_test, 0, 0);
        mcts.tictoc.tic();
        mcts.set_startnode(n_test);
        mcts.set_startvelo(0);
        mcts.set_startangle(0);
        mcts.one_step_MCTS();
        mcts.tictoc.cout_time_used("planner_plan_func ");
        mcts.visualize_in_MCTS();
        // 上面调用一次了规划的过程
        // 现在可以发布出规划的结果了
        // 发布的topic的形式是 nav_msgs/path
        // visualize();    // TODO 先可视化一下自己发出的结果

    }

    // 发布可视化的topic
    void Planner::visualize()
    {
        // res.clear();
        // // 人为设置路点
        // for (int i = 0; i < 10; i++)
        // {
        //     std::vector<double> v;
        //     v.clear();
        //     v.push_back(i + 0.0);
        //     v.push_back(i+0.0);
        //     v.push_back(i+0.0);
        //     res.push_back(v);
        // }
        // vis.pack_marker_msg(res);
        // vis.pub_path();

        // 
        return;
    }
}