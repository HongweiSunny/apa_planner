#include "MCTS.h"

namespace RL_PLANNER_NS
{
    double MCTS::rollout(Node2D startNodeOfRollout, double startSteerOfRollout, double startVeloOfRollout)
{
    double rewMax = -10000.0;
    bool flagStop = false;
    Node2D stateNow ( startNodeOfRollout );
    Node2D stateNext(0.,0.,0.); //  赋值会触发默认拷贝构造函数  赋值不会触发  
    double angleNow = startSteerOfRollout;
    double veloNow = startVeloOfRollout;
    while (!flagStop)
    {
        // 根据NN选择指令 ==TODO==
        // Eigen::Matrix<double, 15, 1> probs;//
        // 输入是先转角再车速
        Eigen::Matrix<double, 5, 1> stateMatrix;
        stateMatrix << stateNow.x, stateNow.y, stateNow.th, angleNow, veloNow;

        fcnn.forward_func(stateMatrix); // 重载
        // int indexAction = fcnn.get_max_prob_rule();
        int indexAction = fcnn.get_max_prob_rule();
        int indexVelo = (indexAction-1) / 5 + 1;
        int indexAngle = indexAction - (indexVelo - 1) * 5;
        // 得到指令
        double angleOrder = angleNow + 10.0 * (indexAngle - 3) ;
        double veloOrder = veloNow + 0.015 * (indexVelo - 2);
        // 范围限制
        angleOrder = angleOrder >= 530. ? 530. : angleOrder;
        angleOrder = angleOrder <= -530. ? -530. : angleOrder;
        veloOrder = veloOrder >= 0.5 ? 0.5 : veloOrder;
        veloOrder = veloOrder <= -1.0 ? -1.0 : veloOrder;
        // 从当前状态生成新的状态
        carInMCTS.kinematic_model(stateNow, stateNext, angleOrder, veloOrder);

        // 判断是否停止
        flagStop = stop_judge_special(stateNext, traj_now.size());
        

        // // 记录轨迹
        // Node2D node_tmp = stateNext;
        traj_now.push_back(stateNext); // 为什么不能直接塞进去？??????? ==TODO== 需要给一个const的
        trajVelo_now.push_back(veloOrder);
        trajSteer_now.push_back(angleOrder);

        // 更新节点和转角和车速
        angleNow = angleOrder;
        veloNow = veloOrder;
        stateNow = stateNext;
    }
    // 计算回报值
    double reRollout = get_reward(stateNow, traj_now.size());

}

void MCTS:: expand_one_layer(Node2D& old, Node2D& newnode, double angleOrder, double veloOrder) // 只往下拓展一层
{
    carInMCTS.kinematic_model(old.x, old.y, old.th, newnode.x, newnode.y, newnode.th, angleOrder, veloOrder);
}

void MCTS::one_step_MCTS()
{
    // 按照NN输出的概率的大小来依次寻找
    Matrix<double, 15, 1> probStart;
    Eigen::Matrix<double, 5, 1> stateMatrix;
    stateMatrix << startNode.x, startNode.y, startNode.th, startSteer, startVelo;
    fcnn.forward_func(stateMatrix, probStart);
    // 排序 利用lambda表达式
    // sort()
    vector<vector<double>> probsVec;
    vector<double> probValueIndex;
    for (size_t i = 0; i < probStart.size(); i++)
    {
        probValueIndex.clear();
        probValueIndex.push_back(probStart(i, 0));
        probValueIndex.push_back( i * 1.0);
        probsVec.push_back(probValueIndex);
    }
    sort(probsVec.begin(), probsVec.end(), [](vector<double> x, vector<double> y) { return x[0] > y[0]; });
    // for (size_t i = 0; i < probsVec.size(); i++)
    // {
    //     cout << probsVec[i][0] << " " << probsVec[i][1] << endl;
    // }

    // 输入是先转角再车速
    for (size_t i = 0; i < 15; i++) // 先走一条测试下 == TODO ==
    {
        // 先从当前位置往下拓展一层 ==TODO==
        Node2D stateSecondLayer;
        
        // 拓展层的指令选择
        // int indexAction = i + 1; // 不排序时
        int indexAction = probsVec[i][1]  + 1; // 排序时
        


        int indexVelo = (indexAction - 1) / 5 + 1;
        int indexAngle = indexAction - (indexVelo - 1) * 5;
        // 得到指令
        double angleOrder = startSteer + 10.0 * (indexAngle - 3) ;
        double veloOrder = startVelo + 0.015 * (indexVelo - 2);
        // 范围限制
        angleOrder = angleOrder >= 530. ? 530. : angleOrder;
        angleOrder = angleOrder <= -530. ? -530. : angleOrder;
        veloOrder = veloOrder >= 0.5 ? 0.5 : veloOrder;
        veloOrder = veloOrder <= -1.0 ? -1.0 : veloOrder;
        expand_one_layer(startNode, stateSecondLayer, angleOrder, veloOrder);

        // 清空之前的结果
         traj_now.clear();
        trajVelo_now.clear();
        trajSteer_now.clear();
        // 放入第二层的状态
        traj_now.push_back(stateSecondLayer);
        trajVelo_now.push_back(veloOrder);
        trajSteer_now.push_back(angleOrder);

        // 开始rollout
        double reRollout = rollout(stateSecondLayer, angleOrder, veloOrder);

        // 根据回报值更新对应的轨迹
        if (reRollout > maxReward) //
        {
            maxReward = reRollout;
            traj.clear();
            trajVelo.clear();
            trajSteer.clear();
            traj = traj_now;                    // ==TODO== 有改进的空间 也许可以都保留下来 发布出去可视化看一下
            trajVelo = trajVelo_now;
            trajSteer = trajSteer_now;

            // 可以提前停止
            if(traj[traj.size()-1].y < -0.88 &&   traj[traj.size()-1].x < 0 && traj[traj.size()-1].th * 180 /M_PI < 0.3)
            {
                cout << "提前停止在第 " << i+1 << " 次搜索" << endl;
                cout << "Pose Stop Planned: " << endl;
                cout << traj[traj.size() - 1].x << " " << traj[traj.size() - 1].y << " " << traj[traj.size() - 1].th * 180 / M_PI << endl;
                break;
            }
            else
                continue;
        }
    }
    cout << "One-step over !" << endl;
    cout << maxReward << endl;
}

double MCTS::get_reward(Node2D& node, double actionLen)
{
    double res = 0.0;
     double cx0 = node.x + carInMCTS.width_ /2* sin(node.th) - carInMCTS.hangRear * cos(node.th);
    double dx0 = node.x + carInMCTS.width_ / 2 * sin(node.th) - carInMCTS.hangRear * cos(node.th);
    double ax0 = dx0 + carInMCTS.wheelbase_ * cos(node.th);
    double bx0 = cx0 + carInMCTS.wheelbase_ * cos(node.th);
    double cy0 = node.y - carInMCTS.width_ / 2 * cos(node.th) - carInMCTS.hangRear * sin(node.th);
    double dy0 = node.y + carInMCTS.width_ / 2 * cos(node.th) - carInMCTS.hangRear * sin(node.th);
    double ay0 = dy0 + carInMCTS.wheelbase_ * sin(node.th);
    double by0 = cy0 + carInMCTS.wheelbase_ * sin(node.th);
    if(( -2 + carInMCTS.width_) <  ay0 && ay0 < 0 
        && (-2 + carInMCTS.width_) <dy0 && dy0 < 0
         )
        res += 10000;

    res += 20000.0 * exp(-abs(node.y-(-0.9))/0.08);
    res += 20000.0 * exp(-abs(node.th)/0.01);
    res += 1000.0 - 40 * actionLen * 0.05;
    bool c = collision_detect(node.x, node.y, node.th);
    if (c)
        res -= 20000.0;
    return res;
}

bool MCTS::stop_judge_special(Node2D& node, double actionLen )
{
    bool res = false;
    if(actionLen >= 30/0.05)
        return true;
    if(node.y > yIni + 0.2)
        return true;
    if(node.y < carInMCTS.width_ * 0.5 - 2)
        return true;
    if(reach_target(node))
        return true;
    if(collision_detect(node.x, node.y, node.th))
        return true;
    if (node.x < -slotLen + 0.05 + carInMCTS.hangRear)
        return true;
    return res;
}

bool MCTS::collision_detect(double x, double y, double th)
{
    // 投影轴法
    double safe_r = 0.05;
    double safe_f = 0.05;
    //  double obsArea = 10 + safe_r
    Matrix<double, 3, 3> Twr;
    Twr   << cos(th), -1*sin(th), x,
        sin(th), cos(th), y,
        0.0, 0.0, 1.0;
    Matrix<double, 3, 3> Twv = Twr * carInMCTS.Tvr_inv;
    Matrix<double,3,9> pAll_w = Twv * carInMCTS.pAll;


    double xmin = pAll_w(0,0);
    double xmax= pAll_w(0,0);
    double ymin = pAll_w(1,0);
    for (size_t i = 0; i < 9; i++)
    {
        if(pAll_w(0,i) < xmin)
            xmin = pAll_w(0,i);
        if(pAll_w(0,i) >xmax)
            xmax = pAll_w(0,i);
        if(pAll_w(1,i) < ymin)
            ymin = pAll_w(1,i);
    } // 找到最大最小的xy
    // 判断流程
    bool res = false;
    if (xmin <= -slotLen + safe_r)
        return true;
    else  // 保证后面不会撞到
    {
        if(xmin>=-safe_f)
        {
            if(ymin > 0) // 没撞到
                res = false;
            else
                res = true;
        }
        else
        {
            if(xmax >= -safe_f)
            { //查看投影
                Matrix<double, 3, 1> pp7;
                pp7 << pAll_w(0, 6), pAll_w(1, 6), pAll_w(2, 6);
                Matrix<double, 3, 1> pp4;
                pp4 << pAll_w(0, 3), pAll_w(1, 3), pAll_w(2, 3);
                Matrix<double, 3, 1> vectorHead = pp4 - pp7;
                // 7这个点的投影坐标
                double cor7 = pp7[0] * vectorHead[0] + pp7[1] * vectorHead[1];
                cor7 /= sqrt(vectorHead[1] * vectorHead[1] + vectorHead[0] * vectorHead[0]);
                // 障碍点投影
                double corf = -safe_f * vectorHead[0] + 0 * vectorHead[1];
                corf /= sqrt(vectorHead[1] * vectorHead[1] + vectorHead[0] * vectorHead[0]);
                // 比较投影
                if(cor7 > corf)
                {    res = false; }
                else
                {
                    res = true;
                }
            }
            else // 车在库位内了
            {
                res = false; // 
            }
        }
    }

    if(ymin < -2.0 + 0.1) // 路沿防撞
        res = true;

    return res;
  }
bool MCTS::reach_target(Node2D& node)
{
    double cx0 = node.x + carInMCTS.width_ /2* sin(node.th) - carInMCTS.hangRear * cos(node.th);
    double dx0 = node.x + carInMCTS.width_ / 2 * sin(node.th) - carInMCTS.hangRear * cos(node.th);
    double ax0 = dx0 + carInMCTS.wheelbase_ * cos(node.th);
    double bx0 = cx0 + carInMCTS.wheelbase_ * cos(node.th);
    double cy0 = node.y - carInMCTS.width_ / 2 * cos(node.th) - carInMCTS.hangRear * sin(node.th);
    double dy0 = node.y + carInMCTS.width_ / 2 * cos(node.th) - carInMCTS.hangRear * sin(node.th);
    double ay0 = dy0 + carInMCTS.wheelbase_ * sin(node.th);
    double by0 = cy0 + carInMCTS.wheelbase_ * sin(node.th);
    bool res = false;
    if(node.th <= 0.3 * M_PI / 180 
        && ay0 < 0 && dy0 < 0
        && by0 > -2 && cy0 > -2
        && ax0 <0 && bx0 < 0
        && cx0>-slotLen && dx0>-slotLen
        )
        res = true;
    return res;
}

 void MCTS::set_startnode(Node2D &node)
 {
     startNode.x = node.x;
     startNode.y = node.y;
     startNode.th = node.th;
 }
void MCTS::set_startangle(double an)
{
    startSteer = an;
}
void MCTS::set_startvelo(double ve)
{
    startVelo = ve;
}

void MCTS::visualize_in_MCTS()
{
    //
    nav_msgs::Path msgPathMCTS;
    geometry_msgs::PoseStamped p;
    msgPathMCTS.header.frame_id = "slot_frame";
    msgPathMCTS.header.stamp = ros::Time::now(); // 打时间标签
    // msgPathMCTS.header.seq = 1; 好像会自己加一
    for (size_t i = 0; i < traj.size(); i++)
    {
        p.pose.position.x = traj[i].x;
        p.pose.position.y = traj[i].y;
        p.pose.position.z = 0;
        p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, traj[i].th);
        msgPathMCTS.poses.push_back(p);
    }
    pubTrajMCTS.publish(msgPathMCTS);
}
} // namespace RL_PLANNER_NS