#include<iostream>
#include "vehicle.h"
#include "planner.h"

int main(int argc, char **argv)
{
    std::cout << "--[START APPLICATION]\n";
    ros::init(argc, argv, "apa_planner_node");
    RL_PLANNER_NS::Vehicle E50(3.57);
    std::cout << E50.get_length() << std::endl;

    RL_PLANNER_NS::Planner rlplanner;
    ros::Rate rate = ros::Rate(0.1);
    int cnt = 0;
    while (ros::ok() && cnt < 5)
    {
        cnt++;
        ros::spinOnce();
        rlplanner.plan();
        rate.sleep();
    }

    return 0;
}