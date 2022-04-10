#include "vehicle.h"
namespace RL_PLANNER_NS
{
    double Vehicle::get_length() const
    {
        return length_;
    }
    Vehicle::~Vehicle(){};

    void Vehicle::kinematic_model(double &old_x, double &old_y, double old_th, double &new_x, double &new_y, double &new_th, double &angle_order, double &velo_order)
    {
        double dt = 0.05; // 50ms
        double x_dot = velo_order * dt * std::cos(old_th);
        double y_dot = velo_order * dt * std::sin(old_th);
        double th_dot = velo_order * dt * std::tan(angle_order/steering_ratio_/180.0*M_PI)/wheelbase_;

        new_x = old_x + x_dot;
        new_y = old_y + y_dot;
        new_th = old_th+ th_dot;
    }

    void Vehicle::kinematic_model(Node2D &old_s, Node2D &new_s,    double &angle_order, double &velo_order)
    {
        double dt = 0.05; // 50ms
        double old_x = old_s.x;
        double old_y = old_s.y;
        double old_th = old_s.th;
        double x_dot = velo_order * dt * std::cos(old_th);
        double y_dot = velo_order * dt * std::sin(old_th);
        double th_dot = velo_order * dt * std::tan(angle_order/steering_ratio_/180.0*M_PI)/wheelbase_;

        new_s.x = old_x + x_dot;
        new_s.y = old_y + y_dot;
        new_s.th = old_th+ th_dot;
    }
}