#ifndef VECHILE_H_
#define VECHILE_H_
#include <cmath>
#include "environment.h"
#include <Eigen/Eigen>
using namespace Eigen;
namespace RL_PLANNER_NS
{
    class Vehicle
    {
    public:
        // 基本参数
        double length_ = 3.57;
        double width_ = 1.551;
        double wheelbase_ = 2.305;
        double height_= 1.7;
        double max_velocity_ = 1.0;
        double max_steering_angle_ = 530.0;
        double steering_ratio_ = 16.68;
        double hangRear = 0.544;

        // 八边形的参数
        double a, b, c, d, e, f, g;      
        double p1[3];
        double p2[3];
        double p3[3];
        double p4[3];
        double p5[3];
        double p6[3];
        double p7[3];
        double p8[3];
        double p9[3];

        Matrix<double,3,9> pAll;
        double egoArea;
        Matrix<double, 3, 3> Tvr_inv;
        Matrix<double, 3, 3> Tvr;
    public:
        Vehicle()
        {
            a = 0.16;
            b = 0.365;
            c = 1.551 - 2 * b;
            d = 3.03;
            e = 0.38;
            f = 1.551 - 2 * e;
            g = 0.544;

            p1[0] = 0.;
            p1[1] = b;
            p1[2] = 1.;

            p2[0] = 0.;
            p2[1] = b + c;
            p2[2] = 1.;

            p3[0] = a;
            p3[1] = 2. * b + c;
            p3[2] = 1.;

            p4[0] = a + d;
            p4[1] = 2. * b + c;
            p4[2] = 1.;

            p5[0] = a + e + d;
            p5[1] = f+e;
            p5[2] = 1.;

            p6[0] = a+d+e;
            p6[1] = e;
            p6[2] = 1.;

            p7[0] = a + d;
            p7[1] = 0.;
            p7[2] = 1.;

            p8[0] = a;
            p8[1] = 0.;
            p8[2] = 1.;

            p9[0] = g;
            p9[1] = b + c / 2.;
            p9[2] = 1.;

            egoArea = (2.*b+c)*(a+e+d) - a*b - e*e;
            Tvr << 1., 0., g,
                0., 1., b + c / 2.,
                0., 0. ,1.;
            Tvr_inv = Tvr.inverse();
            for (size_t i = 0; i < 3; i++)
            {
                pAll(i, 0) = p1[i];
                pAll(i, 1) = p2[i];
                pAll(i, 2) = p3[i];
                pAll(i, 3) = p4[i];
                pAll(i, 4) = p5[i];
                pAll(i, 5) = p6[i];
                pAll(i, 6) = p7[i];
                pAll(i, 7) = p8[i];
                pAll(i, 8) = p9[i];
            }
    };
        Vehicle(double lenght):length_(lenght)
        {
            width_ = wheelbase_ = height_ = max_velocity_ = max_steering_angle_ = steering_ratio_ = 0.0;
        };
        virtual ~Vehicle();
        double  get_length() const ;
        void kinematic_model(double &old_x, double &old_y, double old_th, 
                                                        double &new_x, double &new_y, double &new_th, 
                                                            double &angle_order, double &velo_order);
        void kinematic_model(Node2D &old_s, Node2D& new_s, double &angle_order, double &velo_order);
}; // end_of_class
}



#endif // 


