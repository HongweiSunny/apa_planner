#ifndef NN_H_
#define NN_H_
// neural network from MATLAB .m file
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

namespace RL_PLANNER_NS
{


class NN
{
private:
    // std::vector<double> probs;
    Matrix<double, 5, 1> inputDataOri;
    Matrix<double, 15, 1> probs;
    Matrix<double, 15, 5> w1;
    Matrix<double,15,1> b1;
    Matrix<double,15,15> w2;
    Matrix<double,15,1> b2;
    Matrix<double,15,15> w3;
    Matrix<double,15,1> b3;
    

    // std::vector<double> b1;
    // std::vector<double> w2;
    // std::vector<double> b2;
    // std::vector<double> w3;
    // std::vector<double> b3;
    Matrix<double, 5, 1> xoffset;
    Matrix<double, 5, 1> gain;
    Matrix<double, 5, 1> ymin;

public:
    void forward_func(Matrix<double,5,1> inputData, Matrix<double, 15, 1> &outputData); // 前向计算
    void forward_func(Matrix<double,5,1> inputData); // 前向计算

    int get_max_prob_rule(Matrix<double, 15, 1> &inputProb);
    int get_max_prob_rule();
    int get_max_prob_norule();


public:
    NN();
    virtual ~NN();

    void normalize_func(Matrix<double,15,1> inputData);
    void sigmoid_func(Matrix<double,15,1> inputData);

    void mapminmax_apply(Matrix<double,5,1> &inputData);
    void tansig_apply(Matrix<double, 15, 1> &inputData); 
    void softmax_apply(Matrix<double, 15, 1> &inputData);

    
};  // end of class

} // end of ns

#endif