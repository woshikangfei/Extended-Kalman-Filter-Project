#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;


    assert(estimations.size()>0);

    //  * the estimation vector size should equal ground truth vector size
    assert(estimations.size() == ground_truth.size());


    for(int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i]-ground_truth[i];
        squared_residuals = residual.array()*residual.array();
        rmse += squared_residuals;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE
    double x1 = px*px + py*py;
    double x2 = sqrt(x1);
    double x3 =x1*x2
    //check division by zero
    if (c1<0.0001){
        std::cout << "Division by zero ERROR!" << std::endl;
        return Hj;
    }
    //compute the Jacobian matrix
    double px_x2=px/x2;
    double py_x2 = py/x2;
    double py_x1=-py/x1;
    double px_x1 = px/x1;
    double temp1 =py*(vx*py - vy*px)/x3;
    double temp2 = px*(px*vy - py*vx)/x3;
    Hj << px_x2, py_x2, 0, 0,
            py_x1, px_x1, 0, 0,
            temp1, temp2 , px_x2, py_x2;
   

    return Hj;
}
