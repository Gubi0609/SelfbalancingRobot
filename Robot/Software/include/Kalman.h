#ifndef KALMAN_H
#define KALMAN_H

#include <BasicLinearAlgrebra.h>

using namespace BLA;

class Kalman {

    public:
        Kalman(Matrix<3,1> initialEstimate, Matrix<3,3> initialCoveriance, Matrix<2,2> measureError, Matrix<3,3> stateError, Matrix<2,3> H, Matrix<3,3> A);
        Matrix<3,1> estimateDegreesAndRate(Matrix<2,1> measuredData);
        ~Kalman();
    
    protected:
        Matrix<3,1> estimate; // x n*1
        Matrix<3,1> predictedEstimate; // x- n*1
        Matrix<3,3> covariance; // P n*n
        Matrix<3,3> predictedCoveriance; // P- n*n
        Matrix<2,2> measureError; // R m*m
        Matrix<3,3> stateError; // Q n*n
        Matrix<2,3> H; // H m*n
        Matrix<3,3> A; // A n*n

};

#endif