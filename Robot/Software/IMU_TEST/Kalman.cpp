#include "./include/Kalman.h"

Kalman::Kalman(Matrix<3,1> initialEstimate, Matrix<3,3> initialCoveriance, Matrix<2,2> measureError, Matrix<3,3> stateError, Matrix<2,3> H, Matrix<3,3> A) {
    
    this->estimate = initialEstimate;
    this->covariance = initialCoveriance;
    this->measureError = measureError;
    this->stateError = stateError;
    this->H = H;
    this->A = A;

}

Matrix<3,1> Kalman::estimateDegreesAndRate(Matrix<2,1> measuredData) {

    predictedEstimate = A*estimate;
    predictedCoveriance = A*covariance*~A+stateError;

    Matrix<3,2> kalmanGain = predictedCoveriance*~H*Inverse(H*predictedCoveriance*~H+measureError);

    estimate = predictedEstimate + kalmanGain*(measuredData - H*predictedEstimate);

    covariance = predictedCoveriance - kalmanGain*H*predictedCoveriance;

    return estimate;

}

Kalman::~Kalman() {}