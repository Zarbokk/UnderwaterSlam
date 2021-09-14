//
// Created by jurobotics on 13.09.21.
//
#include "ekf.h"

void ekfClass::predictionImu(double xAccel, double yAccel, double zAccel, ros::Time timeStamp) {
    // A-Matrix is zeros, exept for the entries of transition between velocity and position.(there time diff since last prediction
    //update state
    //@TODO has to compensate the g-vector currently just substracts 9.81 from z
    zAccel = zAccel - 9.81;
    double timeDiff = (this->currentPose.timeLastPrediction - timeStamp).toSec();
    if (timeDiff > 0.1) {
        timeDiff = 0.1;
    }
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12);
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(2, 2) = 1;
    A(0, 3) = timeDiff;
    A(1, 4) = timeDiff;
    A(2, 5) = timeDiff;
    A(6, 6) = 1;
    A(7, 7) = 1;
    A(8, 8) = 1;
    A(6, 9) = timeDiff;
    A(7, 10) = timeDiff;
    A(8, 11) = timeDiff;
    Eigen::VectorXd state = this->currentPose.getStatexyzaxayazrpyrvelpvelyvel();
    Eigen::VectorXd inputMatrix = Eigen::VectorXd::Zero(12);
    inputMatrix(3) = xAccel * timeDiff;
    inputMatrix(4) = yAccel * timeDiff;
    inputMatrix(5) = zAccel * timeDiff;
    state = A * state + inputMatrix;
    this->currentPose.applyState(state);
    //update covariance
    this->currentPose.covariance = A * this->currentPose.covariance * A.transpose() + processNoise;
}

void ekfClass::updateSlam(double xPos, double yPos, double yaw, ros::Time timeStamp) {
    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(0) = xPos;
    z(1) = yPos;
    z(8) = yaw;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(0, 0) = 0.1;
    H(1, 1) = 0.1;
    H(8, 8) = 0.1;
    innovation = z - H * this->currentPose.getStatexyzaxayazrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->currentPose.covariance * H.transpose() + this->measurementNoiseDVL;
    Eigen::MatrixXd K = this->currentPose.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->currentPose.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
    this->currentPose.applyState(newState);
    this->currentPose.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->currentPose.covariance;
}

void ekfClass::updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                         ros::Time timeStamp) {
    //TODO angular velocity has to be changed to global rotation. Currently assumed roll pitch very close to 0
    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(6) = roll;
    z(7) = pitch;
    z(9) = xAngularVel;
    z(10) = yAngularVel;
    z(11) = zAngularVel;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(6, 6) = 0.01;
    H(7, 7) = 0.01;
    H(9, 9) = 0.01;
    H(10, 10) = 0.01;
    H(11, 11) = 0.01;
    innovation = z - H * this->currentPose.getStatexyzaxayazrpyrvelpvelyvel();//also y
    //std::cout << innovation << std::endl;
    Eigen::MatrixXd S = H * this->currentPose.covariance * H.transpose() + this->measurementNoiseDVL;
    //std::cout << S << std::endl;
    //std::cout << S.inverse() << std::endl;
    Eigen::MatrixXd K = this->currentPose.covariance * H.transpose() * S.inverse();
    //std::cout << K << std::endl;
    Eigen::VectorXd newState = this->currentPose.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
//    std::cout << "\n" << std::endl;
//    std::cout << newState << std::endl;
    this->currentPose.applyState(newState);
    this->currentPose.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->currentPose.covariance;
}

void ekfClass::updateDVL(double xVel, double yVel, double zVel, ros::Time timeStamp) {
    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(3) = xVel;
    z(4) = yVel;
    z(5) = zVel;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(3, 3) = 1;
    H(4, 4) = 1;
    H(5, 5) = 1;
    innovation = z - H * this->currentPose.getStatexyzaxayazrpyrvelpvelyvel();//also y
//    std::cout << innovation << std::endl;
    Eigen::MatrixXd S = H * this->currentPose.covariance * H.transpose() + this->measurementNoiseDVL;
//    std::cout << S << std::endl;
//    std::cout << S.inverse() << std::endl;
    Eigen::MatrixXd K = this->currentPose.covariance * H.transpose() * S.inverse();
//    std::cout << K << std::endl;
    Eigen::VectorXd newState = this->currentPose.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
//    std::cout << "\n"<< std::endl;
//    std::cout << newState<< std::endl;
    this->currentPose.applyState(newState);
    this->currentPose.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->currentPose.covariance;
}

void ekfClass::updateDepth(double depth, ros::Time timeStamp) {
    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(2) = depth;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(2, 2) = 1;
    innovation = z - H * this->currentPose.getStatexyzaxayazrpyrvelpvelyvel();//also y
//    std::cout << innovation << std::endl;
    Eigen::MatrixXd S = H * this->currentPose.covariance * H.transpose() + this->measurementNoiseDVL;
//    std::cout << S << std::endl;
//    std::cout << S.inverse() << std::endl;
    Eigen::MatrixXd K = this->currentPose.covariance * H.transpose() * S.inverse();
//    std::cout << K << std::endl;
    Eigen::VectorXd newState = this->currentPose.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
//    std::cout << "\n"<< std::endl;
//    std::cout << newState<< std::endl;
    this->currentPose.applyState(newState);
    this->currentPose.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->currentPose.covariance;
}

pose ekfClass::getState(){
    return this->currentPose;
}



