#ifndef UNDERWATERSLAM_EKF_H
#define UNDERWATERSLAM_EKF_H


#include <deque>
#include "generalHelpfulTools.h"
#include "pose.h"
//asynchronous EKF with horizon horizont correction



class ekfClass {
public:
    ekfClass(ros::Time timeRos) {
        this->stateOfEKF.position = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.rotation = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.velocity = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.angleVelocity = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.timeLastPrediction = timeRos;

        this->processNoise = 10 * Eigen::MatrixXd::Identity(12, 12);
        this->processNoise(0, 0) = 0.002;//x
        this->processNoise(1, 1) = 0.002;//y
        this->processNoise(2, 2) = 0.0001;//z
        this->processNoise(3, 3) = 0.0001;//vx
        this->processNoise(4, 4) = 0.0001;//vy
        this->processNoise(5, 5) = 0.0001;//vz
        this->processNoise(6, 6) = 0.0001;//r
        this->processNoise(7, 7) = 0.0001;//p
        this->processNoise(8, 8) = 0.001;//y
        this->processNoise(9, 9) = 0.0001;//vr
        this->processNoise(10, 10) = 0.0001;//vp
        this->processNoise(11, 11) = 0.0001;//vy

        this->measurementNoiseDepth = 100 * Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseDepth(2, 2) = 0.01;

        this->measurementNoiseDVL = 100 * Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseDVL(3, 3) = 0.01;
        this->measurementNoiseDVL(4, 4) = 0.01;
        this->measurementNoiseDVL(5, 5) = 0.01;

        this->measurementImuVelocity = 100 * Eigen::MatrixXd::Identity(12, 12);
        this->measurementImuVelocity(9, 9) = 0.01;
        this->measurementImuVelocity(10, 10) = 0.01;
        this->measurementImuVelocity(11, 11) = 0.01;

        this->stateOfEKF.covariance = processNoise;

        this->recentPoses.push_back(stateOfEKF);
    }
    ekfClass copyEKF();
    void
    predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation, ros::Time timeStamp);

    void updateDepth(double depth, ros::Time timeStamp);

    void updateDVL(double xVel, double yVel, double zVel, ros::Time timeStamp);

    void updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                   Eigen::Quaterniond currentRotation, ros::Time timeStamp);

    pose getState();

    Eigen::Quaterniond getRotationVector();

    void updateSlam(double xPos, double yPos, double yaw, ros::Time timeStamp);

private:
    pose stateOfEKF;
    std::deque<pose> recentPoses;
    Eigen::MatrixXd processNoise, measurementNoiseDepth, measurementNoiseDVL, measurementImuVelocity;
    ros::Time lastUpdateTime;
};


#endif //UNDERWATERSLAM_EKF_H
