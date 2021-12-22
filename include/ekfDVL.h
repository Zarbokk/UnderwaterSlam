//
// Created by auvatjacobs on 30.11.21.
//

#ifndef UNDERWATERSLAM_EKFDVL_H
#define UNDERWATERSLAM_EKFDVL_H

#include <deque>
#include "generalHelpfulTools.h"
#include "pose.h"
#include "slamToolsRos.h"
//asynchronous EKF with reset of POS correction


class ekfClassDVL {
public:
    ekfClassDVL(ros::Time timeRos) {
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
        this->processNoise(6, 6) = 0.1;//r
        this->processNoise(7, 7) = 0.1;//p
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
        this->measurementImuVelocity(6, 6) = 0.001;
        this->measurementImuVelocity(7, 7) = 0.001;
        this->measurementImuVelocity(9, 9) = 0.01;
        this->measurementImuVelocity(10, 10) = 0.01;
        this->measurementImuVelocity(11, 11) = 0.01;

        this->measurementNoiseSlam = 100 * Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseSlam(0, 0) = 0.01;
        this->measurementNoiseSlam(1, 1) = 0.01;
        this->measurementNoiseSlam(8, 8) = 0.01;

        this->stateOfEKF.covariance = processNoise;

        this->recentPoses.push_back(stateOfEKF);
        this->lastPositionDifferences.clear();
    }

    void
    predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation, ros::Time timeStamp);

    void updateDVL(double xVel, double yVel, double zVel, Eigen::Quaterniond rotationOfDVL, ros::Time timeStamp);

    void updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                   Eigen::Quaterniond currentRotation, ros::Time timeStamp);

    void updateHeight(double depth,ros::Time timeStamp);

    pose getState();

    Eigen::Quaterniond getRotationVector();

    Eigen::Quaterniond getRotationVectorWithoutYaw();

    void resetToPos(double x, double y, double yaw, bool resetCovariance);

    Eigen::VectorXd innovationStateDiff(Eigen::VectorXd z, Eigen::MatrixXd H, Eigen::VectorXd currentStateBeforeUpdate);

private:
    pose stateOfEKF;
    std::deque<edge> lastPositionDifferences;
    std::deque<pose> recentPoses;
    Eigen::MatrixXd processNoise, measurementNoiseDepth, measurementNoiseDVL, measurementImuVelocity, measurementNoiseSlam;
    ros::Time lastUpdateTime;
};


#endif //UNDERWATERSLAM_EKFDVL_H
