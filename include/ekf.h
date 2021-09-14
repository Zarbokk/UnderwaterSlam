#ifndef UNDERWATERSLAM_EKF_H
#define UNDERWATERSLAM_EKF_H


#include <deque>

#include "pose.h"
//asynchronous EKF with horizon horizont correction



class ekfClass {
public:
    ekfClass(ros::Time timeRos) {
        this->currentPose.position = Eigen::Vector3f(0, 0, 0);
        this->currentPose.rotation = Eigen::Vector3f(0, 0, 0);
        this->currentPose.velocity = Eigen::Vector3f(0, 0, 0);
        this->currentPose.angleVelocity = Eigen::Vector3f(0, 0, 0);
        this->currentPose.covarianzPos = Eigen::Vector3f(0, 0, 0);
        this->currentPose.covarianzRPY = Eigen::Vector3f(0, 0, 0);
        this->currentPose.timeLastPrediction = timeRos;

        this->processNoise = 1000*Eigen::MatrixXd::Identity(12,12);
        this->processNoise(0,0)=0.2;//x
        this->processNoise(1,1)=0.2;//y
        this->processNoise(2,2)=0.2;//z
        this->processNoise(3,3)=0.1;//vx
        this->processNoise(4,4)=0.1;//vy
        this->processNoise(5,5)=0.1;//vz
        this->processNoise(6,6)=0.1;//r
        this->processNoise(7,7)=0.1;//p
        this->processNoise(8,8)=0.1;//y
        this->processNoise(9,9)=0.1;//vr
        this->processNoise(10,10)=0.1;//vp
        this->processNoise(11,11)=0.1;//vy

        this->measurementNoiseDepth = 1000*Eigen::MatrixXd::Identity(12,12);
        this->measurementNoiseDepth(2,2) = 0.1;

        this->measurementNoiseDVL = 1000*Eigen::MatrixXd::Identity(12,12);
        this->measurementNoiseDVL(3,3) = 0.1;
        this->measurementNoiseDVL(4,4) = 0.1;
        this->measurementNoiseDVL(5,5) = 0.1;

        this->currentPose.covariance = processNoise;

        this->recentPoses.push_back(currentPose);
    }

    void predictionImu(double xAccel, double yAccel, double zAccel, ros::Time timeStamp);

    void updateDepth(double depth, ros::Time timeStamp);

    void updateDVL(double xVel, double yVel, double zVel, ros::Time timeStamp);

    void updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                   ros::Time timeStamp);
    pose getState();
    void updateSlam(double xPos, double yPos, double yaw, ros::Time timeStamp);

private:
    pose currentPose;
    std::deque<pose> recentPoses;
    Eigen::MatrixXd processNoise,measurementNoiseDepth,measurementNoiseDVL;
    ros::Time lastUpdateTime;
};




#endif //UNDERWATERSLAM_EKF_H
