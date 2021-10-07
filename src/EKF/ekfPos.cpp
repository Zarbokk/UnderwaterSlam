//
// Created by jurobotics on 13.09.21.
//
#include "ekf.h"

void ekfClass::predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation,
                             ros::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();

    // A-Matrix is zeros, except for the entries of transition between velocity and position.(there time diff since last prediction
    // update state


    Eigen::Vector3d currentEuler = generalHelpfulTools::getRollPitchYaw(currentRotation);//roll pitch yaw

    Eigen::AngleAxisd rollAngle(currentEuler.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(currentEuler.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());//remove the yaw rotation

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    Eigen::Vector3d bodyAcceleration(xAccel, yAccel, zAccel);
    bodyAcceleration = q.inverse() * bodyAcceleration;
    bodyAcceleration(2) = bodyAcceleration(2) - 9.81;
    bodyAcceleration = q * bodyAcceleration;
    //changing body system from z up to z down and y from y left to y right
    bodyAcceleration(1) = -bodyAcceleration(1);
    bodyAcceleration(2) = -bodyAcceleration(2);
    // bodyAcceleration has to be changed to correct rotation(body acceleration)
    Eigen::Vector3d localAcceleration = this->getRotationVector() * bodyAcceleration;
//    std::cout << "local acceleration in world system: " << std::endl;
//    std::cout << this->stateOfEKF.rotation.x()<<" "<<this->stateOfEKF.rotation.y()<<" "<<this->stateOfEKF.rotation.z() << std::endl;
    double timeDiff = (timeStamp-this->stateOfEKF.timeLastPrediction).toSec();
    if (timeDiff > 0.1|| timeDiff<0) {
        timeDiff = 0.1;
    }
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(12, 12);

    A(0, 3) = timeDiff;
    A(1, 4) = timeDiff;
    A(2, 5) = timeDiff;
    A(6, 9) = timeDiff;
    A(7, 10) = timeDiff;
    A(8, 11) = timeDiff;
    Eigen::VectorXd state = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();
    Eigen::VectorXd inputMatrix = Eigen::VectorXd::Zero(12);
    inputMatrix(3) = bodyAcceleration(0) * timeDiff;
    inputMatrix(4) = bodyAcceleration(1) * timeDiff;
    inputMatrix(5) = bodyAcceleration(2) * timeDiff;
    state = A * state + inputMatrix;
    if(state(8)>M_PI){
        state(8)=state(8)-2*M_PI;
    }
    if(state(8)<-M_PI){
        state(8)=state(8)+2*M_PI;
    }

    this->stateOfEKF.applyState(state);
    //update covariance
    this->stateOfEKF.covariance = A * this->stateOfEKF.covariance * A.transpose() + processNoise;
    this->stateOfEKF.timeLastPrediction = timeStamp;

    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateAfterUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();
    Eigen::VectorXd differenceStateAfterUpdate = currentStateAfterUpdate-currentStateBeforeUpdate;
    Eigen::Vector3d positionDifference(differenceStateAfterUpdate(0),differenceStateAfterUpdate(1),0);
    Eigen::AngleAxisd rotation_vector(differenceStateAfterUpdate(8), Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond yawRotation(rotation_vector);
    Eigen::Vector3d covariancePos(0, 0, 0);
    edge currentEdge(0,0,positionDifference,yawRotation,covariancePos,0,3,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
    currentEdge.setTimeStamp(timeStamp.toSec());
    this->lastPositionDifferences.push_back(currentEdge);

}

void ekfClass::updateSlam(double xPos, double yPos, double yaw, ros::Time timeStamp) {
    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(0) = xPos;
    z(1) = yPos;
    z(8) = yaw;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(8, 8) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseSlam;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
}

void ekfClass::updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                         Eigen::Quaterniond currentRotation,
                         ros::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();



    Eigen::VectorXd innovation;
    //change from system to body system
    Eigen::Vector3d velocityBodyAngular(xAngularVel, -yAngularVel, -zAngularVel);
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalAngular = this->getRotationVector() * velocityBodyAngular;

    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(6) = roll;
    z(7) = pitch;
    z(9) = velocityLocalAngular(0);
    z(10) = velocityLocalAngular(1);
    z(11) = velocityLocalAngular(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(6, 6) = 1;
    H(7, 7) = 1;
    H(9, 9) = 1;
    H(10, 10) = 1;
    H(11, 11) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();//also called y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementImuVelocity;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;

    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateAfterUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();
    Eigen::VectorXd differenceStateAfterUpdate = currentStateAfterUpdate-currentStateBeforeUpdate;
    Eigen::Vector3d positionDifference(differenceStateAfterUpdate(0),differenceStateAfterUpdate(1),0);
    Eigen::AngleAxisd rotation_vector(differenceStateAfterUpdate(8), Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond yawRotation(rotation_vector);
    Eigen::Vector3d covariancePos(0, 0, 0);
    edge currentEdge(0,0,positionDifference,yawRotation,covariancePos,0,3,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
    currentEdge.setTimeStamp(timeStamp.toSec());
    this->lastPositionDifferences.push_back(currentEdge);
}

void ekfClass::updateDVL(double xVel, double yVel, double zVel, ros::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();


    Eigen::Vector3d velocityBodyLinear(xVel, yVel, zVel);
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalLinear = this->getRotationVector() * velocityBodyLinear;

    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(3) = velocityLocalLinear(0);
    z(4) = velocityLocalLinear(1);
    z(5) = velocityLocalLinear(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(3, 3) = 1;
    H(4, 4) = 1;
    H(5, 5) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();//also y
//    std::cout << innovation << std::endl;
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDVL;
//    std::cout << S << std::endl;
//    std::cout << S.inverse() << std::endl;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
//    std::cout << K << std::endl;
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
//    std::cout << "\n"<< std::endl;
//    std::cout << newState<< std::endl;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;

    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateAfterUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();
    Eigen::VectorXd differenceStateAfterUpdate = currentStateAfterUpdate-currentStateBeforeUpdate;
    Eigen::Vector3d positionDifference(differenceStateAfterUpdate(0),differenceStateAfterUpdate(1),0);
    Eigen::AngleAxisd rotation_vector(differenceStateAfterUpdate(8), Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond yawRotation(rotation_vector);
    Eigen::Vector3d covariancePos(0, 0, 0);
    edge currentEdge(0,0,positionDifference,yawRotation,covariancePos,0,3,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
    currentEdge.setTimeStamp(timeStamp.toSec());
    this->lastPositionDifferences.push_back(currentEdge);

}

void ekfClass::updateDepth(double depth, ros::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();


    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(2) = depth;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(2, 2) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDVL;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;


    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateAfterUpdate = this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel();
    Eigen::VectorXd differenceStateAfterUpdate = currentStateAfterUpdate-currentStateBeforeUpdate;
    Eigen::Vector3d positionDifference(differenceStateAfterUpdate(0),differenceStateAfterUpdate(1),0);
    Eigen::AngleAxisd rotation_vector(differenceStateAfterUpdate(8), Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond yawRotation(rotation_vector);
    Eigen::Vector3d covariancePos(0, 0, 0);
    edge currentEdge(0,0,positionDifference,yawRotation,covariancePos,0,3,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
    currentEdge.setTimeStamp(timeStamp.toSec());
    this->lastPositionDifferences.push_back(currentEdge);

}

pose ekfClass::getState() {
    return this->stateOfEKF;
}

Eigen::Quaterniond ekfClass::getRotationVector() {
    double rotationX = this->stateOfEKF.rotation.x();
    double rotationY = this->stateOfEKF.rotation.y();
    double rotationZ = this->stateOfEKF.rotation.z();

    Eigen::AngleAxisd rollAngle(rotationX, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rotationY, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rotationZ, Eigen::Vector3d::UnitZ());//remove the yaw rotation

    Eigen::Quaterniond rotDiff = yawAngle * pitchAngle * rollAngle;
    return rotDiff;
}

ekfClass ekfClass::copyEKF() {
    ekfClass returnObject = ekfClass(this->lastUpdateTime);
    returnObject.stateOfEKF.applyState(this->stateOfEKF.getStatexyzaxayazrpyrvelpvelyvel());
    returnObject.lastUpdateTime=this->lastUpdateTime;
    returnObject.stateOfEKF.covariance=this->stateOfEKF.covariance;
    returnObject.stateOfEKF.timeLastPrediction=this->stateOfEKF.timeLastPrediction;
    returnObject.lastPositionDifferences = this->lastPositionDifferences;
    return returnObject;
}

std::deque<edge> ekfClass::getLastPoses(){
    return this->lastPositionDifferences;
}

void ekfClass::removePastPoses(){
    this->lastPositionDifferences.clear();
}




