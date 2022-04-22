//
// Created by jurobotics on 13.09.21.
//
#include "ekfDVL.h"

void ekfClassDVL::predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation,
                                ros::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    // A-Matrix is zeros, except for the entries of transition between velocity and position.(there time diff since last prediction
    // update state
//    std::cout << "xPos: " << currentStateBeforeUpdate[0] << std::endl;
//    std::cout << "X-Vel: " <<  currentStateBeforeUpdate[0] << std::endl;
//    std::cout << "########## State: " << std::endl;
//    std::cout << currentStateBeforeUpdate << std::endl;
    Eigen::Vector3d bodyAcceleration(xAccel, yAccel, zAccel);
    // bodyAcceleration has to be changed to correct rotation(body acceleration)
    Eigen::Vector3d localAcceleration = this->getRotationVector() *
                                        bodyAcceleration;// @TODO should be checked if still correct something has to change
    localAcceleration(2) = localAcceleration(2) + 9.81;

//   std::cout << "local acceleration in world system: "<< " x: " << localAcceleration(0) << " y: " << localAcceleration(1) <<std::endl;
//   std::cout << this->stateOfEKF.rotation.x()<<" "<<this->stateOfEKF.rotation.y()<<" "<<this->stateOfEKF.rotation.z() << std::endl;
    double timeDiff = (timeStamp - this->stateOfEKF.timeLastPrediction).toSec();

    if (timeDiff > 0.4 || timeDiff < 0) {
        timeDiff = 0.4;
    }
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(12, 12);
    //state Transition Matrix
    A(0, 3) = timeDiff;
    A(1, 4) = timeDiff;
    A(2, 5) = timeDiff;
    A(6, 9) = timeDiff;
    A(7, 10) = timeDiff;
    A(8, 11) = timeDiff;
    Eigen::VectorXd state = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();
    Eigen::VectorXd inputMatrix = Eigen::VectorXd::Zero(12);
    inputMatrix(3) = localAcceleration(0) * timeDiff;
    inputMatrix(4) = localAcceleration(1) * timeDiff;
    inputMatrix(5) = localAcceleration(2) * timeDiff;//this is z acceleration localAcceleration(2) * timeDiff;
    state = A * state + inputMatrix;
    if (state(8) > M_PI) {
        state(8) = state(8) - 2 * M_PI;
    }
    if (state(8) < -M_PI) {
        state(8) = state(8) + 2 * M_PI;
    }

    this->stateOfEKF.applyState(state);
    //update covariance
    this->stateOfEKF.covariance = A * this->stateOfEKF.covariance * A.transpose() + processNoise;
    this->stateOfEKF.timeLastPrediction = timeStamp;

}

void ekfClassDVL::simplePrediction(ros::Time timeStamp){
    //for saving the current EKF pose difference in
    double timeDiff = (timeStamp - this->stateOfEKF.timeLastPrediction).toSec();

    if (timeDiff > 0.4 || timeDiff < 0) {
        timeDiff = 0.4;
    }
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(12, 12);
    //state Transition Matrix
    A(0, 3) = timeDiff;
    A(1, 4) = timeDiff;
    A(2, 5) = timeDiff;
    A(6, 9) = timeDiff;
    A(7, 10) = timeDiff;
    A(8, 11) = timeDiff;
    Eigen::VectorXd state = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    state = A * state;
    if (state(8) > M_PI) {
        state(8) = state(8) - 2 * M_PI;
    }
    if (state(8) < -M_PI) {
        state(8) = state(8) + 2 * M_PI;
    }

    this->stateOfEKF.applyState(state);
    //update covariance
    this->stateOfEKF.covariance = A * this->stateOfEKF.covariance * A.transpose() + processNoise;
    this->stateOfEKF.timeLastPrediction = timeStamp;
}

void ekfClassDVL::updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                            Eigen::Quaterniond currentRotation,
                            ros::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    Eigen::VectorXd innovation;
    //change from system to body system
    Eigen::Vector3d velocityBodyAngular(xAngularVel, yAngularVel, zAngularVel);
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalAngular = this->getRotationVectorWithoutYaw() * velocityBodyAngular;

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
    if (roll > 0 && currentStateBeforeUpdate[6] < 0) {
        currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] + M_PI * 2;
    }
    if (roll < 0 && currentStateBeforeUpdate[6] > 0) {
        currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] - M_PI * 2;
    }
    innovation = this->innovationStateDiff(z, H, currentStateBeforeUpdate);//also called y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementImuVelocity;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = currentStateBeforeUpdate + K * innovation;
//    std::cout<< K * innovation <<endl;
//    std::cout << "state Roll: " << this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel()[6]*180/M_PI << std::endl;
//    std::cout << "IMU Roll: " << roll*180/M_PI << std::endl;
//    std::cout << "state Pitch: " << this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel()[7]*180/M_PI << std::endl;
//    std::cout << "IMU Pitch: " << pitch*180/M_PI << std::endl;

    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;

}

void
ekfClassDVL::updateDVL(double xVel, double yVel, double zVel, Eigen::Quaterniond rotationOfDVL, ros::Time timeStamp) {
    //for saving the current EKF pose difference in

    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();


    Eigen::Vector3d velocityBodyLinear(xVel, yVel, zVel);
    velocityBodyLinear = rotationOfDVL * velocityBodyLinear;
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalLinear = this->getRotationVector() * velocityBodyLinear;

    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(3) = velocityLocalLinear(0);
    z(4) = velocityLocalLinear(1);
    z(5) = velocityLocalLinear(2);// removed because z is not important. velocityLocalLinear(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(3, 3) = 1;
    H(4, 4) = 1;
    H(5, 5) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
//    std::cout << "Inno: " <<std::endl;
//    std::cout << innovation << std::endl;
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDVL;
//    std::cout << "S: " <<std::endl;
//    std::cout << S << std::endl;
//    std::cout << "S Inverse: " <<std::endl;
//    std::cout << S.inverse() << std::endl;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
//    std::cout << "Covariance: " <<std::endl;
//    std::cout << this->stateOfEKF.covariance << std::endl;
//    std::cout << "K: " <<std::endl;
//    std::cout << K << std::endl;
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
//    std::cout << "\n"<< std::endl;
//    std::cout << newState<< std::endl;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
//    std::cout << "DVL UPDATE: \n" << velocityLocalLinear <<std::endl;
}

void
ekfClassDVL::updateHeight(double depth,ros::Time timeStamp) {
    if(isnan(depth)){
        depth=0;
    }
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();



    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(2) = depth;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(2, 2) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDepth;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
}

void
ekfClassDVL::updateHeading(double yawRotation,ros::Time timeStamp) {
    if(isnan(yawRotation)){
        return;
    }
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    std::cout << "yaw update: " << yawRotation<< std::endl;
    std::cout << "before Update: "<<this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel()[8] << std::endl;

    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(8) = yawRotation;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(8, 8) = 1;
    innovation = this->innovationStateDiff(z, H, currentStateBeforeUpdate);//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDepth;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;

    std::cout << "after Update: "<<this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel()[8] << std::endl;
}



pose ekfClassDVL::getState() {
    return this->stateOfEKF;
}

Eigen::Quaterniond ekfClassDVL::getRotationVector() {
    double rotationX = this->stateOfEKF.rotation.x();
    double rotationY = this->stateOfEKF.rotation.y();
    double rotationZ = this->stateOfEKF.rotation.z();

//    Eigen::AngleAxisd rollAngle(, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd pitchAngle(rotationY, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd yawAngle(rotationZ, Eigen::Vector3d::UnitZ());//remove the yaw rotation

    Eigen::Quaterniond rotDiff = generalHelpfulTools::getQuaternionFromRPY(rotationX, rotationY, rotationZ);
    return rotDiff;
}

Eigen::Quaterniond ekfClassDVL::getRotationVectorWithoutYaw() {
    double rotationX = this->stateOfEKF.rotation.x();
    double rotationY = this->stateOfEKF.rotation.y();

//    Eigen::AngleAxisd rollAngle(rotationX, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd pitchAngle(rotationY, Eigen::Vector3d::UnitY());

    Eigen::Quaterniond rotDiff = generalHelpfulTools::getQuaternionFromRPY(rotationX, rotationY, 0);
    return rotDiff;
}

Eigen::VectorXd ekfClassDVL::innovationStateDiff(Eigen::VectorXd z, Eigen::MatrixXd H,
                                                 Eigen::VectorXd currentStateBeforeUpdate) {//xyz vxvyvz rpy rvel pvel yvel
    Eigen::VectorXd innovation;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
    innovation(6) = generalHelpfulTools::angleDiff(z(6), H(6, 6) * currentStateBeforeUpdate(6));
    innovation(7) = generalHelpfulTools::angleDiff(z(7), H(7, 7) * currentStateBeforeUpdate(7));
    innovation(8) = generalHelpfulTools::angleDiff(z(8), H(8, 8) * currentStateBeforeUpdate(8));

    return innovation;
}

void ekfClassDVL::resetToPos(double x, double y, double yaw, bool resetCovariance){
    //@TODO make sure yaw is possible additionally rotate the current velocity to the correct value of last yaw
    this->stateOfEKF.rotation.z() = yaw;
    this->stateOfEKF.position.x() = x;
    this->stateOfEKF.position.y() = y;
    //xyzvxvyvzrpyrvelpvelyvel
    if(resetCovariance){
        this->stateOfEKF.covariance(0,0) = 0;//x
        this->stateOfEKF.covariance(1,1) = 0;//y
        this->stateOfEKF.covariance(8,8) = 0;//yaw
    }

}

void ekfClassDVL::setProcessNoise(double xNoise, double yNoise, double zNoise, double vxNoise, double vyNoise, double vzNoise,
                                  double rNoise, double pNoise, double yawNoise, double vrNoise, double vpNoise, double vyawNoise){
    this->processNoise(0, 0) = xNoise;//x
    this->processNoise(1, 1) = yNoise;//y
    this->processNoise(2, 2) = zNoise;//z
    this->processNoise(3, 3) = vxNoise;//vx
    this->processNoise(4, 4) = vyNoise;//vy
    this->processNoise(5, 5) = vzNoise;//vz
    this->processNoise(6, 6) = rNoise;//r
    this->processNoise(7, 7) = pNoise;//p
    this->processNoise(8, 8) = yawNoise;//y
    this->processNoise(9, 9) = vrNoise;//vr
    this->processNoise(10, 10) = vpNoise;//vp
    this->processNoise(11, 11) = vyawNoise;//vy
}

void ekfClassDVL::setMeasurementNoiseDVL(double vxNoise, double vyNoise, double vzNoise){
    this->measurementNoiseDVL(3, 3) = vxNoise;//vx
    this->measurementNoiseDVL(4, 4) = vyNoise;//vy
    this->measurementNoiseDVL(5, 5) = vzNoise;//vz
}

void ekfClassDVL::setMeasurementNoiseDepth(double zNoise){
    this->measurementNoiseDepth(2, 2) = zNoise;//z
}

void ekfClassDVL::setMeasurementNoiseIMUVel(double rNoise, double pNoise,double vrNoise, double vpNoise, double vyNoise){
    this->measurementImuVelocity(6, 6) = rNoise;//r
    this->measurementImuVelocity(7, 7) = pNoise;//p
    this->measurementImuVelocity(9, 9) = vrNoise;//vr
    this->measurementImuVelocity(10, 10) = vpNoise;//vp
    this->measurementImuVelocity(11, 11) = vyNoise;//vy
}


