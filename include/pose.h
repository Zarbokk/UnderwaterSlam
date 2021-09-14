//
// Created by jurobotics on 13.09.21.
//

#ifndef UNDERWATERSLAM_POSE_H
#define UNDERWATERSLAM_POSE_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <ros/ros.h>

class pose {//pose is in NED
public:
    Eigen::Vector3f position;
    Eigen::Vector3f rotation;
    Eigen::Vector3f velocity;
    Eigen::Vector3f angleVelocity;
    Eigen::Vector3f covarianzPos;
    Eigen::Vector3f covarianzRPY;
    ros::Time timeLastPrediction;
    Eigen::MatrixXd covariance;
    Eigen::VectorXd getStatexyzaxayazrpyrvelpvelyvel() {
        Eigen::VectorXd state = Eigen::VectorXd::Zero(12);
        state(0) = this->position.x();
        state(1) = this->position.y();
        state(2) = this->position.z();

        state(3) = this->velocity.x();
        state(4) = this->velocity.y();
        state(5) = this->velocity.z();

        state(6) = this->rotation.x();
        state(7) = this->rotation.y();
        state(8) = this->rotation.z();

        state(9) = this->angleVelocity.x();
        state(10) = this->angleVelocity.y();
        state(11) = this->angleVelocity.z();

        return state;
    };

    void applyState(Eigen::VectorXd state) {
        this->position.x() = state(0);
        this->position.y() = state(1);
        this->position.z() = state(2);

        this->velocity.x() = state(3);
        this->velocity.y() = state(4);
        this->velocity.z() = state(5);

        this->rotation.x() = state(6);
        this->rotation.y() = state(7);
        this->rotation.z() = state(8);

        this->angleVelocity.x() = state(9);
        this->angleVelocity.y() = state(10);
        this->angleVelocity.z() = state(11);
    };

};


#endif //UNDERWATERSLAM_POSE_H
