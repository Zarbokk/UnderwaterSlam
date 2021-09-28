//
// Created by jurobotics on 15.09.21.
//

#include "generalHelpfulTools.h"

Eigen::Vector3d generalHelpfulTools::getRollPitchYaw(Eigen::Quaterniond quat){
    tf2::Quaternion tmp(quat.x(),quat.y(),quat.z(),quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r,p,y);
    return returnVector;
}

Eigen::Quaterniond generalHelpfulTools::getQuaternionFromRPY(double roll, double pitch, double yaw){
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return q;
}

