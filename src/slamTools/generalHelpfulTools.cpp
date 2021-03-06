//
// Created by jurobotics on 15.09.21.
//

#include "generalHelpfulTools.h"

Eigen::Vector3d generalHelpfulTools::getRollPitchYaw(Eigen::Quaterniond quat) {
    tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r, p, y);
    return returnVector;
}

Eigen::Quaterniond generalHelpfulTools::getQuaternionFromRPY(double roll, double pitch, double yaw) {
//        tf2::Matrix3x3 m;
//        m.setRPY(roll,pitch,yaw);
//        Eigen::Matrix3d m2;
    tf2::Quaternion qtf2;
    qtf2.setRPY(roll, pitch, yaw);
    Eigen::Quaterniond q;
    q.x() = qtf2.x();
    q.y() = qtf2.y();
    q.z() = qtf2.z();
    q.w() = qtf2.w();

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return q;
}

double generalHelpfulTools::angleDiff(double first, double second) {//first-second
    return atan2(sin(first - second), cos(first - second));
}

double generalHelpfulTools::weighted_mean(const std::vector<double> & data)
{
    double mean=0.0;

    for (int i=0 ; i<data.size() ; i++){
        mean+=data[i];
    }
    return mean/double(data.size()) ;
}

void generalHelpfulTools::smooth_curve(const std::vector<double> & input, std::vector<double> & smoothedOutput, int window_half_width){

    int window_width=2*window_half_width+1;

    int size=input.size();

    std::vector<double> sample(window_width);

    for (int i=0 ; i < size ; i++){

        for ( int j=0 ; j < window_width ; j++ ){

            int shifted_index=i+j-window_half_width;
            if (shifted_index<0) shifted_index=0;
            if (shifted_index>size-1) shifted_index=size-1;
            sample[j]=input[shifted_index];

        }

        smoothedOutput.push_back(generalHelpfulTools::weighted_mean(sample));

    }

}


