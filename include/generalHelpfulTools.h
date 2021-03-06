//
// Created by jurobotics on 15.09.21.
//

#ifndef UNDERWATERSLAM_GENERALHELPFULTOOLS_H
#define UNDERWATERSLAM_GENERALHELPFULTOOLS_H
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
class generalHelpfulTools {
public:
    static Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat);
    static Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw);
    static double angleDiff(double first, double second);//first-second
    static double weighted_mean(const std::vector<double> & data);
    static void smooth_curve(const std::vector<double> & input, std::vector<double> & smoothedOutput, int window_half_width);
};


#endif //UNDERWATERSLAM_GENERALHELPFULTOOLS_H
