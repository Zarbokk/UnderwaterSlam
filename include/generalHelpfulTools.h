//
// Created by jurobotics on 15.09.21.
//

#ifndef UNDERWATERSLAM_GENERALHELPFULTOOLS_H
#define UNDERWATERSLAM_GENERALHELPFULTOOLS_H

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include "graphSlamSaveStructure.h"

class generalHelpfulTools {
public:
    static Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat);

    static Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw);

    static Eigen::Matrix4d getTransformationMatrixFromRPY(double roll, double pitch, double yaw);

    static double angleDiff(double first, double second);//first-second
    static Eigen::Matrix4d
    interpolationTwo4DTransformations(Eigen::Matrix4d &transformation1, Eigen::Matrix4d &transformation2,
                                      double &t);// from 1 to two by t[0-1]
    static Eigen::Matrix4d getTransformationMatrix(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation);

    static double weighted_mean(const std::vector<double> &data);

    static void
    smooth_curve(const std::vector<double> &input, std::vector<double> &smoothedOutput, int window_half_width);

    static double
    createVoxelOfGraph(double voxelData[], int indexStart, Eigen::Matrix4d transformationInTheEndOfCalculation,
                       int numberOfPoints, graphSlamSaveStructure &usedGraph, double ignoreDistanceToRobot,
                       double dimensionOfVoxelData);


    static pcl::PointCloud<pcl::PointXYZ> createPCLFromGraphOnlyThreshold(int indexStart,
                                                                          Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                                          graphSlamSaveStructure &usedGraph,
                                                                          double ignoreDistanceToRobo,
                                                                          double thresholdFactorPoint);

    static pcl::PointCloud<pcl::PointXYZ> createPCLFromGraphOneValue(int indexStart,
                                                                     Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                                     graphSlamSaveStructure &usedGraph,
                                                                     double ignoreDistanceToRobo,
                                                                     double thresholdFactorPoint);


};


#endif //UNDERWATERSLAM_GENERALHELPFULTOOLS_H
