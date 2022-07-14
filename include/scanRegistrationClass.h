//
// Created by tim on 16.02.21.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include "softDescriptorRegistration.h"
#include <gr/algorithms/match4pcsBase.h>
#include <gr/algorithms/FunctorSuper4pcs.h>

#ifndef SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
#define SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H


class scanRegistrationClass {
public:
    scanRegistrationClass(int N = 64, int bwOut = 64 / 2, int bwIn = 64 / 2, int degLim = 64 / 2 - 1)
            : myRegistrationClass(N, bwOut, bwIn, degLim) {

    }

    static Eigen::Matrix4d generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                            double &fitnessScore);

    static Eigen::Matrix4d generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                            double &fitnessScore, Eigen::Matrix4d &guess);

    static Eigen::Matrix4d generalizedIcpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr &Final,
                                                      double &fitnessScore,
                                                      Eigen::Matrix4d &initialGuessTransformation);

    Eigen::Matrix4d sofftRegistration(const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1,
                                      const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2,
                                      double &fitnessX, double &fitnessY, double goodGuessAlpha = -100,
                                      bool debug = false);


    static Eigen::Matrix4d icpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                           pcl::PointCloud<pcl::PointXYZ> &Final);

//    Eigen::Matrix4d sofftRegistrationVoxel2D(double voxelData1[],
//                                           double voxelData2[],
//                                           double &fitnessX, double &fitnessY, double goodGuessAlpha = -100,bool debug = false);

    double
    sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[], double goodGuessAlpha,
                                         bool debug = false);

    Eigen::Vector2d
    sofftRegistrationVoxel2DTranslation(double voxelData1Input[], double voxelData2Input[], double &fitnessX,
                                        double &fitnessY, double cellSize,
                                        Eigen::Vector3d initialGuess, bool useInitialGuess, bool debug = false);


private:
    softDescriptorRegistration myRegistrationClass;

};


#endif //SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
