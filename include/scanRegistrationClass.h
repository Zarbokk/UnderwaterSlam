//
// Created by tim on 16.02.21.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/projection_matrix.h>

//#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/common/time.h>
//#include <pcl/console/print.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/fpfh_omp.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include "softDescriptorRegistration.h"
#include "gr/algorithms/match4pcsBase.h"
#include "gr/algorithms/FunctorSuper4pcs.h"
#include "gr/utils/geometry.h"
#include <gr/algorithms/PointPairFilter.h>
#include "image_registration.h"

#include <Eigen/Dense>


#include <ndt_matcher_p2d.h>
#include <ndt_matcher_d2d_2d.h>


#ifndef SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
#define SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H


class scanRegistrationClass {
public:
    scanRegistrationClass(int N = 64, int bwOut = 64 / 2, int bwIn = 64 / 2, int degLim = 64 / 2 - 1)
            : mySofftRegistrationClass(N, bwOut, bwIn, degLim) {
        sizeVoxelData = N;
    }
    ~scanRegistrationClass(){
        mySofftRegistrationClass.~softDescriptorRegistration();
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

    Eigen::Matrix4d sofftRegistration2D(const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1,
                                        const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2,
                                        double &fitnessX, double &fitnessY, double goodGuessAlpha, bool debug);

    Eigen::Matrix4d sofftRegistration2D(const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1,
                                        const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2,
                                        double &fitnessX, double &fitnessY, Eigen::Matrix4d initialGuess,bool useInitialGuess,
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

    Eigen::Matrix4d
    FMSRegistrationOld(double voxelData1Input[], double voxelData2Input[], double cellSize, bool debug = false);


    Eigen::Matrix4d super4PCSRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                          const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                          Eigen::Matrix4d initialGuess, bool useInitialGuess, bool debug = false);

    static Eigen::Matrix4d
    normalDistributionsTransformRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr &Final,
                                             double &fitnessScore,
                                             Eigen::Matrix4d &initialGuessTransformation, double ndt_resolution = 1.0,
                                             double ndt_step_size = 0.1, double transform_epsilon = 0.1);


    Eigen::Matrix4d RANSACRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                       Eigen::Matrix4d initialGuess, bool useInitialGuess, bool debug = false);

    Eigen::Matrix4d ndt_d2d_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan, Eigen::Matrix4d initialGuess,
                               bool useInitialGuess);

    Eigen::Matrix4d ndt_p2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan, Eigen::Matrix4d initialGuess,
                               bool useInitialGuess);

private:
    softDescriptorRegistration mySofftRegistrationClass;

    int sizeVoxelData;

};


#endif //SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
