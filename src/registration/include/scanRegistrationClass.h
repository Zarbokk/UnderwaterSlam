//
// Created by tim on 16.02.21.
//
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "fs2d/srv/request_one_potential_solution.hpp"
#include "fs2d/srv/request_list_potential_solution.hpp"
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include "opencv4/opencv2/calib3d.hpp"
#include "cv_bridge/cv_bridge.h"
#include "generalHelpfulTools.h"
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/projection_matrix.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/visualization/pcl_visualizer.h>

//#include "softDescriptorRegistration.h"
//#include "gr/algorithms/match4pcsBase.h"
//#include "gr/algorithms/FunctorSuper4pcs.h"
//#include "gr/utils/geometry.h"
//#include <gr/algorithms/PointPairFilter.h>


#include <Eigen/Dense>



#include <ndt_matcher_p2d.h>
#include <ndt_matcher_d2d_2d.h>
//#include "perception_oru/ndt_matcher_d2d_2d.h"
//#include "include/ndt_matcher_d2d_2d.h"
//#include "perception_oru/include/ndt_matcher_p2d.h"
//#include "perception_oru/include/ndt_matcher_d2d_2d.h"

//#include "percep"
//#include ""



#include <image_registration.h>
#include "opencv4/opencv2/features2d.hpp"
#include "opencv4/opencv2/xfeatures2d.hpp"
#include "opencv4/opencv2/xfeatures2d/nonfree.hpp"



#include <gmm_registration/front_end/GaussianMixturesModel.h>
#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/method/DistributionToDistribution2D.h>
#include <gmm_registration/solver/CholeskyLineSearchNewtonMethod.h>
#include <gmm_registration/method/PointsToDistribution2D.h>





struct rotationPeakSLAM {
    double angle;
    double peakCorrelation;
    double covariance;
};

struct translationPeakSLAM {
    Eigen::Vector2d translationSI;
    Eigen::Vector2i translationVoxel;
    double peakHeight;
    Eigen::Matrix2d covariance;
};

struct transformationPeakSLAM {
    std::vector<translationPeakSLAM> potentialTranslations;
    rotationPeakSLAM potentialRotation;
};



#ifndef SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
#define SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H


class scanRegistrationClass : public rclcpp::Node {
public:
    scanRegistrationClass(int N = 64, int bwOut = 64 / 2, int bwIn = 64 / 2, int degLim = 64 / 2 - 1,std::string nameOfNode = "registrationnode" )
            : Node(nameOfNode) {
        sizeVoxelData = N;
        icpMutex = new std::mutex();
        ndtd2dMutex = new std::mutex();
        ndtp2dMutex = new std::mutex();
        fourierMellinMutex = new std::mutex();
        oursMutex = new std::mutex();
        featureBasedMutex = new std::mutex();
        gmmd2dMutex = new std::mutex();
        gmmp2dMutex = new std::mutex();


        this->onePotentialClient = this->create_client<fs2d::srv::RequestOnePotentialSolution>(
                "fs2d/registration/one_solution");
        this->listPotentialClient = this->create_client<fs2d::srv::RequestListPotentialSolution>(
                "fs2d/registration/all_solutions");

    }

//    ~scanRegistrationClass() {
//        mySofftRegistrationClass.~softDescriptorRegistration();
//    }

//    Eigen::Matrix4d generalizedIcpRegistrationSimple(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
//                                                            pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
//                                                            double &fitnessScore);
//
    Eigen::Matrix4d generalizedIcpRegistrationSimple(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                            pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                            double &fitnessScore, Eigen::Matrix4d &guess);
//
    Eigen::Matrix4d generalizedIcpRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                      pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                      pcl::PointCloud<pcl::PointXYZ> &Final,
                                                      double &fitnessScore,
                                                      Eigen::Matrix4d &initialGuessTransformation);
//
//    Eigen::Matrix4d sofftRegistration2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData1,
//                                        pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData2,
//                                        double &fitnessX, double &fitnessY, double goodGuessAlpha, bool debug);
//
//    Eigen::Matrix4d sofftRegistration2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData1,
//                                        pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData2,
//                                        double &fitnessX, double &fitnessY, Eigen::Matrix4d initialGuess,
//                                        bool useInitialGuess,
//                                        bool debug = false);
//
//    Eigen::Matrix4d icpRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
//                                           pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
//                                           pcl::PointCloud<pcl::PointXYZ> &Final);

//    Eigen::Matrix4d sofftRegistrationVoxel2D(double voxelData1[],
//                                           double voxelData2[],
//                                           double &fitnessX, double &fitnessY, double goodGuessAlpha = -100,bool debug = false);

//    double
//    sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[], double goodGuessAlpha,double &covariance,
//                                         bool debug = false);

//    Eigen::Vector2d sofftRegistrationVoxel2DTranslation(double voxelData1Input[],
//                                                        double voxelData2Input[],
//                                                        double &fitnessX, double &fitnessY, double cellSize,
//                                                        Eigen::Vector3d initialGuess, bool useInitialGuess,
//                                                        double &heightMaximumPeak, bool debug = false);


//    Eigen::Matrix4d super4PCSRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
//                                          pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
//                                          Eigen::Matrix4d initialGuess, bool useInitialGuess, bool debug = false);

    Eigen::Matrix4d ndt_d2d_2d(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                               pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan, Eigen::Matrix4d initialGuess,
                               bool useInitialGuess);

    Eigen::Matrix4d ndt_p2d(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                            pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan, Eigen::Matrix4d initialGuess,
                            bool useInitialGuess);

    Eigen::Matrix4d registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],double maximumVoxel1,
                                                     double voxelData2Input[],double maximumVoxel2,
                                                     Eigen::Matrix4d initialGuess,
                                                     Eigen::Matrix3d &covarianceMatrix,
                                                     double cellSize,double &timeToCalculate);

    std::vector<fs2d::msg::PotentialSolution> registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],double maximumVoxel1,
                                                                                  double voxelData2Input[],double maximumVoxel2,
                                                                                  Eigen::Matrix4d initialGuess,
                                                                                  Eigen::Matrix3d &covarianceMatrix,
                                                                                  double cellSize,double &timeToCalculate);

    Eigen::Matrix4d registrationFourerMellin(double voxelData1Input[],
                                             double voxelData2Input[],
                                             double cellSize,
                                             bool debug = false);

    Eigen::Matrix4d registrationFeatureBased(double voxelData1Input[],
                                             double voxelData2Input[],
                                             double cellSize,int methodType,
                                             bool debug = false);

    Eigen::Matrix4d gmmRegistrationD2D(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                       pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                       Eigen::Matrix4d initialGuess, bool useInitialGuess, bool debug = false);

    Eigen::Matrix4d gmmRegistrationP2D(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                       pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                       Eigen::Matrix4d initialGuess, bool useInitialGuess, bool debug = false);



private:
//    softDescriptorRegistration mySofftRegistrationClass;
    rclcpp::Client<fs2d::srv::RequestOnePotentialSolution>::SharedPtr onePotentialClient;
    rclcpp::Client<fs2d::srv::RequestListPotentialSolution>::SharedPtr listPotentialClient;
    int sizeVoxelData;

    std::mutex *icpMutex,*ndtd2dMutex,*ndtp2dMutex,*fourierMellinMutex,*oursMutex,*featureBasedMutex,*gmmd2dMutex,*gmmp2dMutex;
};


#endif //SIMULATION_BLUEROV_SCANREGISTRATIONCLASS_H
