//
// Created by tim on 16.02.21.
//

#include "scanRegistrationClass.h"

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &Final,
                                                  double &fitnessScore, Eigen::Matrix4d &initialGuessTransformation) {

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(cloudFirstScan);
    gicp.setInputTarget(cloudSecondScan);
    //gicp.setSourceCovariances(source_covariances);
    //gicp.setTargetCovariances(target_covariances);
    //gicp.setMaxCorrespondenceDistance(10);
    //gicp.setRANSACOutlierRejectionThreshold(15);
    //gicp.setMaximumIterations(0);
//    gicp.setMaximumOptimizerIterations(100);
//    gicp.setMaximumIterations(100);
//    gicp.setRANSACIterations(100);

    gicp.align(*Final, initialGuessTransformation.cast<float>());
    //std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
    //          gicp.getFitnessScore() << std::endl;
    fitnessScore = gicp.getFitnessScore();
    return gicp.getFinalTransformation().cast<double>();
}

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                        double &fitnessScore) {
    Eigen::Matrix4d guess;
    guess << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    return scanRegistrationClass::generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                                             fitnessScore, guess);
}

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                        double &fitnessScore, Eigen::Matrix4d &guess) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    return scanRegistrationClass::generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                                             fitnessScore, guess);
}


Eigen::Matrix4d scanRegistrationClass::icpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                       pcl::PointCloud<pcl::PointXYZ> &Final) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudFirstScan);
    icp.setInputTarget(cloudSecondScan);
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return icp.getFinalTransformation().cast<double>();
}


Eigen::Matrix4d scanRegistrationClass::sofftRegistration(const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1,
                                                         const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2, double &fitnessX, double &fitnessY,
                                                         double goodGuessAlpha,bool debug) {

    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1New(pointCloudInputData1.makeShared());
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2New(pointCloudInputData2.makeShared());

    return myRegistrationClass.registrationOfTwoPCL2D(pointCloudInputData1New, pointCloudInputData2New, fitnessX,
                                                      fitnessY, goodGuessAlpha, debug);
}

double scanRegistrationClass::sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[],double voxelData2Input[],double goodGuessAlpha,bool debug){


    return myRegistrationClass.sofftRegistrationVoxel2DRotationOnly(voxelData1Input,voxelData2Input,goodGuessAlpha,debug);

}

Eigen::Vector2d
scanRegistrationClass::sofftRegistrationVoxel2DTranslation(double voxelData1Input[], double voxelData2Input[], double &fitnessX, double &fitnessY,double cellSize,
                                    Eigen::Vector3d initialGuess,bool useInitialGuess,bool debug){

    return myRegistrationClass.sofftRegistrationVoxel2DTransformation(voxelData1Input,voxelData2Input,fitnessX,fitnessY,cellSize,initialGuess,useInitialGuess,debug);

}

//Eigen::Matrix4d sofftRegistrationVoxel(double voxelData1[],
//                                       double voxelData2[],
//                                       double &fitnessX, double &fitnessY, double goodGuessAlpha = -100,bool debug = false){
//
//}