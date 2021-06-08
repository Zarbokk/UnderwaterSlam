//
// Created by tim on 16.02.21.
//

#include "scanRegistrationClass.h"

Eigen::Matrix4d scanRegistrationClass::generalizedIcpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &Final,
                                                       double &fitnessScore,Eigen::Matrix4d &initialGuessTransformation){

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> gicp;
    gicp.setInputSource(cloudFirstScan);
    gicp.setInputTarget(cloudSecondScan);
    //gicp.setSourceCovariances(source_covariances);
    //gicp.setTargetCovariances(target_covariances);
    gicp.setMaxCorrespondenceDistance(10);
    gicp.setRANSACOutlierRejectionThreshold(5);
    //gicp.setMaximumIterations(1);
//    gicp.setMaximumOptimizerIterations(100);
//    gicp.setMaximumIterations(100);
//    gicp.setRANSACIterations(100);

    gicp.align(*Final,initialGuessTransformation.cast<float>());
    //std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
    //          gicp.getFitnessScore() << std::endl;
    fitnessScore = gicp.getFitnessScore();
    return gicp.getFinalTransformation().cast<double>();
}

Eigen::Matrix4d scanRegistrationClass::generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                                  double &fitnessScore){
    Eigen::Matrix4d guess;
    guess << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    return scanRegistrationClass::generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                           fitnessScore,guess);
}

Eigen::Matrix4d scanRegistrationClass::generalizedIcpRegistrationSimple(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                                        double &fitnessScore,Eigen::Matrix4d &guess){

    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    return scanRegistrationClass::generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                                             fitnessScore,guess);
}




Eigen::Matrix4d scanRegistrationClass::icpRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                            pcl::PointCloud<pcl::PointXYZ> &Final){
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloudFirstScan);
    icp.setInputTarget(cloudSecondScan);
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return icp.getFinalTransformation().cast<double>();
}

