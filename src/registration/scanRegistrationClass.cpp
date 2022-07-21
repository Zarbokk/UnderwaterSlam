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
//    gicp.setSourceCovariances(source_covariances);
//    gicp.setTargetCovariances(target_covariances);
    gicp.setMaxCorrespondenceDistance(1.0);
//    gicp.setRANSACOutlierRejectionThreshold(15);
//    gicp.setMaximumIterations(0);
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
                                                         const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2,
                                                         double &fitnessX, double &fitnessY,
                                                         double goodGuessAlpha, bool debug) {

    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1New(pointCloudInputData1.makeShared());
    const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2New(pointCloudInputData2.makeShared());

    return mySofftRegistrationClass.registrationOfTwoPCL2D(pointCloudInputData1New, pointCloudInputData2New, fitnessX,
                                                           fitnessY, goodGuessAlpha, debug);
}

double scanRegistrationClass::sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[],
                                                                   double goodGuessAlpha, bool debug) {


    return mySofftRegistrationClass.sofftRegistrationVoxel2DRotationOnly(voxelData1Input, voxelData2Input,
                                                                         goodGuessAlpha, debug);

}

Eigen::Vector2d
scanRegistrationClass::sofftRegistrationVoxel2DTranslation(double voxelData1Input[], double voxelData2Input[],
                                                           double &fitnessX, double &fitnessY, double cellSize,
                                                           Eigen::Vector3d initialGuess, bool useInitialGuess,
                                                           bool debug) {

    return mySofftRegistrationClass.sofftRegistrationVoxel2DTransformation(voxelData1Input, voxelData2Input, fitnessX,
                                                                           fitnessY, cellSize, initialGuess,
                                                                           useInitialGuess, debug);

}

Eigen::Matrix4d scanRegistrationClass::super4PCSRegistration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
                                                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
                                                             Eigen::Matrix4d initialGuess, bool useInitialGuess,
                                                             bool debug) {

    using TrVisitor = gr::DummyTransformVisitor;

    using MatcherType = gr::Match4pcsBase<gr::FunctorSuper4PCS, gr::Point3D<float>, TrVisitor, gr::AdaptivePointFilter, gr::AdaptivePointFilter::Options>;
    using OptionType = typename MatcherType::OptionsType;
    using SamplerType = gr::UniformDistSampler<gr::Point3D<float> >;

    std::vector<gr::Point3D<float>> set1, set2;
    std::vector<Eigen::Matrix2f> tex_coords1, tex_coords2;
    std::vector<typename gr::Point3D<float>::VectorType> normals1, normals2;
    std::vector<std::string> mtls1, mtls2;

    //fill in set1 and set2

//    pcl::PointCloud<pcl::PointXYZ>::Ptr scan1(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr scan2(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",*scan1);
//    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",*scan2);

    Eigen::Matrix4d transformationX180Degree;
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationX180Degree(3, 3) = 1;

//    pcl::transformPointCloud(*scan1, *scan1, transformationX180Degree);
//    pcl::transformPointCloud(*scan2, *scan2, transformationX180Degree);





    for (int i = 0; i < cloudFirstScan->points.size(); i++) {
        gr::Point3D<float> tmpPoint;
        tmpPoint.x() = cloudFirstScan->points.at(i).x;
        tmpPoint.y() = cloudFirstScan->points.at(i).y;
        tmpPoint.z() = cloudFirstScan->points.at(i).z;
        set1.push_back(tmpPoint);
    }
    for (int i = 0; i < cloudSecondScan->points.size(); i++) {
        gr::Point3D<float> tmpPoint;
        tmpPoint.x() = cloudSecondScan->points.at(i).x;
        tmpPoint.y() = cloudSecondScan->points.at(i).y;
        tmpPoint.z() = cloudSecondScan->points.at(i).z;
        set2.push_back(tmpPoint);
    }


    // dummy calls, to test symbols accessibility
    // check availability of the Utils functions
    gr::Utils::CleanInvalidNormals(set1, normals1);

    // Our matcher.
    OptionType options;

    // Set parameters.
    typename MatcherType::MatrixType mat;
    std::vector<Eigen::Matrix4d> transformationsList;
    std::vector<double> scoreList;
    for (int i = 2; i < 8; i++) {
        // test different Overlap
        double overlap(0.1 + 0.1 * i);
        options.configureOverlap(overlap);
        options.delta = 0.1;
        options.sample_size = 2000;


        typename gr::Point3D<float>::Scalar score = 0;

        constexpr gr::Utils::LogLevel loglvl = gr::Utils::NoLog;
        gr::Utils::Logger logger(loglvl);
        SamplerType sampler;
        TrVisitor visitor;

        MatcherType matcher(options, logger);
        score = matcher.ComputeTransformation(set1, set2, mat, sampler, visitor);

        logger.Log<gr::Utils::Verbose>("Score: ", score);

        //change to NED

        mat(0, 1) = -mat(0, 1);
        mat(1, 0) = -mat(1, 0);
        mat(1, 3) = -mat(1, 3);

        //calculate a score how good that is.
        Eigen::Matrix4d saveMat;
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                saveMat(k, j) = mat(k, j);
            }
        }


        double initialGuessAngle = std::atan2(initialGuess(1, 0),
                                              initialGuess(0, 0));
        double estimatedAngle = std::atan2(saveMat(1, 0),
                                           saveMat(0, 0));
        double angleDiff = generalHelpfulTools::angleDiff(initialGuessAngle, estimatedAngle);
        double distanceEuclidDiff = (saveMat.block<3, 1>(0, 3) - initialGuess.block<3, 1>(0, 3)).norm();

        double scoreInitialGuessAndEstimation = distanceEuclidDiff * sqrt(abs(angleDiff));
        transformationsList.push_back(saveMat);
        scoreList.push_back(scoreInitialGuessAndEstimation);
    }


    auto maxElementIter = std::min_element(scoreList.begin(), scoreList.end());
    int distanceToMaxElement = (int) std::distance(scoreList.begin(), maxElementIter);

//    for(int i = 0 ; i<transformationsList.size();i++){
//        std::cout << scoreList[i] << std::endl;
//        std::cout << transformationsList[i] << std::endl;
//
//    }
//
//
//    std::cout << "score and mat: "<< std::endl;
//    std::cout << transformationsList[distanceToMaxElement] << std::endl;
//    std::cout << scoreList[distanceToMaxElement] << std::endl;
//    std::cout << mat << std::endl;


    //copy mat to Eigen Matrix4d


    return transformationsList[distanceToMaxElement];
}


Eigen::Matrix4d
scanRegistrationClass::FMSRegistrationOld(double voxelData1Input[], double voxelData2Input[], double cellSize,
                                          bool debug) {

    cv::Mat im0(this->sizeVoxelData, this->sizeVoxelData, CV_64F);
    std::memcpy(im0.data, voxelData1Input, this->sizeVoxelData * this->sizeVoxelData * sizeof(double));
    cv::Mat im1(this->sizeVoxelData, this->sizeVoxelData, CV_64F);
    std::memcpy(im1.data, voxelData2Input, this->sizeVoxelData * this->sizeVoxelData * sizeof(double));


    for (int i = 0; i < this->sizeVoxelData; i++) {
        for (int j = 0; j < this->sizeVoxelData; j++) {

            im0.at<double>(i, j) = im0.at<double>(i, j) / 255;
            im1.at<double>(i, j) = im1.at<double>(i, j) / 255;
        }
    }






//    cv::imshow("im0", im0);
//    cv::imshow("im1", im1);
//    cv::waitKey(0);
    ImageRegistration image_registration(im0);

    // x, y, rotation, scale
    std::vector<double> transform_params(4, 0.0);
    cv::Mat registered_image;
    image_registration.registerImage(im1, registered_image, transform_params, true);


//    std::cout << "x: " << transform_params[0]*cellSize << ", y: "
//              << transform_params[1]*cellSize << ", rotation: " << transform_params[2]
//              << ", scale: " << transform_params[3] << std::endl;

//    cv::Mat overlay_image;
//    cv::addWeighted(image_registration.getCurrentImage(), 0.5, registered_image, 0.5, 0.0, overlay_image);
//    cv::imwrite("/home/tim-external/Documents/imreg_fmt/result.jpg", overlay_image);
//    cv::imshow("overlay_image", overlay_image);

//    cv::waitKey(0);

    Eigen::Matrix4d returnMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, transform_params[2]);
    returnMatrix(1, 3) = transform_params[1] * cellSize;
    returnMatrix(0, 3) = transform_params[0] * cellSize;
    return returnMatrix;
}


Eigen::Matrix4d scanRegistrationClass::normalDistributionsTransformRegistration(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFirstScan,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudSecondScan,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &Final,
        double &fitnessScore,
        Eigen::Matrix4d &initialGuessTransformation, double ndt_resolution, double ndt_step_size,
        double transform_epsilon) {


    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter_;
    voxel_grid_filter_.setLeafSize(0.5,0.5,0.5);
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration_;
    pcl::NormalDistributionsTransform2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
            new pcl::NormalDistributionsTransform2D<pcl::PointXYZ, pcl::PointXYZ>());
    ndt->setStepSize(ndt_step_size);
//    std::cout << "get Step size: "<< ndt->getStepSize() << std::endl;
//    std::cout << "get getResolution: "<< ndt->getResolution() << std::endl;
//    std::cout << "get getTransformationEpsilon: "<< ndt->getTransformationEpsilon() << std::endl;
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(transform_epsilon);
    ndt->setMaximumIterations(100000);
    registration_ = ndt;



    //input cloud(has to be copied
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//    *cloud_ptr = *cloudFirstScan;
//
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_grid_filter_.setInputCloud(cloudFirstScan);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);


    registration_->setInputSource(filtered_cloud_ptr);
    registration_->setInputTarget(cloudSecondScan);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    registration_->align(*output_cloud, initialGuessTransformation.cast<float>());
    std::cout << "has converged?: " << registration_->hasConverged() << std::endl;


    Eigen::Matrix4d final_transformation = registration_->getFinalTransformation().cast<double>();
    return final_transformation;

}


