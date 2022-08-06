//
// Created by tim on 16.02.21.
//

#include "scanRegistrationClass.h"

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                  pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                  pcl::PointCloud<pcl::PointXYZ> &Final,
                                                  double &fitnessScore, Eigen::Matrix4d &initialGuessTransformation) {

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(cloudFirstScan.makeShared());
    gicp.setInputTarget(cloudSecondScan.makeShared());
//    gicp.setSourceCovariances(source_covariances);
//    gicp.setTargetCovariances(target_covariances);
    gicp.setMaxCorrespondenceDistance(1.0);
//    gicp.setRANSACOutlierRejectionThreshold(15);
//    gicp.setMaximumIterations(0);
//    gicp.setMaximumOptimizerIterations(100);
//    gicp.setMaximumIterations(100);
//    gicp.setRANSACIterations(100);

    gicp.align(Final, initialGuessTransformation.cast<float>());
    //std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
    //          gicp.getFitnessScore() << std::endl;
    fitnessScore = gicp.getFitnessScore();
    return gicp.getFinalTransformation().cast<double>();
}

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistrationSimple(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                        pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                        double &fitnessScore) {
    Eigen::Matrix4d guess;
    guess << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    pcl::PointCloud<pcl::PointXYZ> Final;
    return scanRegistrationClass::generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                                             fitnessScore, guess);
}

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistrationSimple(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                        pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                        double &fitnessScore, Eigen::Matrix4d &guess) {

    pcl::PointCloud<pcl::PointXYZ> Final;
    return scanRegistrationClass::generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                                             fitnessScore, guess);
}


Eigen::Matrix4d scanRegistrationClass::icpRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                       pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                       pcl::PointCloud<pcl::PointXYZ> &Final) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL1 = cloudFirstScan.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL2 = cloudSecondScan.makeShared();

    icp.setInputSource(tmpPCL1);
    icp.setInputTarget(tmpPCL2);

    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;




    return icp.getFinalTransformation().cast<double>();
}


Eigen::Matrix4d scanRegistrationClass::sofftRegistration2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData1,
                                                           pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData2,
                                                           double &fitnessX, double &fitnessY, double goodGuessAlpha, bool debug) {

//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1New(pointCloudInputData1.makeShared());
//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2New(pointCloudInputData2.makeShared());

    return mySofftRegistrationClass.registrationOfTwoPCL2D(pointCloudInputData1, pointCloudInputData2, fitnessX,
                                                           fitnessY, goodGuessAlpha, debug);
}

Eigen::Matrix4d scanRegistrationClass::sofftRegistration2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData1,
                                                           pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData2,
                                                           double &fitnessX, double &fitnessY,
                                                           Eigen::Matrix4d initialGuess,bool useInitialGuess,
                                                           bool debug) {

//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1New(pointCloudInputData1.makeShared());
//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2New(pointCloudInputData2.makeShared());

    return mySofftRegistrationClass.registrationOfTwoPCL2D(pointCloudInputData1, pointCloudInputData2, fitnessX,
                                                           fitnessY, initialGuess,useInitialGuess, debug);
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

Eigen::Matrix4d scanRegistrationClass::super4PCSRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                             pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
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

//    pcl::PointCloud<pcl::PointXYZ> scan1(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ> scan2(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",*scan1);
//    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",*scan2);

    Eigen::Matrix4d transformationX180Degree;
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationX180Degree(3, 3) = 1;

//    pcl::transformPointCloud(*scan1, *scan1, transformationX180Degree);
//    pcl::transformPointCloud(*scan2, *scan2, transformationX180Degree);





    for (int i = 0; i < cloudFirstScan.points.size(); i++) {
        gr::Point3D<float> tmpPoint;
        tmpPoint.x() = cloudFirstScan.points.at(i).x;
        tmpPoint.y() = cloudFirstScan.points.at(i).y;
        tmpPoint.z() = cloudFirstScan.points.at(i).z;
        set1.push_back(tmpPoint);
    }
    for (int i = 0; i < cloudSecondScan.points.size(); i++) {
        gr::Point3D<float> tmpPoint;
        tmpPoint.x() = cloudSecondScan.points.at(i).x;
        tmpPoint.y() = cloudSecondScan.points.at(i).y;
        tmpPoint.z() = cloudSecondScan.points.at(i).z;
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
//    std::cout << "we are here1" << std::endl;

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
//        std::cout << "we are here2" << std::endl;
        MatcherType matcher(options, logger);
        score = matcher.ComputeTransformation(set1, set2, mat, sampler, visitor);
//        std::cout << "we are here3" << std::endl;
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

Eigen::Matrix4d scanRegistrationClass::ndt_d2d_2d(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                           pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan, Eigen::Matrix4d initialGuess,
                           bool useInitialGuess){

    initialGuess(0, 1) = -initialGuess(0, 1);
    initialGuess(1, 0) = -initialGuess(1, 0);
    initialGuess(1, 3) = -initialGuess(1, 3);


    double __res[] = {0.5, 1, 2, 4};
    std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));


    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tout(initialGuess);
//    Tout.setIdentity();


//    std::cout<<"Transform Before: \n"<<Tout.matrix()<<std::endl;
    lslgeneric::NDTMatcherD2D_2D<pcl::PointXYZ,pcl::PointXYZ> matcherD2D(false, false, resolutions);
        bool ret = matcherD2D.match(cloudSecondScan,cloudFirstScan,Tout,true);

    Tout(0, 1) = -Tout(0, 1);
    Tout(1, 0) = -Tout(1, 0);
    Tout(1, 3) = -Tout(1, 3);

//    std::cout<<"Transform: \n"<<Tout.matrix()<<std::endl;
    return Tout.matrix();
}

Eigen::Matrix4d scanRegistrationClass::ndt_p2d(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                        pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan, Eigen::Matrix4d initialGuess,
                        bool useInitialGuess){

    initialGuess(0, 1) = -initialGuess(0, 1);
    initialGuess(1, 0) = -initialGuess(1, 0);
    initialGuess(1, 3) = -initialGuess(1, 3);
//
//    printf("X %f Y %f Z %f Roll %f Pitch %f Yaw %f \n",xoffset,yoffset,zoffset,roll,pitch,yaw);


    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tout(initialGuess);

//    std::cout<<"Transform Before: \n"<<Tout.matrix()<<std::endl;

    lslgeneric::NDTMatcherP2D<pcl::PointXYZ,pcl::PointXYZ> matcherP2D;

    bool ret = matcherP2D.match(cloudSecondScan,cloudFirstScan,Tout);

    Tout(0, 1) = -Tout(0, 1);
    Tout(1, 0) = -Tout(1, 0);
    Tout(1, 3) = -Tout(1, 3);

//    std::cout<<"Transform: \n"<<Tout.matrix()<<std::endl;


    return Tout.matrix();
}


