//
// Created by tim on 16.02.21.
//


#include "scanRegistrationClass.h"

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                  pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                  pcl::PointCloud<pcl::PointXYZ> &Final,
                                                  double &fitnessScore, Eigen::Matrix4d &initialGuessTransformation) {

    std::lock_guard<std::mutex> guard(*this->icpMutex);

    if(cloudFirstScan.size()<20 || cloudSecondScan.size()<20){
        Eigen::Matrix4d guess = Eigen::Matrix4d::Identity();
        guess(3,3) = -1;
        return guess;
    }
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

    // change here again the direction because we want to have the transformation from 1 to 2 and not from 2 to 1(which is the registration)
    gicp.setInputSource(cloudSecondScan.makeShared());
    gicp.setInputTarget(cloudFirstScan.makeShared());
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
    return this->generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
                                                             fitnessScore, guess);
}

Eigen::Matrix4d
scanRegistrationClass::generalizedIcpRegistrationSimple(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                        pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                        double &fitnessScore, Eigen::Matrix4d &guess) {

    pcl::PointCloud<pcl::PointXYZ> Final;
    return this->generalizedIcpRegistration(cloudFirstScan, cloudSecondScan, Final,
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
                                                           double &fitnessX, double &fitnessY, double goodGuessAlpha,
                                                           bool debug) {

//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1New(pointCloudInputData1.makeShared());
//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2New(pointCloudInputData2.makeShared());
    std::cout << "not Implemented" << std::endl;
    return Eigen::Matrix4d::Identity();
//    return mySofftRegistrationClass.registrationOfTwoPCL2D(pointCloudInputData1, pointCloudInputData2, fitnessX,
//                                                           fitnessY, goodGuessAlpha, debug);
}

Eigen::Matrix4d scanRegistrationClass::sofftRegistration2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData1,
                                                           pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData2,
                                                           double &fitnessX, double &fitnessY,
                                                           Eigen::Matrix4d initialGuess, bool useInitialGuess,
                                                           bool debug) {

//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData1New(pointCloudInputData1.makeShared());
//    const pcl::PointCloud<pcl::PointXYZ> pointCloudInputData2New(pointCloudInputData2.makeShared());
    std::cout << "not Implemented" << std::endl;
    return Eigen::Matrix4d::Identity();
//    return mySofftRegistrationClass.registrationOfTwoPCL2D(pointCloudInputData1, pointCloudInputData2, fitnessX,
//                                                           fitnessY, initialGuess, useInitialGuess, debug);
}


double scanRegistrationClass::sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[],
                                                                   double goodGuessAlpha,double &covariance, bool debug) {


    return mySofftRegistrationClass.sofftRegistrationVoxel2DRotationOnly(voxelData1Input, voxelData2Input,
                                                                         goodGuessAlpha,covariance, debug);

}

//Eigen::Vector2d
//scanRegistrationClass::sofftRegistrationVoxel2DTranslation(double voxelData1Input[], double voxelData2Input[],
//                                                           double &fitnessX, double &fitnessY, double cellSize,
//                                                           Eigen::Vector3d initialGuess, bool useInitialGuess,
//                                                           bool debug) {
//
//    return mySofftRegistrationClass.sofftRegistrationVoxel2DTranslation(voxelData1Input, voxelData2Input, fitnessX,
//                                                                        fitnessY, cellSize, initialGuess,
//                                                                        useInitialGuess, debug);
//
//}

Eigen::Matrix4d scanRegistrationClass::super4PCSRegistration(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                             pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                             Eigen::Matrix4d initialGuess, bool useInitialGuess,
                                                             bool debug) {
    std::lock_guard<std::mutex> guard(*this->supersMutex);

//    Eigen::Matrix4d transformationX180Degree = generalHelpfulTools::getTransformationMatrixFromRPY(0.00001, 0, 0);
//    pcl::transformPointCloud(cloudFirstScan, cloudFirstScan, transformationX180Degree);
//    pcl::transformPointCloud(cloudSecondScan, cloudSecondScan, transformationX180Degree);

    Eigen::Matrix4d initialGuessTMP = initialGuess.inverse();
    initialGuess = initialGuessTMP;


    using TrVisitor = gr::DummyTransformVisitor;

    using MatcherType = gr::Match4pcsBase<gr::FunctorSuper4PCS, gr::Point3D<float>, TrVisitor, gr::AdaptivePointFilter, gr::AdaptivePointFilter::Options>;
    using OptionType = typename MatcherType::OptionsType;
    using SamplerType = gr::UniformDistSampler<gr::Point3D<float>>;

    std::vector<gr::Point3D<float>>
            set1, set2;
    std::vector<Eigen::Matrix2f> tex_coords1, tex_coords2;
    std::vector<typename gr::Point3D<float>::VectorType> normals1, normals2;
    std::vector<std::string> mtls1, mtls2;

    //fill in set1 and set2

//    pcl::PointCloud<pcl::PointXYZ> scan1(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ> scan2(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",*scan1);
//    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",*scan2);

//    Eigen::Matrix4d transformationX180Degree;
//    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
//    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
//    transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
//    transformationX180Degree(3, 3) = 1;

//    pcl::transformPointCloud(*scan1, *scan1, transformationX180Degree);
//    pcl::transformPointCloud(*scan2, *scan2, transformationX180Degree);


//    std::cout << cloudFirstScan.points.size() << std::endl;
//
//    std::cout << cloudSecondScan.points.size() << std::endl;

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
//        std::cout << i << std::endl;
        // test different Overlap
        double overlap(0.1 + 0.1 * i);
        options.configureOverlap(overlap);
        options.delta = 4.5;
//        options.sample_size = 1000;
        options.max_angle = -1;
        options.max_translation_distance = -1;
        options.sample_size = (cloudFirstScan.points.size() + cloudSecondScan.points.size()) / 2;


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

//        mat(0, 1) = -mat(0, 1);
//        mat(1, 0) = -mat(1, 0);
//        mat(1, 3) = -mat(1, 3);
//        mat(0, 3) = -mat(0, 3);
        //calculate a score how good that is.
        Eigen::Matrix4d saveMat;
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                saveMat(k, j) = mat(k, j);
            }
        }
//        std::cout << "Init guess: " << std::endl;
//        std::cout << initialGuess << std::endl;
//        std::cout << "current test: " << std::endl;
//        std::cout << saveMat << std::endl;
        Eigen::Matrix4d tmpMat = saveMat.inverse();
        saveMat = tmpMat;

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


    return transformationsList[distanceToMaxElement].inverse();
}


Eigen::Matrix4d scanRegistrationClass::ndt_d2d_2d(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                  pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                  Eigen::Matrix4d initialGuess,
                                                  bool useInitialGuess) {
    std::lock_guard<std::mutex> guard(*this->ndtd2dMutex);
    Eigen::Matrix4d transformationX180Degree = generalHelpfulTools::getTransformationMatrixFromRPY(0.00001, 0, 0);
    pcl::transformPointCloud(cloudFirstScan, cloudFirstScan, transformationX180Degree);
    pcl::transformPointCloud(cloudSecondScan, cloudSecondScan, transformationX180Degree);
//    initialGuess(0, 1) = -initialGuess(0, 1);
//    initialGuess(1, 0) = -initialGuess(1, 0);
//    initialGuess(1, 3) = -initialGuess(1, 3);


    double __res[] = {0.5, 1, 2, 4};
    std::vector<double> resolutions(__res, __res + sizeof(__res) / sizeof(double));


    Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> Tout(initialGuess);
//    Tout.setIdentity();


//    std::cout<<"Transform Before: \n"<<Tout.matrix()<<std::endl;
    lslgeneric::NDTMatcherD2D_2D<pcl::PointXYZ, pcl::PointXYZ> matcherD2D(false, false, resolutions);
    bool ret = matcherD2D.match(cloudFirstScan, cloudSecondScan, Tout, useInitialGuess);

//    Tout(0, 1) = -Tout(0, 1);
//    Tout(1, 0) = -Tout(1, 0);
//    Tout(1, 3) = -Tout(1, 3);

//    std::cout<<"Transform: \n"<<Tout.matrix()<<std::endl;
    return Tout.matrix();
}

Eigen::Matrix4d scanRegistrationClass::ndt_p2d(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                               pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                               Eigen::Matrix4d initialGuess,
                                               bool useInitialGuess) {
    std::lock_guard<std::mutex> guard(*this->ndtp2dMutex);
    Eigen::Matrix4d transformationX180Degree = generalHelpfulTools::getTransformationMatrixFromRPY(0.00001, 0, 0);
    pcl::transformPointCloud(cloudFirstScan, cloudFirstScan, transformationX180Degree);
    pcl::transformPointCloud(cloudSecondScan, cloudSecondScan, transformationX180Degree);

//    initialGuess(0, 1) = -initialGuess(0, 1);
//    initialGuess(1, 0) = -initialGuess(1, 0);
//    initialGuess(1, 3) = -initialGuess(1, 3);
//
//    printf("X %f Y %f Z %f Roll %f Pitch %f Yaw %f \n",xoffset,yoffset,zoffset,roll,pitch,yaw);


    Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> Tout(initialGuess);

//    std::cout<<"Transform Before: \n"<<Tout.matrix()<<std::endl;

    lslgeneric::NDTMatcherP2D<pcl::PointXYZ, pcl::PointXYZ> matcherP2D;

    bool ret = matcherP2D.match(cloudFirstScan, cloudSecondScan, Tout);

//    Tout(0, 1) = Tout(0, 1);
//    Tout(1, 0) = Tout(1, 0);
//    Tout(1, 3) = Tout(1, 3);

//    std::cout<<"Transform: \n"<<Tout.matrix()<<std::endl;


    return Tout.matrix();
}


//Eigen::Vector2d scanRegistrationClass::sofftRegistrationVoxel2DTranslation(double voxelData1Input[],
//                                                                           double voxelData2Input[],
//                                                                           double &fitnessX, double &fitnessY,
//                                                                           double cellSize,
//                                                                           Eigen::Vector3d initialGuess,
//                                                                           bool useInitialGuess,
//                                                                           double &heightMaximumPeak, bool debug) {
//
//    this->mySofftRegistrationClass.sofftRegistrationVoxel2DTranslation(voxelData1Input, voxelData2Input, fitnessX,
//                                                                       fitnessY, cellSize, initialGuess,
//                                                                       useInitialGuess, heightMaximumPeak, debug);
//
//}

Eigen::Matrix4d scanRegistrationClass::registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                                        double voxelData2Input[],
                                                                        Eigen::Matrix4d initialGuess,
                                                                        Eigen::Matrix3d &covarianceMatrix,
                                                                        bool useInitialAngle,
                                                                        bool useInitialTranslation,
                                                                        double cellSize,
                                                                        bool useGauss,
                                                                        bool debug) {

    std::lock_guard<std::mutex> guard(*this->oursMutex);
    std::cout << "Starting soft: " <<this->sizeVoxelData << std::endl;

    //changing voxel 1 and 2 because we want to have the transformation from 1 to 2 and not from 2 to 1(which is the registration)@TODO
    return mySofftRegistrationClass.registrationOfTwoVoxelsSOFFTFast(voxelData1Input,
                                                                     voxelData2Input,
                                                                     initialGuess,covarianceMatrix,
                                                                     useInitialAngle, useInitialTranslation,
                                                                     cellSize,
                                                                     useGauss,
                                                                     debug);
}

std::vector<transformationPeak>
scanRegistrationClass::registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
                                                                 double voxelData2Input[],
                                                                 double cellSize,
                                                                 bool useGauss,
                                                                 bool debug, double potentialNecessaryForPeak,
                                                                 bool multipleRadii,
                                                                 bool useClahe,
                                                                 bool useHamming) {

    std::lock_guard<std::mutex> guard(*this->oursMutex);
    std::cout << "Starting soft: " <<this->sizeVoxelData << std::endl;
    //changing voxel 1 and 2 because we want to have the transformation from 1 to 2 and not from 2 to 1(which is the registration)@TODO
    return mySofftRegistrationClass.registrationOfTwoVoxelsSOFFTAllSoluations(voxelData1Input,
                                                                              voxelData2Input,
                                                                              cellSize,
                                                                              useGauss,
                                                                              debug, potentialNecessaryForPeak,
                                                                              multipleRadii,
                                                                              useClahe,
                                                                              useHamming);
}


//Eigen::Matrix4d
//scanRegistrationClass::registrationOfTwoVoxelsSOFFTAllSoluations(double voxelData1Input[],
//                                                                 double voxelData2Input[],
//                                                                 double cellSize,
//                                                                 bool useGauss,
//                                                                 bool debug, double potentialNecessaryForPeak,
//                                                                 bool multipleRadii,
//                                                                 bool useClahe,
//                                                                 bool useHamming, Eigen::Matrix4d initialGuess, bool useInitialGuess) {
//
//
//
//    //changing voxel 1 and 2 because we want to have the transformation from 1 to 2 and not from 2 to 1(which is the registration)@TODO
//    std::vector<transformationPeak> estimatedTransformations =  mySofftRegistrationClass.registrationOfTwoVoxelsSOFFTAllSoluations(voxelData1Input,
//                                                                              voxelData2Input,
//                                                                              cellSize,
//                                                                              useGauss,
//                                                                              debug, potentialNecessaryForPeak,
//                                                                              multipleRadii,
//                                                                              useClahe,
//                                                                              useHamming);
//
//    if(!useInitialGuess){
//        double highestPeak = 0;
//        Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
//        for (auto &estimatedTransformation: estimatedTransformations) {
//            //rotation
//
//            for (auto &potentialTranslation: estimatedTransformation.potentialTranslations) {
//                if (potentialTranslation.peakHeight > highestPeak) {
//                    currentMatrix.block<3, 3>(0, 0) = generalHelpfulTools::getQuaternionFromRPY(0, 0,
//                                                                                                estimatedTransformation.potentialRotation.angle).toRotationMatrix();
//                    currentMatrix.block<3, 1>(0, 3) = Eigen::Vector3d(potentialTranslation.translationSI.x(),
//                                                                      potentialTranslation.translationSI.y(), 0);
//                    std::cout << estimatedTransformation.potentialRotation.angle << std::endl;
//                    highestPeak = potentialTranslation.peakHeight;
//                }
//                //translation
//            }
//        }
//
//        std::cout << "our match" << std::endl;
//        std::cout << currentMatrix << std::endl;
//        return currentMatrix;
//    }else{
//
//
//    }
//
//}



Eigen::Matrix4d scanRegistrationClass::registrationFourerMellin(double voxelData1Input[],
                                                                double voxelData2Input[],
                                                                double cellSize,
                                                                bool debug) {
    std::lock_guard<std::mutex> guard(*this->fourierMellinMutex);
    cv::Mat convertedMat1;
    cv::Mat convertedMat2;


    cv::Mat magTMP1(this->sizeVoxelData, this->sizeVoxelData, CV_64F, voxelData1Input);
    cv::Mat magTMP2(this->sizeVoxelData, this->sizeVoxelData, CV_64F, voxelData2Input);
    magTMP1.convertTo(convertedMat1, CV_8U);
    magTMP2.convertTo(convertedMat2, CV_8U);

    cv::cvtColor(convertedMat1, convertedMat1, cv::COLOR_GRAY2BGR);
    cv::cvtColor(convertedMat2, convertedMat2, cv::COLOR_GRAY2BGR);




    fourierMellinRegistration image_registration(convertedMat1);

    // x, y, rotation, scale
    std::vector<double> transform_params(4, 0.0);
    cv::Mat registered_image;
    image_registration.registerImage(convertedMat2, registered_image, transform_params, debug);

    Eigen::Matrix4d resultTransformation = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rotation_vector2(transform_params[2] / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    resultTransformation.block<3, 3>(0, 0) = tmpMatrix3d;

    resultTransformation(0, 3) = transform_params[1] * cellSize;
    resultTransformation(1, 3) = transform_params[0] * cellSize;
    resultTransformation(2, 3) = 0;
    if (debug) {
        cv::imshow("im0_rotated", convertedMat1);
        cv::imshow("im1_rotated", convertedMat2);
        cv::waitKey(0);
    }

    return resultTransformation;
}

Eigen::Matrix4d scanRegistrationClass::registrationFeatureBased(double voxelData1Input[],
                                                                double voxelData2Input[],
                                                                double cellSize,int methodType,
                                                                bool debug) {
    std::lock_guard<std::mutex> guard(*this->featureBasedMutex);

    cv::Mat magTMP1(this->sizeVoxelData, this->sizeVoxelData, CV_64F, voxelData1Input);
    cv::Mat magTMP2(this->sizeVoxelData, this->sizeVoxelData, CV_64F, voxelData2Input);

    cv::Mat convertedMat1;
    cv::Mat convertedMat2;

    magTMP1.convertTo(convertedMat1, CV_8U);
    magTMP2.convertTo(convertedMat2, CV_8U);

    cv::cvtColor(convertedMat1,convertedMat1,cv::COLOR_GRAY2BGR);
    cv::cvtColor(convertedMat2,convertedMat2,cv::COLOR_GRAY2BGR);
    cv::Ptr<cv::Feature2D> D;
    cv::BFMatcher M;
    //    D = cv::KAZE::create();
    switch(methodType){
        case 0:

            D = cv::AKAZE::create();
            M = cv::BFMatcher(cv::NORM_HAMMING);
            break;
        case 1:
            D = cv::KAZE::create();
            M = cv::BFMatcher(cv::NORM_L2);
            break;
        case 2:
            D = cv::ORB::create();
            M = cv::BFMatcher(cv::NORM_HAMMING);
            break;
        case 3:
            D = cv::BRISK::create();
            M = cv::BFMatcher(cv::NORM_HAMMING);
            break;
        case 4:
            D = cv::xfeatures2d::SURF::create();
            M = cv::BFMatcher(cv::NORM_L2);
            break;
        case 5:
            D = cv::SIFT::create();
            M = cv::BFMatcher(cv::NORM_L2);
            break;
    }


    std::vector<cv::KeyPoint> kpts1, kpts2, matched1, matched2;
    cv::Mat desc1, desc2;

    D->detectAndCompute(convertedMat1, cv::noArray(), kpts1, desc1);
    D->detectAndCompute(convertedMat2, cv::noArray(), kpts2, desc2);
    if(kpts1.empty() || kpts2.empty()){
        Eigen::Matrix4d returnMatrix =  Eigen::Matrix4d::Identity();
        returnMatrix(3,3) = -1;
        return returnMatrix;
    }

    cv::Mat J; //to save temporal images

    if(debug)
    {
        drawKeypoints(convertedMat1, kpts1, J);
        imshow("Keypoints 1", J);
//    imwrite("Keypoints1.jpg", J);
        drawKeypoints(convertedMat2, kpts2, J);
        imshow("Keypoints 2", J);
//    imwrite("Keypoints2.jpg", J);
        cv::waitKey(0);
    }




    std::vector< std::vector<cv::DMatch> > knn_matches;
    std::vector<cv::DMatch> good_matches;
    M.knnMatch(desc2, desc1, knn_matches, 2);
    const float nn_match_ratio = 0.9f;   // Nearest neighbor matching ratio


    //Use 2-nn matches to find correct keypoint matches
    for(size_t i = 0; i < knn_matches.size(); i++) {
        cv::DMatch nearest = knn_matches[i][0];
        double dist1 = knn_matches[i][0].distance;
        double dist2 = knn_matches[i][1].distance;
        if(dist1 < dist2*nn_match_ratio) {
            int new_i = static_cast<int>(matched1.size());
            matched1.push_back(kpts1[nearest.trainIdx]);
            matched2.push_back(kpts2[nearest.queryIdx]);
            good_matches.push_back(cv::DMatch(new_i, new_i, 0));
        }
    }
    if(debug){
        drawMatches(convertedMat1, matched1, convertedMat2, matched2, good_matches, J);
        imshow("Matches", J);
        cv::waitKey(0);
    }


//Use matches to compute the homography matrix
    std::vector<cv::Point2f> first;
    std::vector<cv::Point2f> second;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        first.push_back( matched1[i].pt);
        second.push_back( matched2[i].pt);
    }
//    cv::Mat H = findHomography(second, first, cv::RANSAC, 5.0);
    if(second.empty() || first.empty()){
        Eigen::Matrix4d returnMatrix =  Eigen::Matrix4d::Identity();
        returnMatrix(3,3) = -1;
        return returnMatrix;
    }

    cv::Mat HNew = cv::estimateAffine2D(second, first);
    if(HNew.empty()){
        Eigen::Matrix4d returnMatrix =  Eigen::Matrix4d::Identity();
        returnMatrix(3,3) = -1;
        return returnMatrix;
    }
    cv::Mat H = cv::Mat::eye(3,3,CV_64F);
    Eigen::Matrix3d testResultMatrix = Eigen::Matrix3d::Identity();
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 3; j++){
            H.at<double>(i,j) = HNew.at<double>(i,j);
            testResultMatrix(i,j) = H.at<double>(i,j);
        }
    }
    Eigen::Matrix3d onlyRotationMatrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d negativeTranslation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d positiveTranslation = Eigen::Matrix3d::Identity();

    onlyRotationMatrix.block<2,2>(0,0) = testResultMatrix.block<2,2>(0,0);
    negativeTranslation(0,2) =-this->sizeVoxelData/2.0;//-testResultMatrix(0,2);
    negativeTranslation(1,2) =-this->sizeVoxelData/2.0;//-testResultMatrix(1,2);
    positiveTranslation(0,2) =this->sizeVoxelData/2.0;//testResultMatrix(0,2);
    positiveTranslation(1,2) =this->sizeVoxelData/2.0;//testResultMatrix(1,2);


    Eigen::Matrix3d testHowGood = negativeTranslation*testResultMatrix*positiveTranslation;
    double x = testHowGood(0,2);
    double y = testHowGood(1,2);
    testHowGood(0,2) = -y*cellSize;
    testHowGood(1,2) = -x*cellSize;
    Eigen::Matrix4d returnMatrix = Eigen::Matrix4d::Identity();

    returnMatrix(0,3) = testHowGood(0,2);
    returnMatrix(1,3) = testHowGood(1,2);
    returnMatrix.block<2,2>(0,0) = testHowGood.block<2,2>(0,0);



    if(debug){
        cv::cvtColor(convertedMat1,convertedMat1,cv::COLOR_BGR2GRAY);
        cv::cvtColor(convertedMat2,convertedMat2,cv::COLOR_BGR2GRAY);

        //Apply the computed homography matrix to warp the second image
        cv::Mat Panorama(convertedMat1.rows, 2 * convertedMat1.cols,  CV_8U);
        warpPerspective(convertedMat2, Panorama, H, Panorama.size());


        Panorama.convertTo(Panorama,CV_64F,1.0/255.0);
        convertedMat1.convertTo(convertedMat1,CV_64F,1.0/255.0);
        convertedMat2.convertTo(convertedMat2,CV_64F,1.0/255.0);

        for(int i = 0; i < convertedMat1.rows; i++){
            for(int j = 0; j < convertedMat1.cols; j++){
                Panorama.at<double>(i,j) = (Panorama.at<double>(i,j)+convertedMat1.at<double>(i,j))/2.0;
            }
        }
        imshow("Panorama1", Panorama);
        for(int i = 0; i < convertedMat1.rows; i++){
            for(int j = 0; j < convertedMat1.cols; j++){
                Panorama.at<double>(i,j) = (convertedMat1.at<double>(i,j)+convertedMat2.at<double>(i,j))/2.0;
            }
        }
        cv::waitKey(0);
    }



    return returnMatrix;
}




Eigen::Matrix4d scanRegistrationClass::gmmRegistrationD2D(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                   pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                   Eigen::Matrix4d initialGuess, bool useInitialGuess, bool debug){
    std::lock_guard<std::mutex> guard(*this->gmmd2dMutex);

    //Convert scan to 2D vector xy points.
    std::vector<Eigen::Vector2d> currentScanVector(cloudFirstScan.points.size());
    std::vector<Eigen::Vector2d> referenceScanVector(cloudSecondScan.points.size());
    for(int i = 0 ; i <cloudFirstScan.points.size();i++){
        currentScanVector[i] = Eigen::Vector2d(cloudFirstScan.points[i].x,cloudFirstScan.points[i].y);
    }

    for(int i = 0 ; i <cloudSecondScan.points.size();i++){
        referenceScanVector[i] = Eigen::Vector2d(cloudFirstScan.points[i].x,cloudFirstScan.points[i].y);
    }
    // reference scan
    shared_ptr<GaussianMixturesModel<2>> reference_gmm;

    reference_gmm = bayesian_gmm_constructor(referenceScanVector, 50);
    reference_gmm->balance_covariances(0.05);
//    reference_gmm->plot_components_density(0, referenceScanVector, false);

    // current Scan
    shared_ptr<GaussianMixturesModel<2>> current_gmm;
    current_gmm = bayesian_gmm_constructor(currentScanVector, 50);
    current_gmm->balance_covariances(0.05);
//    current_gmm->plot_components_density(1, currentScanVector, false);


    shared_ptr<DistributionToDistribution2D> method(new DistributionToDistribution2D(reference_gmm, current_gmm));
    unique_ptr<CholeskyLineSearchNewtonMethod<3>> solver(new CholeskyLineSearchNewtonMethod<3>(method));
    solver->compute_optimum();
//    solver->plot_process(0, false, 0.01);
    Eigen::Vector3d t_opt = solver->get_optimal();
    Eigen::Matrix3d h_opt = solver->get_optimal_uncertainty();

//  convert t_opt to Matrix4d
    std::cout << t_opt << std::endl;
    Eigen::Matrix4d returnMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0,0,t_opt[2]);
    returnMatrix(0,3) = t_opt[0];
    returnMatrix(1,3) = t_opt[1];

    if(h_opt.isZero(0.0000001)){
        returnMatrix =  Eigen::Matrix4d::Identity();
        returnMatrix(3,3) = -1;
        return returnMatrix;

    }
    return returnMatrix;


}

Eigen::Matrix4d scanRegistrationClass::gmmRegistrationP2D(pcl::PointCloud<pcl::PointXYZ> &cloudFirstScan,
                                                          pcl::PointCloud<pcl::PointXYZ> &cloudSecondScan,
                                                          Eigen::Matrix4d initialGuess, bool useInitialGuess, bool debug){
    std::lock_guard<std::mutex> guard(*this->gmmp2dMutex);

    //Convert scan to 2D vector xy points.
    std::vector<Eigen::Vector2d> currentScanVector(cloudFirstScan.points.size());
    std::vector<Eigen::Vector2d> referenceScanVector(cloudSecondScan.points.size());
    for(int i = 0 ; i <cloudFirstScan.points.size();i++){
        currentScanVector[i] = Eigen::Vector2d(cloudFirstScan.points[i].x,cloudFirstScan.points[i].y);
    }

    for(int i = 0 ; i <cloudSecondScan.points.size();i++){
        referenceScanVector[i] = Eigen::Vector2d(cloudFirstScan.points[i].x,cloudFirstScan.points[i].y);
    }

    // reference scan
    shared_ptr<GaussianMixturesModel<2>> reference_gmm;

    reference_gmm = bayesian_gmm_constructor(referenceScanVector, 50);
    reference_gmm->balance_covariances(0.05);
//    reference_gmm->plot_components_density(0, referenceScanVector, false);
//    std::cout << "test3:" <<reference_gmm->k() << std::endl;

    // current Scan
//    shared_ptr<GaussianMixturesModel<2>> current_gmm;
//    current_gmm = bayesian_gmm_constructor(currentScanVector, 10);
//    current_gmm->balance_covariances(0.05);
//    current_gmm->plot_components_density(1, currentScanVector, true);


    shared_ptr<PointsToDistribution2D> method(
            new PointsToDistribution2D(reference_gmm, make_shared<std::vector<Eigen::Vector2d>>(currentScanVector)));

    unique_ptr<CholeskyLineSearchNewtonMethod<3>> solver(new CholeskyLineSearchNewtonMethod<3>(method));
    solver->compute_optimum();
//    solver->plot_process(0, false, 0.01);
    Eigen::Vector3d t_opt = solver->get_optimal();
    Eigen::Matrix3d h_opt = solver->get_optimal_uncertainty();

//  convert t_opt to Matrix4d
    Eigen::Matrix4d returnMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0,0,t_opt[2]);
    returnMatrix(0,3) = t_opt[0];
    returnMatrix(1,3) = t_opt[1];

    if(h_opt.isZero(0.0000001) ){
        returnMatrix =  Eigen::Matrix4d::Identity();
        returnMatrix(3,3) = -1;
        return returnMatrix;
    }
    return returnMatrix;
}
