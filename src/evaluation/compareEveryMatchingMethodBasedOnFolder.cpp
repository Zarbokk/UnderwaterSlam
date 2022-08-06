//
// Created by tim-external on 27.07.22.
//

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
//#include "ping360_sonar/SonarEcho.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "commonbluerovmsg/staterobotforevaluation.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include <hilbertMap.h>
#include <opencv2/imgcodecs.hpp>
#include "std_srvs/SetBool.h"
#include <filesystem>

//#define HOME_LOCATION "/home/tim-external/dataFolder/ValentinBunkerData/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "4_7_Bunker_range_30_5_RandomShifts/"

#define HOME_LOCATION "/home/tim-external/dataFolder/StPereDataset/"
#define WHICH_FOLDER_SHOULD_BE_SAVED "randomShifts/"



struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
};
//1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform(oneSize), 6: Our 32 FMS 2D, 7: Our 64 FMS 2D, 8: Our 128 FMS 2D, 9: Our 256 FMS 2D
struct measurementResults {
    std::vector<double> calculationTime;
    std::vector<double> errorInRotation;
    std::vector<double> errorInDistance;
};

std::vector<std::string> get_directories(const std::string &s) {
    std::vector<std::string> r;
    for (auto &p: std::filesystem::directory_iterator(s))
        if (p.is_directory())
            r.push_back(p.path().string());
    return r;
}

Eigen::Matrix4d registrationOfTwoVoxelsSOFFTFasterTest(double voxelData1[],
                                                       double voxelData2[],
                                                       Eigen::Matrix4d initialGuess,
                                                       bool useInitialAngle, bool useInitialTranslation,
                                                       int numberOfPoints, scanRegistrationClass &scanRegistrationObject,
                                                       const int dimensionOfVoxelData,
                                                       bool debug = false) {
    double goodGuessAlpha = -100;
    if (useInitialAngle) {
        goodGuessAlpha = std::atan2(initialGuess(1, 0),
                                    initialGuess(0, 0));
    }


    double estimatedAngle = scanRegistrationObject.sofftRegistrationVoxel2DRotationOnly(voxelData1,
                                                                                        voxelData2,
                                                                                        goodGuessAlpha, debug);


    Eigen::Matrix4d rotationMatrixTMP = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd tmpRotVec(estimatedAngle, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpMatrix3d = tmpRotVec.toRotationMatrix();
    rotationMatrixTMP.block<3, 3>(0, 0) = tmpMatrix3d;

    if (true) {
        for (int i = 0; i < 2; i++) {


            cv::Mat magTMP1(numberOfPoints, numberOfPoints, CV_64F, voxelData1);
            //add gaussian blur
            cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
            //            cv::imwrite("/home/tim-external/Documents/imreg_fmt/firstImage.jpg", magTMP1);

            cv::Mat magTMP2(numberOfPoints, numberOfPoints, CV_64F, voxelData2);
            //add gaussian blur
            cv::GaussianBlur(magTMP2, magTMP2, cv::Size(9, 9), 0);

            cv::Point2f pc(magTMP2.cols / 2., magTMP2.rows / 2.);
            cv::Mat r = cv::getRotationMatrix2D(pc, -estimatedAngle * 180.0 / M_PI, 1.0);

            cv::warpAffine(magTMP2, magTMP2, r, magTMP2.size()); // what size I should use?
        }
//            cv::imwrite("/home/tim-external/Documents/imreg_fmt/secondImage.jpg", magTMP2);

//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
    }

    if (debug) {
        std::ofstream myFile3, myFile6;
        myFile3.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW1.csv");
        myFile6.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW2.csv");
        for (int i = 0; i < numberOfPoints; i++) {
            for (int j = 0; j < numberOfPoints; j++) {

                myFile3 << voxelData1[j + numberOfPoints * i]; // imaginary part
                myFile3 << "\n";
                myFile6 << voxelData2[j + numberOfPoints * i]; // imaginary part
                myFile6 << "\n";
            }
        }
        myFile3.close();
        myFile6.close();
    }
    double fitnessX = 0;
    double fitnessY = 0;
    Eigen::Vector2d translation = scanRegistrationObject.sofftRegistrationVoxel2DTranslation(voxelData1,
                                                                                             voxelData2,
                                                                                             fitnessX,
                                                                                             fitnessY,
                                                                                             (double) dimensionOfVoxelData /
                                                                                             (double) numberOfPoints,
                                                                                             initialGuess.block<3, 1>(
                                                                                                     0, 3),
                                                                                             useInitialTranslation,
                                                                                             debug);

    Eigen::Matrix4d estimatedRotationScans = Eigen::Matrix4d::Identity();//from second scan to first
    //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd rotation_vectorTMP(estimatedAngle, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpRotMatrix3d = rotation_vectorTMP.toRotationMatrix();
    estimatedRotationScans.block<3, 3>(0, 0) = tmpRotMatrix3d;
    estimatedRotationScans(0, 3) = translation.x();
    estimatedRotationScans(1, 3) = translation.y();
    estimatedRotationScans(2, 3) = 0;
    estimatedRotationScans(3, 3) = 1;

    if (debug) {
        std::ofstream myFile1, myFile2;
        myFile1.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel1.csv");
        myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel2.csv");
        for (int i = 0; i < numberOfPoints; i++) {
            for (int j = 0; j < numberOfPoints; j++) {
                myFile1 << voxelData1[j + numberOfPoints * i]; // real part
                myFile1 << "\n";
                myFile2 << voxelData2[j + numberOfPoints * i]; // imaginary part
                myFile2 << "\n";
            }
        }
        myFile1.close();
        myFile2.close();
    }

    return estimatedRotationScans;//should be the transformation matrix from 1 to 2
}

std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream &str) {
    std::vector<std::string> result;
    std::string line;
    std::getline(str, line);

    std::stringstream lineStream(line);
    std::string cell;

    while (std::getline(lineStream, cell, ',')) {
        result.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty()) {
        // If there was a trailing comma then add an empty element.
        result.push_back("");
    }
    return result;
}


std::vector<measurementResults>
handleRegistrationsOfDirectory(const std::string &s, scanRegistrationClass &scanRegistrationObject32,
                               scanRegistrationClass &scanRegistrationObject64,
                               scanRegistrationClass &scanRegistrationObject128,
                               scanRegistrationClass &scanRegistrationObject256) {
    //load all the data
    std::vector<measurementResults> returnMeasurementsList;


    std::ifstream sizeVoxelGridFile(s + std::string("/dimensionVoxelData.csv"));
    std::cout << s + std::string("/dimensionVoxelData.csv") << std::endl;

    std::vector<std::string> sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    double sizeOfVoxelGrid = std::stod(sizeOfVoxelGridList[0]);


    for (int currentNumberOfScanInDirectory = 0;
         currentNumberOfScanInDirectory < 10; currentNumberOfScanInDirectory++) {
        measurementResults tmpMeasurements;


        //get true transformation
        std::ifstream transformationFile(
                s + std::string("/" + std::to_string(currentNumberOfScanInDirectory) + "_transformation.csv"));
        Eigen::Matrix4d gtTransformation;
        gtTransformation.setZero();
        for (int i = 0; i < 4; i++) {
            std::vector<std::string> transformationMatrixList = getNextLineAndSplitIntoTokens(transformationFile);
            for (int j = 0; j < 4; j++) {
                gtTransformation(i, j) = std::stod(transformationMatrixList[j]);
            }
        }
        std::cout << gtTransformation << std::endl;
        pcl::PointCloud<pcl::PointXYZ> final;

        pcl::PointCloud<pcl::PointXYZ> oneValuePCL;
        pcl::PointCloud<pcl::PointXYZ> oneValuePCLShifted;

        pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_OneValue.ply", oneValuePCL);
        pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_OneValueShifted.ply",
                             oneValuePCLShifted);

        pcl::PointCloud<pcl::PointXYZ> ThresholdPCL;
        pcl::PointCloud<pcl::PointXYZ> ThresholdPCLShifted;

        pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_Threshold.ply",
                             ThresholdPCL);
        pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_ThresholdShifted.ply",
                             ThresholdPCLShifted);


        std::chrono::steady_clock::time_point begin;
        std::chrono::steady_clock::time_point end;
        Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d estimatedTransformation;
        double fitnessScoreX, fitnessScoreY;


        //GICP
        begin = std::chrono::steady_clock::now();
        estimatedTransformation = scanRegistrationObject32.generalizedIcpRegistration(oneValuePCL,
                                                                                           oneValuePCLShifted,
                                                                                           final,
                                                                                           fitnessScoreX,
                                                                                           initialGuess);
        end = std::chrono::steady_clock::now();
        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
        tmpMeasurements.calculationTime.push_back(timeToCalculate);

        //calculate the angle
        double angleGT = std::atan2(gtTransformation(1, 0), gtTransformation(0, 0));
        double angleEstimated = std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0));
        double angleDiff = abs(generalHelpfulTools::angleDiff(angleGT, angleEstimated));
        tmpMeasurements.errorInRotation.push_back(angleDiff);
        //calculate difference angle and take abs
        Eigen::Vector3d translationGT = gtTransformation.block<3, 1>(0, 3);
        Eigen::Vector3d translationEstimated = estimatedTransformation.block<3, 1>(0, 3);
        double errorDistance = (translationGT - translationEstimated).norm();
        tmpMeasurements.errorInDistance.push_back(errorDistance);


        //SUPER4PCS
        begin = std::chrono::steady_clock::now();
        estimatedTransformation = scanRegistrationObject32.super4PCSRegistration(oneValuePCL,
                                                                                      oneValuePCLShifted,
                                                                                      initialGuess,
                                                                                      true, false);

        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
        tmpMeasurements.calculationTime.push_back(timeToCalculate);

        //calculate the angle
        angleGT = std::atan2(gtTransformation(1, 0), gtTransformation(0, 0));
        angleEstimated = std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0));
        angleDiff = abs(generalHelpfulTools::angleDiff(angleGT, angleEstimated));
        tmpMeasurements.errorInRotation.push_back(angleDiff);
        //calculate difference angle and take abs
        translationGT = gtTransformation.block<3, 1>(0, 3);
        translationEstimated = estimatedTransformation.block<3, 1>(0, 3);
        errorDistance = (translationGT - translationEstimated).norm();
        tmpMeasurements.errorInDistance.push_back(errorDistance);

        //NDT D2D 2D,
        begin = std::chrono::steady_clock::now();
        estimatedTransformation = scanRegistrationObject32.ndt_d2d_2d(oneValuePCL,
                                                                           oneValuePCLShifted,
                                                                           initialGuess,
                                                                           true);

        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
        tmpMeasurements.calculationTime.push_back(timeToCalculate);

        //calculate the angle
        angleGT = std::atan2(gtTransformation(1, 0), gtTransformation(0, 0));
        angleEstimated = std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0));
        angleDiff = abs(generalHelpfulTools::angleDiff(angleGT, angleEstimated));
        tmpMeasurements.errorInRotation.push_back(angleDiff);
        //calculate difference angle and take abs
        translationGT = gtTransformation.block<3, 1>(0, 3);
        translationEstimated = estimatedTransformation.block<3, 1>(0, 3);
        errorDistance = (translationGT - translationEstimated).norm();
        tmpMeasurements.errorInDistance.push_back(errorDistance);

        //NDT P2D
        begin = std::chrono::steady_clock::now();
        estimatedTransformation = scanRegistrationObject32.ndt_p2d(oneValuePCL,
                                                                        oneValuePCLShifted,
                                                                        initialGuess,
                                                                        true);

        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
        tmpMeasurements.calculationTime.push_back(timeToCalculate);

        //calculate the angle
        angleGT = std::atan2(gtTransformation(1, 0), gtTransformation(0, 0));
        angleEstimated = std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0));
        angleDiff = abs(generalHelpfulTools::angleDiff(angleGT, angleEstimated));
        tmpMeasurements.errorInRotation.push_back(angleDiff);
        //calculate difference angle and take abs
        translationGT = gtTransformation.block<3, 1>(0, 3);
        translationEstimated = estimatedTransformation.block<3, 1>(0, 3);
        errorDistance = (translationGT - translationEstimated).norm();
        tmpMeasurements.errorInDistance.push_back(errorDistance);
        // FourierMellinTransform(oneSize) (currently missing)


        tmpMeasurements.calculationTime.push_back(0);
        tmpMeasurements.errorInRotation.push_back(0);
        tmpMeasurements.errorInDistance.push_back(0);


        // Our 32 FMS 2D, 7: Our 64 FMS 2D, 8: Our 128 FMS 2D, 9: Our 256 FMS 2D
        for (int numberOfPoints = 32; numberOfPoints <= 256; numberOfPoints = numberOfPoints * 2) {
            scanRegistrationClass scanRegistrationObject(numberOfPoints, numberOfPoints / 2, numberOfPoints / 2,
                                                         numberOfPoints / 2 - 1);

            double *voxelData;
            voxelData = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);

            std::ifstream voxelDataFile(s + std::string(
                    "/" + std::to_string(currentNumberOfScanInDirectory) + "intensity" +
                    std::to_string(numberOfPoints) +
                    ".csv"));
            for (int i = 0; i < numberOfPoints; i++) {
                std::vector<std::string> voxelDataVector = getNextLineAndSplitIntoTokens(voxelDataFile);
                for (int j = 0; j < numberOfPoints; j++) {
                    voxelData[i + numberOfPoints * j] = std::stod(voxelDataVector[j]);
                }
            }

            double *voxelDataShifted;
            voxelDataShifted = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
            std::ifstream voxelDataShiftedFile(s + std::string(
                    "/" + std::to_string(currentNumberOfScanInDirectory) + "intensityShifted" +
                    std::to_string(numberOfPoints) + ".csv"));
            for (int i = 0; i < numberOfPoints; i++) {
                std::vector<std::string> voxelDataShiftedVector = getNextLineAndSplitIntoTokens(voxelDataShiftedFile);
                for (int j = 0; j < numberOfPoints; j++) {
                    voxelDataShifted[i + numberOfPoints * j] = std::stod(voxelDataShiftedVector[j]);
                    //                std::cout <<voxelDataShifted[i + numberOfPoints * j] << std::endl;
                }
            }


            //        std::cout << "start registration of: " << numberOfPoints << std::endl;
            //registration can be done here
            begin = std::chrono::steady_clock::now();
            if(numberOfPoints == 32){
                estimatedTransformation = registrationOfTwoVoxelsSOFFTFasterTest(voxelDataShifted, voxelData,
                                                                                 Eigen::Matrix4d::Identity(), true,
                                                                                 true, numberOfPoints,
                                                                                 scanRegistrationObject32,
                                                                                 sizeOfVoxelGrid, true);
            }else{
                if(numberOfPoints == 64){
                        estimatedTransformation = registrationOfTwoVoxelsSOFFTFasterTest(voxelDataShifted, voxelData,
                                                                                         Eigen::Matrix4d::Identity(), true,
                                                                                         true, numberOfPoints,
                                                                                         scanRegistrationObject64,
                                                                                         sizeOfVoxelGrid, true);
                }else{
                    if(numberOfPoints == 128){
                        estimatedTransformation = registrationOfTwoVoxelsSOFFTFasterTest(voxelDataShifted, voxelData,
                                                                                         Eigen::Matrix4d::Identity(), true,
                                                                                         true, numberOfPoints,
                                                                                         scanRegistrationObject128,
                                                                                         sizeOfVoxelGrid, true);
                    }else{
                        if(numberOfPoints == 256){
                            estimatedTransformation = registrationOfTwoVoxelsSOFFTFasterTest(voxelDataShifted, voxelData,
                                                                                             Eigen::Matrix4d::Identity(), true,
                                                                                             true, numberOfPoints,
                                                                                             scanRegistrationObject256,
                                                                                             sizeOfVoxelGrid, true);
                        }else{
                            std::cout << "should never happen" << std::endl;
                            exit(-1);
                        }
                    }
                }
            }

            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            std::cout << timeToCalculate << std::endl;
            tmpMeasurements.calculationTime.push_back(timeToCalculate);

            //calculate the angle
            angleGT = std::atan2(gtTransformation(1, 0), gtTransformation(0, 0));
            angleEstimated = std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0));
            angleDiff = abs(generalHelpfulTools::angleDiff(angleGT, angleEstimated));
            tmpMeasurements.errorInRotation.push_back(angleDiff);
            //calculate difference angle and take abs
            translationGT = gtTransformation.block<3, 1>(0, 3);
            translationEstimated = estimatedTransformation.block<3, 1>(0, 3);
            errorDistance = (translationGT - translationEstimated).norm();
            tmpMeasurements.errorInDistance.push_back(errorDistance);
            free(voxelDataShifted);
            free(voxelData);

            //        std::cout << "estimated Transformation " << resultTransformation << std::endl;
            //        std::cout << "test123" << std::endl;
            scanRegistrationObject.~scanRegistrationClass();
        }

        returnMeasurementsList.push_back(tmpMeasurements);
    }
    return returnMeasurementsList;
}


int main(int argc, char **argv) {
    //get list of all the directories
    std::vector<std::string> listOfDirectories = get_directories(
            std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED));
    scanRegistrationClass scanRegistrationObject32(32, 32 / 2, 32 / 2, 32 / 2 - 1);
    scanRegistrationClass scanRegistrationObject64(64, 64 / 2, 64 / 2, 64 / 2 - 1);
    scanRegistrationClass scanRegistrationObject128(128, 128 / 2, 128 / 2, 128 / 2 - 1);
    scanRegistrationClass scanRegistrationObject256(256, 256 / 2, 256 / 2, 256 / 2 - 1);
    std::vector<measurementResults> resultVector;
    for (int i = 0; i < listOfDirectories.size(); i++) {

        std::vector<measurementResults> tmpVector = handleRegistrationsOfDirectory(listOfDirectories[i],
                                                                                   scanRegistrationObject32,
                                                                                   scanRegistrationObject64,
                                                                                   scanRegistrationObject128,
                                                                                   scanRegistrationObject256);

        resultVector.insert(std::end(resultVector), std::begin(tmpVector), std::end(tmpVector));
        std::cout << "############################# we are at Number: " << i << std::endl;

    }

    std::ofstream errorAngleFile;
    errorAngleFile.open(std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "errorAngle.csv");

    //save errors angle
    for (int i = 0; i < resultVector.size(); i++) {
        for (int j = 0; j < resultVector[i].errorInRotation.size(); j++) {
            errorAngleFile << resultVector[i].errorInRotation[j];//time
            if (j != resultVector[i].errorInRotation.size() - 1) {
                errorAngleFile << ",";
            }
        }
        errorAngleFile << "\n";//error
    }





    //save times
    std::ofstream calculationTimeFile;
    calculationTimeFile.open(
            std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "calculationTime.csv");

    //save errors angle
    for (int i = 0; i < resultVector.size(); i++) {
        for (int j = 0; j < resultVector[i].calculationTime.size(); j++) {
            calculationTimeFile << resultVector[i].calculationTime[j];//time
            if (j != resultVector[i].calculationTime.size() - 1) {
                calculationTimeFile << ",";
            }
        }
        calculationTimeFile << "\n";//error
    }
    //save errors distance

    std::ofstream errorDistanceFile;
    errorDistanceFile.open(
            std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "errorDistance.csv");

    //save errors angle
    for (int i = 0; i < resultVector.size(); i++) {
        for (int j = 0; j < resultVector[i].errorInDistance.size(); j++) {
            errorDistanceFile << resultVector[i].errorInDistance[j];//time
            if (j != resultVector[i].errorInDistance.size() - 1) {
                errorDistanceFile << ",";
            }
        }
        errorDistanceFile << "\n";//error
    }


    return (0);
}


