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
//#define WHICH_FOLDER_SHOULD_BE_SAVED "4_7_Bunker_range_30_5_RandomShifts1510/"

//#define HOME_LOCATION "/home/tim-external/dataFolder/ValentinBunkerData/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "4_7_Bunker_range_30_5_RandomShifts105/"
//
//#define HOME_LOCATION "/home/tim-external/dataFolder/ValentinBunkerData/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "4_7_Bunker_range_15_1_RandomShifts1510/"
//
//#define HOME_LOCATION "/home/tim-external/dataFolder/ValentinBunkerData/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "4_7_Bunker_range_15_1_RandomShifts105/"

//#define HOME_LOCATION "/home/tim-external/dataFolder/StPereDataset/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "randomShifts/"

//#define HOME_LOCATION "/home/tim-external/dataFolder/StPereDataset/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "randomShifts105/"








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
                               scanRegistrationClass &scanRegistrationObject256,
                               int howManyScans,
                               int thresholdUsage) {
    //load all the data
    std::vector<measurementResults> returnMeasurementsList;


    std::ifstream sizeVoxelGridFile(s + std::string("/dimensionVoxelData.csv"));
    std::cout << s + std::string("/dimensionVoxelData.csv") << std::endl;

    std::vector<std::string> sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    double sizeOfVoxelGrid = std::stod(sizeOfVoxelGridList[0]);


    for (int currentNumberOfScanInDirectory = 0;
         currentNumberOfScanInDirectory < howManyScans; currentNumberOfScanInDirectory++) {
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

        pcl::PointCloud<pcl::PointXYZ> pclNotShifted;
        pcl::PointCloud<pcl::PointXYZ> pclShifted;


        if (thresholdUsage) {
            pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_Threshold.ply",
                                 pclNotShifted);
            pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_ThresholdShifted.ply",
                                 pclShifted);
        } else {
            pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_OneValue.ply",
                                 pclNotShifted);
            pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_OneValueShifted.ply",
                                 pclShifted);
        }

//        pcl::PointCloud<pcl::PointXYZ> ThresholdPCL;
//        pcl::PointCloud<pcl::PointXYZ> ThresholdPCLShifted;
//
//        pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_Threshold.ply",
//                             ThresholdPCL);
//        pcl::io::loadPLYFile(s + "/" + std::to_string(currentNumberOfScanInDirectory) + "_ThresholdShifted.ply",
//                             ThresholdPCLShifted);


        std::chrono::steady_clock::time_point begin;
        std::chrono::steady_clock::time_point end;
        Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d estimatedTransformation;
        double fitnessScoreX, fitnessScoreY;


        //GICP
        begin = std::chrono::steady_clock::now();
        estimatedTransformation = scanRegistrationObject32.generalizedIcpRegistration(pclNotShifted,
                                                                                      pclShifted,
                                                                                      final,
                                                                                      fitnessScoreX,
                                                                                      initialGuess);
        end = std::chrono::steady_clock::now();
        double timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
//        std::cout << estimatedTransformation << std::endl;
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
        estimatedTransformation = scanRegistrationObject32.super4PCSRegistration(pclNotShifted,
                                                                                 pclShifted,
                                                                                 initialGuess,
                                                                                 true, false);

        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
        tmpMeasurements.calculationTime.push_back(timeToCalculate);
//        std::cout << estimatedTransformation << std::endl;

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
        estimatedTransformation = scanRegistrationObject32.ndt_d2d_2d(pclNotShifted,
                                                                      pclShifted,
                                                                      initialGuess,
                                                                      true);

        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
        tmpMeasurements.calculationTime.push_back(timeToCalculate);
        std::cout << estimatedTransformation << std::endl;

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
        estimatedTransformation = scanRegistrationObject32.ndt_p2d(pclNotShifted,
                                                                   pclShifted,
                                                                   initialGuess,
                                                                   true);

        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        std::cout << timeToCalculate << std::endl;
        tmpMeasurements.calculationTime.push_back(timeToCalculate);
//        std::cout << estimatedTransformation << std::endl;

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
        for (int boolState = 0; boolState < 2; boolState++) {
            bool useInitialAngle = boolState == 0;
            bool useInitialTranslation = boolState == 0;


            // Our 32 FMS 2D, 7: Our 64 FMS 2D, 8: Our 128 FMS 2D, 9: Our 256 FMS 2D
            for (int numberOfPoints = 32; numberOfPoints <= 256; numberOfPoints = numberOfPoints * 2) {
                //            scanRegistrationClass scanRegistrationObject(numberOfPoints, numberOfPoints / 2, numberOfPoints / 2,
                //                                                         numberOfPoints / 2 - 1);

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
                    std::vector<std::string> voxelDataShiftedVector = getNextLineAndSplitIntoTokens(
                            voxelDataShiftedFile);
                    for (int j = 0; j < numberOfPoints; j++) {
                        voxelDataShifted[i + numberOfPoints * j] = std::stod(voxelDataShiftedVector[j]);
                        //                std::cout <<voxelDataShifted[i + numberOfPoints * j] << std::endl;
                    }
                }

                bool debug = false;


                //        std::cout << "start registration of: " << numberOfPoints << std::endl;
                //registration can be done here
                begin = std::chrono::steady_clock::now();
                if (numberOfPoints == 32) {

                    estimatedTransformation = scanRegistrationObject32.registrationOfTwoVoxelsSOFFTFast(
                            voxelDataShifted, voxelData,
                            Eigen::Matrix4d::Identity(), useInitialAngle,
                            useInitialTranslation, (double) sizeOfVoxelGrid /
                                                   (double) numberOfPoints, true, debug);
                } else {
                    if (numberOfPoints == 64) {
                        estimatedTransformation = scanRegistrationObject64.registrationOfTwoVoxelsSOFFTFast(
                                voxelDataShifted, voxelData,
                                Eigen::Matrix4d::Identity(), useInitialAngle,
                                useInitialTranslation, (double) sizeOfVoxelGrid /
                                                       (double) numberOfPoints, true, debug);
                    } else {
                        if (numberOfPoints == 128) {
                            estimatedTransformation = scanRegistrationObject128.registrationOfTwoVoxelsSOFFTFast(
                                    voxelDataShifted, voxelData,
                                    Eigen::Matrix4d::Identity(), useInitialAngle,
                                    useInitialTranslation, (double) sizeOfVoxelGrid /
                                                           (double) numberOfPoints, true, debug);
                        } else {
                            if (numberOfPoints == 256) {
                                estimatedTransformation = scanRegistrationObject256.registrationOfTwoVoxelsSOFFTFast(
                                        voxelDataShifted, voxelData,
                                        Eigen::Matrix4d::Identity(), useInitialAngle,
                                        useInitialTranslation, (double) sizeOfVoxelGrid /
                                                               (double) numberOfPoints, true, debug);
                            } else {
                                std::cout << "should never happen" << std::endl;
                                exit(-1);
                            }
                        }
                    }
                }
//                std::cout << estimatedTransformation << std::endl;
                end = std::chrono::steady_clock::now();
                timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
                std::cout << timeToCalculate << std::endl;
                tmpMeasurements.calculationTime.push_back(timeToCalculate);
//                std::cout << estimatedTransformation << std::endl;

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
                //            scanRegistrationObject.~scanRegistrationClass();
                //        std::cout << "estimated Transformation " << resultTransformation << std::endl;
                //        std::cout << "test123" << std::endl;
            }
        }
        returnMeasurementsList.push_back(tmpMeasurements);
    }
    return returnMeasurementsList;
}

// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle15 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle20 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle25 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle30 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle35 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle40 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle45 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle50 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle55 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle60 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle65 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle70 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle75 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle80 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle85 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle90 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle95 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle100 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle105 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle110 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle115 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/ValentinBunkerData/ 4_7_Bunker_range_30_5_OnlyAngle120 1


// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/StPereDataset/ onlyAngles15 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/StPereDataset/ onlyAngles20 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/StPereDataset/ onlyAngles25 1
// rosrun underwaterslam compareEveryMatchingMethodBasedOnFolder /home/tim-external/dataFolder/StPereDataset/ onlyAngles30 1



// HOME_LOCATION WHICH_FOLDER_SHOULD_BE_SAVED numberOfScans
int main(int argc, char **argv) {
    std::string current_exec_name = argv[0]; // Name of the current exec program
    std::vector<std::string> all_args;
    if (argc < 4) {
        std::cout << "not the correct arguments given" << std::endl;
        exit(-1);
    } else {
        all_args.assign(argv + 1, argv + argc);
    }

    std::string HOME_LOCATION = all_args[0];//"/home/tim-external/dataFolder/ValentinBunkerData/";
    std::string WHICH_FOLDER_SHOULD_BE_SAVED = all_args[1];//"4_7_Bunker_range_30_5_RandomShifts1510/";
    int howManyScans = std::stoi(all_args[2]);
    int thresholdUsage = std::stoi(all_args[3]);// 0 = only one Value 1 = threshold
    //get list of all the directories
    std::vector<std::string> listOfDirectories = get_directories(
            std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED));


    std::vector<measurementResults> resultVector;
    scanRegistrationClass scanRegistrationObject32(32, 32 / 2, 32 / 2, 32 / 2 - 1);
    scanRegistrationClass scanRegistrationObject64(64, 64 / 2, 64 / 2, 64 / 2 - 1);
    scanRegistrationClass scanRegistrationObject128(128, 128 / 2, 128 / 2, 128 / 2 - 1);
    scanRegistrationClass scanRegistrationObject256(256, 256 / 2, 256 / 2, 256 / 2 - 1);


    for (int i = 0; i < listOfDirectories.size(); i++) {

        std::vector<measurementResults> tmpVector = handleRegistrationsOfDirectory(listOfDirectories[i],
                                                                                   scanRegistrationObject32,
                                                                                   scanRegistrationObject64,
                                                                                   scanRegistrationObject128,
                                                                                   scanRegistrationObject256,
                                                                                   howManyScans,
                                                                                   thresholdUsage);

        resultVector.insert(std::end(resultVector), std::begin(tmpVector), std::end(tmpVector));
        std::cout << "############################# we are at Number: " << i << std::endl;

    }

    std::ofstream errorAngleFile;
    errorAngleFile.open(std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "/errorAngle" +
                        std::to_string(thresholdUsage) + ".csv");

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
            std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "/calculationTime" +
            std::to_string(thresholdUsage) + ".csv");

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
            std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "/errorDistance" +
            std::to_string(thresholdUsage) + ".csv");

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


