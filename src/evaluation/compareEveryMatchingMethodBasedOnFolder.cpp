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
#include <opencv2/imgcodecs.hpp>
#include "std_srvs/SetBool.h"
#include <filesystem>
#include "scanRegistrationClass.h"
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




//1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform, 6: Our FMS 2D 7: initial guess
struct settingsMethodUsed {
    int whichMethod;
    bool initialGuess;
};


//1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform, 6: Our FMS 2D 7: initial guess
struct measurementResults {
    double calculationTime;
    double errorInRotation;
    double errorInDistance;
    int whichMethod;
    bool initialGuess;
    double overlapValue;
    int voxelSize;
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

Eigen::Matrix4d registrationOfDesiredMethod(pcl::PointCloud<pcl::PointXYZ> pclNotShifted,
                                            pcl::PointCloud<pcl::PointXYZ> pclShifted,
                                            pcl::PointCloud<pcl::PointXYZ> final, double voxelData[],
                                            double voxelDataShifted[],
                                            Eigen::Matrix4d initialGuess, double currentCellSize,
                                            int whichMethod, bool useInitialGuess,
                                            scanRegistrationClass &scanRegistrationObject) {

    //1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform, 6: Our FMS 2D 7: initial guess
    Eigen::Matrix4d returnMatrix;
    Eigen::Matrix3d covarianceEstimation;
    switch (whichMethod) {
        case 1:
            double fitnessScore;
            returnMatrix = scanRegistrationObject.generalizedIcpRegistration(pclNotShifted, pclShifted, final,
                                                                             fitnessScore, initialGuess).inverse();
            break;
        case 2:
            returnMatrix = scanRegistrationObject.super4PCSRegistration(pclNotShifted, pclShifted, initialGuess,
                                                                        useInitialGuess);
            break;
        case 3:
            returnMatrix = scanRegistrationObject.ndt_d2d_2d(pclNotShifted, pclShifted, initialGuess,
                                                             useInitialGuess).inverse();
            break;
        case 4:
            returnMatrix = scanRegistrationObject.ndt_p2d(pclNotShifted, pclShifted, initialGuess,
                                                          useInitialGuess).inverse();
            break;
        case 5:
            returnMatrix = scanRegistrationObject.registrationFourerMellin(voxelData, voxelDataShifted, currentCellSize,
                                                                           false);
            break;
        case 6:

            covarianceEstimation = Eigen::Matrix3d::Zero();
            // THRESHOLD_FOR_TRANSLATION_MATCHING 0.05 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben
            returnMatrix = slamToolsRos::registrationOfTwoVoxels(voxelData, voxelDataShifted,
                                                                 initialGuess,
                                                                 covarianceEstimation, useInitialGuess,
                                                                 useInitialGuess,
                                                                 currentCellSize,
                                                                 false, scanRegistrationObject,
                                                                 false,
                                                                 0.05);
            break;
        case 7:
            returnMatrix = scanRegistrationObject.registrationFeatureBased(voxelData, voxelDataShifted, currentCellSize,
                                                                           0, false);
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 8:
            returnMatrix = scanRegistrationObject.registrationFeatureBased(voxelData, voxelDataShifted, currentCellSize,
                                                                           1, false);
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 9:
            returnMatrix = scanRegistrationObject.registrationFeatureBased(voxelData, voxelDataShifted, currentCellSize,
                                                                           2, false);
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 10:
            returnMatrix = scanRegistrationObject.registrationFeatureBased(voxelData, voxelDataShifted, currentCellSize,
                                                                           3, false);
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;

    }

    return returnMatrix;

}

std::vector<measurementResults>
handleRegistrationsOfDirectory(const std::string &s,
                               std::vector<scanRegistrationClass> &scanRegistrationList,
                               std::vector<settingsMethodUsed> settingsMethodUsedList) {
    //load all the data
    std::vector<measurementResults> returnMeasurementsList;


    std::ifstream sizeVoxelGridFile(s + std::string("/settingsForThisDirectory.csv"));
//    std::cout << s + std::string("/settingsForThisDirectory.csv") << std::endl;

    std::vector<std::string> sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    int numberOfScans = std::stoi(sizeOfVoxelGridList[0]);
    sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    double sizeOfVoxelGrid = std::stod(sizeOfVoxelGridList[0]);
    //here are the voxel sizes in there
    sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    for (int currentNumberOfScanInDirectory = 0;
         currentNumberOfScanInDirectory < numberOfScans; currentNumberOfScanInDirectory++) {
//    for (int currentNumberOfScanInDirectory = 0;
//         currentNumberOfScanInDirectory < 1; currentNumberOfScanInDirectory++) {
        std::cout << "current number = " << currentNumberOfScanInDirectory << std::endl;
        std::vector<std::string> transformationMatrixList = getNextLineAndSplitIntoTokens(
                sizeVoxelGridFile);
        double overlapValue = std::stod(transformationMatrixList[0]);

        //get true transformation
        Eigen::Matrix4d gtTransformation;
        gtTransformation.setZero();

        for (int i = 0; i < 4; i++) {
            std::vector<std::string> transformationMatrixList = getNextLineAndSplitIntoTokens(
                    sizeVoxelGridFile);
            for (int j = 0; j < 4; j++) {
                gtTransformation(i, j) = std::stod(transformationMatrixList[j]);
            }
        }
        for (int currentVoxelSize = 0;
             currentVoxelSize < sizeOfVoxelGridList.size() - 1; currentVoxelSize++) {
            int ourN = std::stoi(sizeOfVoxelGridList[currentVoxelSize]);
            for (auto currentSettings: settingsMethodUsedList) {
                measurementResults tmpMeasurements;
                tmpMeasurements.initialGuess = currentSettings.initialGuess;
                tmpMeasurements.whichMethod = currentSettings.whichMethod;
                tmpMeasurements.voxelSize = std::stoi(sizeOfVoxelGridList[currentVoxelSize]);
                tmpMeasurements.overlapValue = overlapValue;

                //        std::cout << gtTransformation << std::endl;
                pcl::PointCloud<pcl::PointXYZ> final;

                pcl::PointCloud<pcl::PointXYZ> pclNotShifted;
                pcl::PointCloud<pcl::PointXYZ> pclShifted;


                pcl::io::loadPCDFile(
                        s + "/scan1_" + std::to_string(currentNumberOfScanInDirectory) + "_" +
                        sizeOfVoxelGridList[currentVoxelSize] +
                        "_pcl.pcd",
                        pclNotShifted);
                pcl::io::loadPCDFile(
                        s + "/scan2_" + std::to_string(currentNumberOfScanInDirectory) + "_" +
                        sizeOfVoxelGridList[currentVoxelSize] +
                        "_pcl.pcd",
                        pclShifted);
                double *voxelData;
                voxelData = (double *) malloc(sizeof(double) * ourN * ourN);

                std::ifstream voxelDataFile(s + "/scan1_" + std::to_string(currentNumberOfScanInDirectory) + "_" +
                                            std::to_string(ourN) +
                                            ".csv");
                for (int i = 0; i < ourN; i++) {
                    std::vector<std::string> voxelDataVector = getNextLineAndSplitIntoTokens(voxelDataFile);
                    for (int j = 0; j < ourN; j++) {
                        voxelData[i + ourN * j] = std::stod(voxelDataVector[j]);
                    }
                }
                voxelDataFile.close();
                double *voxelDataShifted;
                voxelDataShifted = (double *) malloc(sizeof(double) * ourN * ourN);
                std::ifstream voxelDataShiftedFile(
                        s + "/scan2_" + std::to_string(currentNumberOfScanInDirectory) + "_" +
                        std::to_string(ourN) +
                        ".csv");
//                std::cout << s + "/scan2_" + std::to_string(currentNumberOfScanInDirectory) + "_" +
//                             std::to_string(ourN) +
//                             ".csv" << std::endl;
                for (int i = 0; i < ourN; i++) {
                    std::vector<std::string> voxelDataShiftedVector = getNextLineAndSplitIntoTokens(
                            voxelDataShiftedFile);
                    for (int j = 0; j < ourN; j++) {
                        voxelDataShifted[i + ourN * j] = std::stod(voxelDataShiftedVector[j]);
                        //                std::cout <<voxelDataShifted[i + numberOfPoints * j] << std::endl;
                    }
                }
                voxelDataShiftedFile.close();
//                std::cout << s + "/scan1_" + std::to_string(currentNumberOfScanInDirectory) + "_"+
//                             sizeOfVoxelGridList[currentVoxelSize] +
//                             ".csv" << std::endl;

                //        Eigen::Matrix4d initialGuess = gtTransformation;
                Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();


                Eigen::Matrix4d estimatedTransformation;

                // for each test do X
                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

                estimatedTransformation = registrationOfDesiredMethod(pclNotShifted,
                                                                      pclShifted, final, voxelData, voxelDataShifted,
                                                                      initialGuess,
                                                                      sizeOfVoxelGrid /
                                                                      std::stod(sizeOfVoxelGridList[currentVoxelSize]),
                                                                      currentSettings.whichMethod,
                                                                      currentSettings.initialGuess,
                                                                      scanRegistrationList[currentVoxelSize]);


//                estimatedTransformation = scanRegistrationList[currentVoxelSize].generalizedIcpRegistration(pclNotShifted,
//                                                                                                            pclShifted,
//                                                                                                            final,
//                                                                                                            fitnessScoreX,
//                                                                                                            initialGuess);
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                double timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
                tmpMeasurements.calculationTime = timeToCalculate;

                //calculate the angle
                double angleGT = std::atan2(gtTransformation(1, 0), gtTransformation(0, 0));
                double angleEstimated = std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0));
                double angleDiff = abs(generalHelpfulTools::angleDiff(angleGT, angleEstimated));
                tmpMeasurements.errorInRotation = angleDiff;
                //calculate difference angle and take abs
                Eigen::Vector3d translationGT = gtTransformation.block<3, 1>(0, 3);
                Eigen::Vector3d translationEstimated = estimatedTransformation.block<3, 1>(0, 3);
                double errorDistance = (translationGT - translationEstimated).norm();
                tmpMeasurements.errorInDistance = errorDistance;

                free(voxelDataShifted);
                free(voxelData);

                if(estimatedTransformation(3,3)<0){
                    tmpMeasurements.errorInRotation = -1;
                    tmpMeasurements.errorInDistance = -1;
                }
//                std::cout << estimatedTransformation.inverse() << std::endl;
//                std::cout << estimatedTransformation << std::endl;
//                std::cout << gtTransformation << std::endl;
                returnMeasurementsList.push_back(tmpMeasurements);
            }
        }
        // ####################################################### Calculate error of Initial Guess  #######################################################

        measurementResults tmpMeasurements;
        Eigen::Matrix4d initialGuess = Eigen::Matrix4d::Identity();
        tmpMeasurements.calculationTime = 0;
        //calculate the angle
        double angleGT = std::atan2(gtTransformation(1, 0), gtTransformation(0, 0));
        double angleEstimated = std::atan2(initialGuess(1, 0), initialGuess(0, 0));
        double angleDiff = abs(generalHelpfulTools::angleDiff(angleGT, angleEstimated));
        tmpMeasurements.errorInRotation = angleDiff;
        //calculate difference angle and take abs

        Eigen::Vector3d translationGT = gtTransformation.block<3, 1>(0, 3);
        Eigen::Vector3d translationEstimated = initialGuess.block<3, 1>(0, 3);
        double errorDistance = (translationGT - translationEstimated).norm();
        tmpMeasurements.errorInDistance = errorDistance;
        tmpMeasurements.whichMethod = 11;
        returnMeasurementsList.push_back(tmpMeasurements);

    }
    return returnMeasurementsList;
}


// HOME_LOCATION WHICH_FOLDER_SHOULD_BE_SAVED numberOfScans
int main(int argc, char **argv) {

    //1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform, 6: Our FMS 2D 7: initial guess
    std::vector<settingsMethodUsed> settingsMethodUsedList;
    // fill in objects


    settingsMethodUsed tmpSettings;
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 1;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 2;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 3;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 4;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 5;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 6;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 6;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 7;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 8;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 9;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 10;
    settingsMethodUsedList.push_back(tmpSettings);

    std::string current_exec_name = argv[0]; // Name of the current exec program
    std::vector<std::string> all_args;
    if (argc < 3) {
        std::cout << "not the correct arguments given" << std::endl;
        exit(-1);
    } else {
        all_args.assign(argv + 1, argv + argc);
    }

    std::string HOME_LOCATION = all_args[0]; // "/home/tim-external/dataFolder/ValentinBunkerData/";
    std::string WHICH_FOLDER_SHOULD_BE_SAVED = all_args[1]; //"4_7_Bunker_range_30_5_RandomShifts1510/";
    int howManyScans = std::stoi(all_args[2]);
//    int thresholdUsage = std::stoi(all_args[3]); // 0 = only one Value 1 = threshold
    // get list of all the directories
    std::vector<std::string> listOfDirectories = get_directories(
            std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED));
    //Read in CSV file
    std::ifstream sizeVoxelGridFile(listOfDirectories[0] + std::string("/settingsForThisDirectory.csv"));
    std::vector<std::string> sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    sizeOfVoxelGridList = getNextLineAndSplitIntoTokens(sizeVoxelGridFile);
    sizeVoxelGridFile.close();
    std::vector<scanRegistrationClass> scanRegistrationList;
    for (int i = 0; i < sizeOfVoxelGridList.size() - 1; i++) {
        int sizeVoxel = std::stoi(sizeOfVoxelGridList[i]);
        scanRegistrationClass scanRegistrationObject(sizeVoxel, sizeVoxel / 2, sizeVoxel / 2, sizeVoxel / 2 - 1);
        scanRegistrationList.push_back(scanRegistrationObject);
    }


    std::vector<measurementResults> resultVector;


    for (int i = 0; i < listOfDirectories.size(); i++) {
//    for (int i = 0; i < 2; i++) {
        std::vector<measurementResults> tmpVector = handleRegistrationsOfDirectory(listOfDirectories[i],
                                                                                   scanRegistrationList,
                                                                                   settingsMethodUsedList);

        resultVector.insert(std::end(resultVector), std::begin(tmpVector), std::end(tmpVector));
        std::cout << "############################# we are at Number: " << i << std::endl;

    }

    std::ofstream resultsMatching;
    resultsMatching.open(std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "/results.csv");

    //save errors angle
    for (int i = 0; i < resultVector.size(); i++) {
        resultsMatching << resultVector[i].whichMethod;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].voxelSize;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].initialGuess;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].errorInDistance;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].errorInRotation;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].calculationTime;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].overlapValue;//time
        resultsMatching << "\n";//error

    }
    resultsMatching.close();

    return (0);
}


