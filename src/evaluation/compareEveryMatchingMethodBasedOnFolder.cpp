//
// Created by tim-external on 27.07.22.
//


#include "generalHelpfulTools.h"

#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include "future"
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

// /home/tim-external/dataFolder/journalPaperDatasets/newDatasetsCreation/ speedTestsValentin/ 10


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
    double xGT;
    double yGT;
    double angleGT;
};


// GLOBAL
std::vector<scanRegistrationClass *> scanRegistrationList;
std::vector<settingsMethodUsed> settingsMethodUsedList;


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
                                            scanRegistrationClass *scanRegistrationObject,double &timeToCalculate) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    //1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform,
    //6: Our FMS 2D 7: FMS hamming 8: FMS none
    //9: Feature0 10: Feature1  11: Feature2 12: Feature3 13: Feature4 14: Feature5
    //15: gmmRegistrationD2D 16: gmmRegistrationP2D
    Eigen::Matrix4d returnMatrix;
    Eigen::Matrix3d covarianceEstimation;
    switch (whichMethod) {
        case 1:
            double fitnessScore;
            returnMatrix = scanRegistrationObject->generalizedIcpRegistration(pclNotShifted, pclShifted, final,
                                                                              fitnessScore, initialGuess).inverse();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 2:
//            returnMatrix = scanRegistrationObject.super4PCSRegistration(pclNotShifted, pclShifted, initialGuess,
//                                                                        useInitialGuess);
            std::cout << "super 4PCS not implemented anymore" << std::endl;
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 3:
            returnMatrix = scanRegistrationObject->ndt_d2d_2d(pclNotShifted, pclShifted, initialGuess,
                                                              useInitialGuess).inverse();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 4:
            returnMatrix = scanRegistrationObject->ndt_p2d(pclNotShifted, pclShifted, initialGuess,
                                                           useInitialGuess).inverse();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 5:
            returnMatrix = scanRegistrationObject->registrationFourerMellin(voxelData, voxelDataShifted,
                                                                            currentCellSize,
                                                                            false);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 6:

            covarianceEstimation = Eigen::Matrix3d::Zero();
            // THRESHOLD_FOR_TRANSLATION_MATCHING 0.05 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben
            if (useInitialGuess) {
                returnMatrix = scanRegistrationObject->registrationOfTwoVoxelsSOFFTFast(voxelData, 1,
                                                                                        voxelDataShifted, 1,
                                                                                        initialGuess,
                                                                                        covarianceEstimation,
                                                                                        currentCellSize,timeToCalculate);

            } else {
                scanRegistrationObject->registrationOfTwoVoxelsSOFFTAllSoluations(voxelData, 1,
                                                                                  voxelDataShifted, 1,
                                                                                  initialGuess,
                                                                                  covarianceEstimation,
                                                                                  currentCellSize,timeToCalculate);
            }
//            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 7:
            covarianceEstimation = Eigen::Matrix3d::Zero();
            // THRESHOLD_FOR_TRANSLATION_MATCHING 0.05 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben

            if (useInitialGuess) {
                returnMatrix = scanRegistrationObject->registrationOfTwoVoxelsSOFFTFast(voxelData, 1,
                                                                                        voxelDataShifted, 1,
                                                                                        initialGuess,
                                                                                        covarianceEstimation,
                                                                                        currentCellSize,timeToCalculate);

            } else {
                scanRegistrationObject->registrationOfTwoVoxelsSOFFTAllSoluations(voxelData, 1,
                                                                                  voxelDataShifted, 1,
                                                                                  initialGuess,
                                                                                  covarianceEstimation,
                                                                                  currentCellSize,timeToCalculate);
            }
//            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 8:
            covarianceEstimation = Eigen::Matrix3d::Zero();
            // THRESHOLD_FOR_TRANSLATION_MATCHING 0.05 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben
            if (useInitialGuess) {
                returnMatrix = scanRegistrationObject->registrationOfTwoVoxelsSOFFTFast(voxelData, 1,
                                                                                        voxelDataShifted, 1,
                                                                                        initialGuess,
                                                                                        covarianceEstimation,
                                                                                        currentCellSize,timeToCalculate);

            } else {
                scanRegistrationObject->registrationOfTwoVoxelsSOFFTAllSoluations(voxelData, 1,
                                                                                  voxelDataShifted, 1,
                                                                                  initialGuess,
                                                                                  covarianceEstimation,
                                                                                  currentCellSize,timeToCalculate);
            }
//           std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            break;
        case 9:
            returnMatrix = scanRegistrationObject->registrationFeatureBased(voxelData, voxelDataShifted,
                                                                            currentCellSize,
                                                                            0, false);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 10:
            returnMatrix = scanRegistrationObject->registrationFeatureBased(voxelData, voxelDataShifted,
                                                                            currentCellSize,
                                                                            1, false);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 11:
            returnMatrix = scanRegistrationObject->registrationFeatureBased(voxelData, voxelDataShifted,
                                                                            currentCellSize,
                                                                            2, false);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 12:
            returnMatrix = scanRegistrationObject->registrationFeatureBased(voxelData, voxelDataShifted,
                                                                            currentCellSize,
                                                                            3, false);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 13:
            returnMatrix = scanRegistrationObject->registrationFeatureBased(voxelData, voxelDataShifted,
                                                                            currentCellSize,
                                                                            4, false);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 14:
            returnMatrix = scanRegistrationObject->registrationFeatureBased(voxelData, voxelDataShifted,
                                                                            currentCellSize,
                                                                            5, false);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 15:
            returnMatrix = scanRegistrationObject->gmmRegistrationD2D(pclNotShifted, pclShifted, initialGuess,
                                                                      useInitialGuess, true);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
        case 16:
            returnMatrix = scanRegistrationObject->gmmRegistrationP2D(pclNotShifted, pclShifted, initialGuess,
                                                                      useInitialGuess, true);
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//            std::cout << returnMatrix << std::endl;
//            std::cout << "test" << std::endl;
            break;
    }



    return returnMatrix;

}

std::vector<measurementResults>
handleRegistrationsOfDirectory(const std::string &s) {
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
//         currentNumberOfScanInDirectory < 2; currentNumberOfScanInDirectory++) {
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
//        for (int currentVoxelSize = 0;currentVoxelSize < sizeOfVoxelGridList.size() - 1; currentVoxelSize++) {//this is size()-1 but for now size()-2
        for (int currentVoxelSize = 0;currentVoxelSize < sizeOfVoxelGridList.size() - 2; currentVoxelSize++) {//this is size()-1 but for now size()-2

            int ourN = std::stoi(sizeOfVoxelGridList[currentVoxelSize]);
//            std::cout << "ourN"<< std::endl;
//            std::cout << ourN<< std::endl;

            for (auto currentSettings: settingsMethodUsedList) {
                measurementResults tmpMeasurements;
                tmpMeasurements.initialGuess = currentSettings.initialGuess;
                tmpMeasurements.whichMethod = currentSettings.whichMethod;
                tmpMeasurements.voxelSize = std::stoi(sizeOfVoxelGridList[currentVoxelSize]);
                tmpMeasurements.overlapValue = overlapValue;
                std::cout << "currentMethod:" << std::endl;
                std::cout << tmpMeasurements.whichMethod << std::endl;

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

                double timeToCalculate;
                estimatedTransformation = registrationOfDesiredMethod(pclNotShifted,
                                                                      pclShifted, final, voxelData, voxelDataShifted,
                                                                      initialGuess,
                                                                      sizeOfVoxelGrid /
                                                                      std::stod(sizeOfVoxelGridList[currentVoxelSize]),
                                                                      currentSettings.whichMethod,
                                                                      currentSettings.initialGuess,
                                                                      scanRegistrationList[currentVoxelSize],timeToCalculate);


//                estimatedTransformation = scanRegistrationList[currentVoxelSize].generalizedIcpRegistration(pclNotShifted,
//                                                                                                            pclShifted,
//                                                                                                            final,
//                                                                                                            fitnessScoreX,
//                                                                                                            initialGuess);

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

                if (estimatedTransformation(3, 3) < 0) {
                    tmpMeasurements.errorInRotation = -1;
                    tmpMeasurements.errorInDistance = -1;
                }

                tmpMeasurements.xGT = gtTransformation.block<3, 1>(0, 3)[0];
                tmpMeasurements.yGT = gtTransformation.block<3, 1>(0, 3)[1];
                tmpMeasurements.angleGT = angleGT;

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
        tmpMeasurements.whichMethod = -1;
        returnMeasurementsList.push_back(tmpMeasurements);

    }
    return returnMeasurementsList;
}


// HOME_LOCATION WHICH_FOLDER_SHOULD_BE_SAVED numberOfScans
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    //1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform, 6: Our FMS 2D 7: initial guess
//    std::vector<settingsMethodUsed> settingsMethodUsedList;
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
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 7;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 8;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = true;
    tmpSettings.whichMethod = 8;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 9;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 10;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 11;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 12;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 13;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 14;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 15;
    settingsMethodUsedList.push_back(tmpSettings);
    tmpSettings.initialGuess = false;
    tmpSettings.whichMethod = 16;
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
//    std::vector<scanRegistrationClass> scanRegistrationList;
    for (int i = 0; i < sizeOfVoxelGridList.size() - 1; i++) {
//    for (int i = 0; i < 1; i++) {
        std::string nameOfTheNode = "registrationObject" +
                                    to_string(i);
        int sizeVoxel = std::stoi(sizeOfVoxelGridList[i]);
//        scanRegistrationClass scanRegistrationObject(sizeVoxel, sizeVoxel / 2, sizeVoxel / 2, sizeVoxel / 2 - 1,
//                                                     nameOfTheNode);
        scanRegistrationList.push_back(
                new scanRegistrationClass(sizeVoxel, sizeVoxel / 2, sizeVoxel / 2, sizeVoxel / 2 - 1,
                                          nameOfTheNode));
    }


    std::vector<measurementResults> resultVector;

    std::vector<std::future<std::vector<measurementResults>>> resultsVectors;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    for (int i = 0; i < listOfDirectories.size(); i++) {
//    for (int i = 0; i < 1; i++) {
        resultsVectors.push_back(std::async(std::launch::deferred, handleRegistrationsOfDirectory, listOfDirectories[i]));
//        resultsVectors.push_back(std::async(std::launch::deferred, handleRegistrationsOfDirectory, listOfDirectories[i]));

        sleep(1);
//        std::vector<measurementResults> tmpVector = resultsVectors.back().get();
//        resultVector.insert(std::end(resultVector), std::begin(tmpVector), std::end(tmpVector));
    }
    std::cout << "started all threads" << std::endl;

    for (auto &&fut: resultsVectors) {
        std::vector<measurementResults> tmpVector = fut.get();
        resultVector.insert(std::end(resultVector), std::begin(tmpVector), std::end(tmpVector));
    }
    std::cout << "done" << std::endl;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]"
              << std::endl;

//    for (int i = 0; i < listOfDirectories.size(); i++) {
////    for (int i = 0; i < 2; i++) {
//        std::vector<measurementResults> tmpVector = handleRegistrationsOfDirectory(listOfDirectories[i]);
//
//        resultVector.insert(std::end(resultVector), std::begin(tmpVector), std::end(tmpVector));
//        std::cout << "############################# we are at Number: " << i << std::endl;
//
//    }




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
        resultsMatching << ",";
        resultsMatching << resultVector[i].xGT;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].yGT;//time
        resultsMatching << ",";
        resultsMatching << resultVector[i].angleGT;//time
        resultsMatching << "\n";//error
    }

    resultsMatching.close();

    return (0);
}


