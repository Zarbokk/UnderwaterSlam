//
// Created by jurobotics on 13.09.21.
//


#include "geometry_msgs/PoseStamped.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include "nav_msgs/OccupancyGrid.h"
#include "scanRegistrationClass.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/SetBoolRequest.h"
#include <std_srvs/SetBool.h>
#include "commonbluerovmsg/staterobotforevaluation.h"
#include <filesystem>


#define NUMBER_OF_POINTS_DIMENSION 128
#define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 50 // was 50 //tuhh tank 6
#define NUMBER_OF_POINTS_MAP 512//was 512
// 80 simulation 300 valentin 45.0 for Keller 10.0 TUHH TANK
#define DIMENSION_OF_MAP 45.0

#define IGNORE_DISTANCE_TO_ROBOT 1.0 // was 1.0 // TUHH 0.2
#define DEBUG_REGISTRATION false

#define ROTATION_SONAR M_PI // sonar on robot M_PI // simulation 0
#define SHOULD_USE_ROSBAG true
#define FACTOR_OF_MATCHING 1.0 //1.5
#define THRESHOLD_FOR_TRANSLATION_MATCHING 0.1 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben

#define INTEGRATED_NOISE_XY 0.005 // was 0.03  // TUHH 0.005
#define INTEGRATED_NOISE_YAW 0.005 // was 0.03 // TUHH 0.005

#define USE_INITIAL_TRANSLATION_LOOP_CLOSURE true
#define MAXIMUM_LOOP_CLOSURE_DISTANCE 2.0 // 0.2 TUHH 2.0 valentin Keller 4.0 Valentin Oben // 2.0 simulation

#define TUHH_SYSTEM false
#define SIMULATION_SYSTEM false


#define HOME_LOCATION "/home/tim-external/dataFolder/testFolder/"
#define WHICH_FOLDER_SHOULD_BE_SAVED "stPereHighNoise52NEW/"

// Shifts/Dimensions(list)/Occlusions/Noise/shiftProperties
struct settingsExtension {
    int numberOfShifts;
    bool useShift;
    bool useOcclusions;
    bool useNoise;
    std::vector<int> listOfDimensions;
    double occlusionPercentage;
    double noiseSaltPepper;
    double noiseGauss;
};



class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_, const std::string &nameOfTheServiceCall) : graphSaved(3, INTENSITY_BASED_GRAPH),
                                                                               scanRegistrationObject(
                                                                                       NUMBER_OF_POINTS_DIMENSION,
                                                                                       NUMBER_OF_POINTS_DIMENSION / 2,
                                                                                       NUMBER_OF_POINTS_DIMENSION / 2,
                                                                                       NUMBER_OF_POINTS_DIMENSION / 2 -
                                                                                       1) {
        this->ourSettings.numberOfShifts = 10;
        this->ourSettings.useOcclusions = true;
        this->ourSettings.useShift = true;
        this->ourSettings.useNoise = true;
        this->ourSettings.listOfDimensions;
        this->ourSettings.noiseSaltPepper = 0.1;
        this->ourSettings.noiseGauss = 0.1;
        this->ourSettings.occlusionPercentage = 0.1;




        this->subscriberEKF = n_.subscribe("publisherPoseEkf", 10000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(2).sleep();
        this->subscriberIntensitySonar = n_.subscribe("sonar/intensity", 10000, &rosClassEKF::scanCallback, this);
//        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);
        this->subscriberPositionGT = n_.subscribe("positionGT", 100000, &rosClassEKF::groundTruthEvaluationCallback,
                                                  this);
        this->subscriberPositionGTGantry = n_.subscribe("gantry/path_follower/current_position", 100000,
                                                        &rosClassEKF::groundTruthEvaluationTUHHCallback,
                                                        this);

        publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        publisherOccupancyMap = n_.advertise<nav_msgs::OccupancyGrid>("occupancyHilbertMap", 10);

        publisherPoseSLAM = n_.advertise<geometry_msgs::PoseStamped>("slamEndPose", 10);

        if (SHOULD_USE_ROSBAG) {
            pauseRosbag = n_.serviceClient<std_srvs::SetBoolRequest>(nameOfTheServiceCall);
        }

        this->sigmaScaling = 0.1;// was 1.0
        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;
        this->numberOfTimesFirstScan = 0;
        this->saveGraphStructure = false;
//        this->maxTimeOptimization = 1.0;
        this->currentGTPosition = Eigen::Matrix4d::Identity();

        map.info.height = NUMBER_OF_POINTS_MAP;
        map.info.width = NUMBER_OF_POINTS_MAP;
        map.info.resolution = DIMENSION_OF_MAP / NUMBER_OF_POINTS_MAP;
        map.info.origin.position.x = -DIMENSION_OF_MAP / 2;
        map.info.origin.position.y = -DIMENSION_OF_MAP / 2;
        map.info.origin.position.z = +0.5;
        for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
                //determine color:
                map.data.push_back(50);
            }
        }

        this->numberOfScan = 0;
        std::filesystem::create_directory(std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED));
        // Save Settings to File
    }


private:
    nav_msgs::OccupancyGrid map;
    int numberOfScan;
    settingsExtension ourSettings;
    ros::Subscriber subscriberEKF, subscriberIntensitySonar, subscriberPositionGT, subscriberPositionGTGantry;
    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
    ros::ServiceServer serviceSaveGraph;
    ros::ServiceClient pauseRosbag;
    std::mutex stateEstimationMutex;
    std::mutex groundTruthMutex;
    std::mutex graphSlamMutex;

    //GraphSlam things
    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;

    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;


    //EKF savings
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
    std::deque<Eigen::Quaterniond> rotationVector;

    // GT savings
    std::deque<transformationStamped> currentPositionGTDeque;
    Eigen::Matrix4d currentGTPosition;

    double sigmaScaling;

    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan, saveGraphStructure;
    int numberOfTimesFirstScan;

    void scanCallback(const commonbluerovmsg::SonarEcho2::ConstPtr &msg) {
//        std::cout << "huhu1" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
//        std::cout << "huhu2" << std::endl;
        intensityMeasurement intensityTMP;
        if (TUHH_SYSTEM) {
            intensityTMP.angle = std::fmod(-msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR,
                                           M_PI * 2);// TEST TRYING OUT -
        } else {
            intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR,
                                           M_PI * 2);// TEST TRYING OUT -
        }

        intensityTMP.time = msg->header.stamp.toSec();
        intensityTMP.range = msg->range;
        intensityTMP.increment = msg->step_size;
        std::vector<double> intensitiesVector;
        for (int i = 0; i < msg->intensities.size(); i++) {
            intensitiesVector.push_back(msg->intensities[i]);
        }
        intensityTMP.intensities = intensitiesVector;

        if (firstSonarInput) {

            this->graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                       Eigen::Matrix3d::Zero(), intensityTMP, msg->header.stamp.toSec(),
                                       FIRST_ENTRY);
            if (TUHH_SYSTEM) {
                //SAVE TUHH GT POSE
                this->graphSaved.getVertexList()->at(0).setGroundTruthTransformation(getCurrentGTPosition());
            }
            firstSonarInput = false;
            return;
        }
        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement

        bool waitingForMessages = waitForEKFMessagesToArrive(msg->header.stamp.toSec());
        if (!waitingForMessages) {
            return;
        }
        edge differenceOfEdge = slamToolsRos::calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved.getVertexList()->back().getTimeStamp(), msg->header.stamp.toSec(), this->timeVector,
                this->xPositionVector, this->yPositionVector,
                this->zPositionVector, this->rotationVector, this->stateEstimationMutex);

        slamToolsRos::clearSavingsOfPoses(this->graphSaved.getVertexList()->back().getTimeStamp() - 2, this->timeVector,
                                          this->xPositionVector, this->yPositionVector,
                                          this->zPositionVector, this->rotationVector, this->stateEstimationMutex);

        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);


        this->graphSaved.addVertex(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
                                   this->graphSaved.getVertexList()->back().getCovarianceMatrix(),
                                   intensityTMP,
                                   msg->header.stamp.toSec(),
                                   INTENSITY_SAVED);
        if (TUHH_SYSTEM) {
            //SAVE TUHH GT POSE
            this->graphSaved.getVertexList()->back().setGroundTruthTransformation(getCurrentGTPosition());
        }


        Eigen::Matrix3d covarianceMatrix = Eigen::Matrix3d::Zero();
        covarianceMatrix(0, 0) = INTEGRATED_NOISE_XY;
        covarianceMatrix(1, 1) = INTEGRATED_NOISE_XY;
        covarianceMatrix(2, 2) = INTEGRATED_NOISE_YAW;
        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 covarianceMatrix, INTEGRATED_POSE);


        int indexOfLastKeyframe;
        double angleDiff = slamToolsRos::angleBetweenLastKeyframeAndNow(this->graphSaved);// i think this is always true

        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (abs(angleDiff) > 2 * M_PI / FACTOR_OF_MATCHING) {

            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
            if (firstCompleteSonarScan) {
                numberOfTimesFirstScan++;
                if (numberOfTimesFirstScan > 2 * FACTOR_OF_MATCHING - 1) {
                    firstCompleteSonarScan = false;
                }
                return;
            }

            if (SHOULD_USE_ROSBAG) {
                std_srvs::SetBool srv;
                srv.request.data = true;
                pauseRosbag.call(srv);
            }
            //sort in GT
            if (SIMULATION_SYSTEM) {
                this->saveCurrentGTPosition();
            }

            //angleDiff = angleBetweenLastKeyframeAndNow(false);
//            indexOfLastKeyframe = slamToolsRos::getLastIntensityKeyframe(this->graphSaved);
            int indexStart1, indexEnd1, indexStart2, indexEnd2;
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);

            // we inverse the initial guess, because the registration creates a T from scan 1 to scan 2.
            // But the graph creates a transformation from 1 -> 2 by the robot, therefore inverse.
            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart1).getTransformation());


            std::string directoryOfInterest = std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) +
                                              std::string("scanNumber_") + std::to_string(numberOfScan) +
                                              std::string("/");
            std::filesystem::create_directory(directoryOfInterest);
            //create current settings: Number of Shifts/Dimensions(list)/Occlusions/Noise/shiftProperties


            for(int numberOfShifts = 0 ; numberOfShifts<this->ourSettings.numberOfShifts ; numberOfShifts++){

                for(auto &numberOfPoints: this->ourSettings.listOfDimensions) {

                    double *voxelData1,*voxelData2;

                    voxelData1 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
                    voxelData2 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
                    if(this->ourSettings.useShift){
                        //if used save in txt file correct shift

                    }
                    double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                                         numberOfPoints,
                                                                                         this->graphSaved,
                                                                                         IGNORE_DISTANCE_TO_ROBOT,
                                                                                         DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                         Eigen::Matrix4d::Identity());//get voxel
                    double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                                         numberOfPoints,
                                                                                         this->graphSaved,
                                                                                         IGNORE_DISTANCE_TO_ROBOT,
                                                                                         DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                         Eigen::Matrix4d::Identity());//get voxel

                    if(this->ourSettings.useOcclusions){
                        //if used, save % removed, save Overlap of both Occlusions

                    }
                    if(this->ourSettings.useNoise){

                    }


                    //convert every scan to Point Cloud






                    std::ofstream outShifted1(directoryOfInterest + std::to_string(this->numberOfScan) +
                                             std::string("scan1") + "_" + std::to_string(numberOfShifts) + "_" +
                                             std::string(std::to_string(numberOfPoints)) + std::string(".csv"));
                    for (int j = 0; j < numberOfPoints; j++) {
                        for (int k = 0; k < numberOfPoints; k++) {
                            outShifted1 << voxelData1[j + numberOfPoints * k] << ',';
                        }
                        outShifted1 << '\n';
                    }
                    outShifted1.close();

                    std::ofstream outShifted2(directoryOfInterest + std::to_string(this->numberOfScan) +
                                              std::string("scan2") + "_" + std::to_string(numberOfShifts) + "_" +
                                              std::string(std::to_string(numberOfPoints)) + std::string(".csv"));
                    for (int j = 0; j < numberOfPoints; j++) {
                        for (int k = 0; k < numberOfPoints; k++) {
                            outShifted2 << voxelData2[j + numberOfPoints * k] << ',';
                        }
                        outShifted2 << '\n';
                    }
                    outShifted2.close();


                }
            }

//            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2, indexStart2, indexEnd2,
//                                                                                 NUMBER_OF_POINTS_DIMENSION,
//                                                                                 this->graphSaved,
//                                                                                 IGNORE_DISTANCE_TO_ROBOT,
//                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
//                                                                                 Eigen::Matrix4d::Identity());//get voxel

            Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
            std::cout << "direct matching consecutive: " << std::endl;
            // result is matrix to transform scan 1 to scan 2 therefore later inversed + initial guess inversed


            slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                    this->publisherMarkerArray, this->sigmaScaling,
                                                    this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);

            std::cout << "next: " << std::endl;
            this->numberOfScan++;
            if (SHOULD_USE_ROSBAG) {
                std_srvs::SetBool srv;
                srv.request.data = false;
                pauseRosbag.call(srv);
            }

        }
        slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                this->publisherMarkerArray, this->sigmaScaling,
                                                this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);
    }

    void stateEstimationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
        double currentTimeStamp = msg->header.stamp.toSec();
        // calculate where to put the current new message
        int i = 0;
        if (!this->timeVector.empty()) {
            i = this->timeVector.size();
            while (this->timeVector[i - 1] > currentTimeStamp) {
                i--;
            }
        }

        if (i == this->timeVector.size() || i == 0) {
            Eigen::Quaterniond tmpQuad(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            this->rotationVector.push_back(tmpQuad);
            this->timeVector.push_back(msg->header.stamp.toSec());
            this->xPositionVector.push_back(msg->pose.pose.position.x);
            this->yPositionVector.push_back(msg->pose.pose.position.y);
            this->zPositionVector.push_back(msg->pose.pose.position.z);
        } else {
            std::cout << "we test it" << std::endl;
            exit(0);
        }
    }

    bool waitForEKFMessagesToArrive(double timeUntilWait) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << "tmp1" << std::endl;
        while (this->timeVector.empty() && timeToCalculate < 10) {
            ros::Duration(0.002).sleep();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }
//        std::cout << "tmp2" << std::endl;
        while (timeUntilWait > timeVector[timeVector.size() - 1]) {
            ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf", ros::Duration(10));
            ros::Duration(0.001).sleep();

            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//            std::cout << timeToCalculate << std::endl;
            if (timeToCalculate > 10) {
                break;
            }
        }
//        std::cout << "tmp3" << std::endl;
//        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
        ros::Duration(0.002).sleep();
//        std::cout << "tmp4" << std::endl;
        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        if (timeToCalculate > 8) {
            return false;
        } else {
            return true;
        }
    }

    void groundTruthEvaluationCallback(const commonbluerovmsg::staterobotforevaluation::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        //first time? calc current Position
        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(msg->roll, msg->pitch,
                                                                                        msg->yaw);
        tmpMatrix(0, 3) = msg->xPosition;
        tmpMatrix(1, 3) = msg->yPosition;
        tmpMatrix(2, 3) = msg->zPosition;
        transformationStamped tmpValue;
        tmpValue.transformation = tmpMatrix;
        tmpValue.timeStamp = msg->header.stamp.toSec();
        this->currentPositionGTDeque.push_back(tmpValue);
    }

    void groundTruthEvaluationTUHHCallback(const geometry_msgs::Point::ConstPtr &msg) {


//        std::cout << "test" << std::endl;
        //first time? calc current Position
        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0);
        tmpMatrix(0, 3) = msg->x;
        tmpMatrix(1, 3) = msg->y;
        tmpMatrix(2, 3) = msg->z;
//        std:: cout << tmpMatrix << std::endl;
        transformationStamped tmpValue;
        tmpValue.transformation = tmpMatrix;
        tmpValue.timeStamp = ros::Time::now().toSec();

        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        this->currentGTPosition = tmpMatrix;

//        this->currentPositionGTDeque.push_back(tmpValue);
    }

    Eigen::Matrix4d getCurrentGTPosition() {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        return this->currentGTPosition;
    }

    void saveCurrentGTPosition() {
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        while (!this->currentPositionGTDeque.empty()) {
            double currentTimeStampOfInterest = this->currentPositionGTDeque[0].timeStamp;
            std::cout << currentTimeStampOfInterest << std::endl;
            int i = this->graphSaved.getVertexList()->size() - 1;
            while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >= currentTimeStampOfInterest) {
                i--;
                if (i == -1) {
                    break;
                }
            }
            i++;
            if (i == this->graphSaved.getVertexList()->size()) {
                break;
            }
//            if (i == 0) {
//                break;
//            }

//            std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
//            std::cout << currentTimeStampOfInterest << std::endl;



            //sort in
            int j = 0;
            while (this->graphSaved.getVertexList()->at(i).getTimeStamp() >=
                   this->currentPositionGTDeque[j].timeStamp) {
                j++;
                if (j == this->currentPositionGTDeque.size()) {
                    break;
                }
            }
            if (j == this->currentPositionGTDeque.size()) {
                break;
            }
//            std::cout << this->graphSaved.getVertexList()->at(i).getTimeStamp() << std::endl;
//            std::cout << this->currentPositionGTDeque[j].timeStamp << std::endl;
            this->graphSaved.getVertexList()->at(i).setGroundTruthTransformation(
                    this->currentPositionGTDeque[j].transformation);


            for (int k = 0; k < j + 1; k++) {
                this->currentPositionGTDeque.pop_front();
            }




//            this->currentPositionGTDeque.pop_front();
        }

    }


    Eigen::Matrix4d createVoxelWithFollowingRandomChanges(double voxelData[],Eigen::Matrix4d movePosition,int typeOfMatching,int numberOfPoints,double percentageSaltAndPepper,double stdDivGaussianNoise,double percentageOcclusions,std::ofstream outShifted, int indexStart,int indexEnd){

//        int numberOfPoints = 32;//NUMBER_OF_POINTS_DIMENSION;
        double *voxelData1;

        voxelData1 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);

        //potential movement here
        double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart, indexEnd,
                                                                             numberOfPoints,
                                                                             this->graphSaved,
                                                                             IGNORE_DISTANCE_TO_ROBOT,
                                                                             DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                             Eigen::Matrix4d::Identity());//get voxel



        //add occlusion
        //add salt and pepper noise
        //add gaussian noise
        //maybe create here a pointcloud

        int numberOfTransformation = 0;
//        std::ofstream outShifted(directoryOfInterest + std::to_string(numberOfTransformation) +
//                                 std::string("intensity2DMap") +
//                                 std::string(std::to_string(numberOfPoints)) + std::string(".csv"));

        for (int j = 0; j < numberOfPoints; j++) {
            for (int k = 0; k < numberOfPoints; k++){
                outShifted << voxelData1[j + numberOfPoints * k] << ',';
            }
            outShifted << '\n';
        }
//        outShifted.close();


        return Eigen::Matrix4d::Identity();
    }

public:

    void createImageOfAllScans() {

        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->graphSlamMutex);
        //homePosition is 0 0
        //size of mapData is defined in NUMBER_OF_POINTS_MAP

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        for (int currentPosition = 0;
             currentPosition < dataSet.size(); currentPosition++) {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
            //was 90 yaw and 180 roll

            Eigen::Matrix4d transformationOfIntensityRay =
                    generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.0 / 180.0 * M_PI) *
                    generalHelpfulTools::getTransformationMatrixFromRPY(0.0 / 180.0 * M_PI, 0, 0) *
                    dataSet[currentPosition].transformation;
            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             dataSet[currentPosition].intensity.angle);

            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT / (dataSet[currentPosition].intensity.range /
                                                                    ((double) dataSet[currentPosition].intensity.intensities.size())));


            for (int j = ignoreDistance;
                 j <
                 dataSet[currentPosition].intensity.intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) dataSet[currentPosition].intensity.intensities.size()) *
                        ((double) dataSet[currentPosition].intensity.range);

                int incrementOfScan = dataSet[currentPosition].intensity.increment;
                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                    Eigen::Vector4d positionOfIntensity(
                            distanceOfIntensity,
                            0,
                            0,
                            1);
                    double rotationOfPoint = l / 400.0;
                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0,
                                                                                                                0,
                                                                                                                rotationOfPoint);
                    positionOfIntensity = rotationForBetterView * positionOfIntensity;

                    positionOfIntensity =
                            transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                    int indexX =
                            (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                   2) +
                            NUMBER_OF_POINTS_MAP / 2;


                    if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                                mapData[indexX + NUMBER_OF_POINTS_MAP * indexY] +
                                dataSet[currentPosition].intensity.intensities[j];
                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                        //                    std::cout << "random: " << std::endl;
                    }
                }
            }

        }


        //make sure next iteration the correct registrationis calculated
        //TO THE END
        //NOW: TO THE BEGINNING


        double maximumOfVoxelData = 0;
        double minimumOfVoxelData = INFINITY;

        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            if (voxelDataIndex[i] > 0) {
                mapData[i] = mapData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < mapData[i]) {
                    maximumOfVoxelData = mapData[i];
                }
                if (minimumOfVoxelData > mapData[i]) {
                    minimumOfVoxelData = mapData[i];
                }
                //std::cout << voxelData[i] << std::endl;
            }
        }


        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {

            mapData[i] = (mapData[i] - minimumOfVoxelData) / (maximumOfVoxelData - minimumOfVoxelData) * 250;
        }


        nav_msgs::OccupancyGrid occupanyMap;
        occupanyMap.header.frame_id = "map_ned";
        occupanyMap.info.height = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.width = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.resolution = DIMENSION_OF_MAP / NUMBER_OF_POINTS_MAP;
        occupanyMap.info.origin.position.x = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.y = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.z = +0.5;
        for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
                //determine color:
                occupanyMap.data.push_back((int) (mapData[j + NUMBER_OF_POINTS_MAP * i]));
            }
        }
        this->publisherOccupancyMap.publish(occupanyMap);
        free(voxelDataIndex);
        free(mapData);
    }

};


int main(int argc, char **argv) {


    ros::init(argc, argv, "ekfwithros");


    ros::V_string nodes;
    int i = 0;
    std::string stringForRosClass;
    if (SHOULD_USE_ROSBAG) {
        slamToolsRos::getNodes(nodes);


        for (i = 0; i < nodes.size(); i++) {

            if (nodes[i].substr(1, 4) == "play") {
                //            std::cout << "we found it" << std::endl;
                break;
            }
        }
//            std::cout << nodes[i]+"/pause_playback" << std::endl;
//        ros::ServiceServer serviceResetEkf;


        if (ros::service::exists(nodes[i] + "/pause_playback", true)) {

        } else {
            exit(-1);
        }

        stringForRosClass = nodes[i] + "/pause_playback";
    }


    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_, stringForRosClass);


//    ros::spin();


    ros::Rate loop_rate(0.1);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Duration(10).sleep();

    while (ros::ok()) {
        // ros::spinOnce();

//         rosClassForTests.updateHilbertMap();
//         rosClassForTests.updateMap();

        rosClassForTests.createImageOfAllScans();
        loop_rate.sleep();

        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}
