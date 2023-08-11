//
// Created by jurobotics on 13.09.21.
//


#include "geometry_msgs/PoseStamped.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include "nav_msgs/OccupancyGrid.h"
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


#define HOME_LOCATION "/home/tim-external/dataFolder/journalPaperDatasets/newDatasetsCreation/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "highNoiseBigMotionValentin/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "highNoiseBigMotionKeller/"

//#define WHICH_FOLDER_SHOULD_BE_SAVED "noNoiseSmallMotionValentin/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "noNoiseSmallMotionKeller/"

#define WHICH_FOLDER_SHOULD_BE_SAVED "onlyRotationNoNoiseKeller/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "onlyRotationNoNoiseValentin/"

//#define WHICH_FOLDER_SHOULD_BE_SAVED "speedTestsValentin/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "speedTestsKeller/"

//#define WHICH_FOLDER_SHOULD_BE_SAVED "noNoiseSmallMotionStPere/"


// Shifts/Dimensions(list)/Occlusions/Noise/shiftProperties
struct settingsExtension {
    int numberOfShifts;
    bool useShift;
    bool useOcclusions;
    bool useNoise;
    std::vector<int> listOfDimensions;
    double occlusionPercentage;
    double noiseSaltPepperPercentage;
    double noiseGaussPercentage;
    double randomShiftXY;
    double randomRotation;
    bool onlyOneRotationDirection;

};


struct saveSettingsOfRandomSettings {
    Eigen::Matrix4d randomShift;
    Eigen::Vector2d shiftFirstOcclusion;
    Eigen::Vector2d shiftSecondOcclusion;
    double rotationFirstOcclusion;
    double rotationSecondOcclusion;
    int patternOcclusionFirst;
    int patternOcclusionSecond;
    std::vector<int> listN;
    double scalingOcclusionFirst;
    double scalingOcclusionSecond;
    double randomOcclusionParameterFirst;
    double randomOcclusionParameterSecond;
    double sizeVoxelData;
    double overlapOcclusions;
};


class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_, const std::string &nameOfTheServiceCall) : graphSaved(3, INTENSITY_BASED_GRAPH) {

        // simple random motion

//        this->ourSettings.numberOfShifts = 100;
//        this->ourSettings.useOcclusions = false;
//        this->ourSettings.useShift = true;
//        this->ourSettings.useNoise = false;
//        std::vector<int> vect{64, 128, 256};
//        this->ourSettings.listOfDimensions = vect;
//        this->ourSettings.noiseSaltPepperPercentage = 0.005;
//        this->ourSettings.noiseGaussPercentage = 0.035;
//        this->ourSettings.occlusionPercentage = 0.8;
//        this->ourSettings.randomRotation = 5.0 / 180.0 * M_PI;
//        this->ourSettings.randomShiftXY = 1;
//        this->ourSettings.onlyOneRotationDirection = false;

        // high noise and high motions
//        this->ourSettings.numberOfShifts = 100;
//        this->ourSettings.useOcclusions = true;
//        this->ourSettings.useShift = true;
//        this->ourSettings.useNoise = true;
//        std::vector<int> vect{64,128,256};
//        this->ourSettings.listOfDimensions = vect;
//        this->ourSettings.noiseSaltPepperPercentage = 0.006;
//        this->ourSettings.noiseGaussPercentage = 0.040;//0.035
//        this->ourSettings.occlusionPercentage = 1.0;
//        this->ourSettings.randomRotation = 15.0 / 180.0 * M_PI;
//        this->ourSettings.randomShiftXY = 5;
//        this->ourSettings.onlyOneRotationDirection = false;

        // only Angle
        this->ourSettings.numberOfShifts = 100;
        this->ourSettings.useOcclusions = false;
        this->ourSettings.useShift = true;
        this->ourSettings.useNoise = false;
        std::vector<int> vect{64, 128, 256};
        this->ourSettings.listOfDimensions = vect;
        this->ourSettings.noiseSaltPepperPercentage = 0.005;
        this->ourSettings.noiseGaussPercentage = 0.035;
        this->ourSettings.occlusionPercentage = 0.1;
        this->ourSettings.randomRotation = 120.0 / 180.0 * M_PI;
        this->ourSettings.randomShiftXY = 0;
        this->ourSettings.onlyOneRotationDirection = true;

        // speed Test
//        this->ourSettings.numberOfShifts = 10;
//        this->ourSettings.useOcclusions = false;
//        this->ourSettings.useShift = true;
//        this->ourSettings.useNoise = true;
//        std::vector<int> vect{64, 128, 256, 512, 1024};
//        this->ourSettings.listOfDimensions = vect;
//        this->ourSettings.noiseSaltPepperPercentage = 0.001;
//        this->ourSettings.noiseGaussPercentage = 0.015;
//        this->ourSettings.occlusionPercentage = 0.1;
//        this->ourSettings.randomRotation = 10.0 / 180.0 * M_PI;
//        this->ourSettings.randomShiftXY = 2;
//        this->ourSettings.onlyOneRotationDirection = false;




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


        std::ofstream fileForSettings(
                std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) + "generalSettings" +
                std::string(".csv"));
        fileForSettings << this->ourSettings.useOcclusions << '\n';
        fileForSettings << this->ourSettings.useShift << '\n';
        fileForSettings << this->ourSettings.useNoise << '\n';
        fileForSettings << this->ourSettings.randomRotation << '\n';
        fileForSettings << this->ourSettings.noiseSaltPepperPercentage << '\n';
        fileForSettings << this->ourSettings.noiseGaussPercentage << '\n';
        fileForSettings << this->ourSettings.occlusionPercentage << '\n';
        fileForSettings << this->ourSettings.randomShiftXY << '\n';

        fileForSettings.close();


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
    scanRegistrationClass tmpTest;
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
//    scanRegistrationClass scanRegistrationObject;
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

            std::vector<saveSettingsOfRandomSettings> listOfSettings;

            for (int numberOfShifts = 0; numberOfShifts < this->ourSettings.numberOfShifts; numberOfShifts++) {

                Eigen::Matrix4d randomShiftAndRotation = Eigen::Matrix4d::Identity();


                if (this->ourSettings.useShift) {
                    //if used save in txt file correct shift
                    randomShiftAndRotation = this->randomShiftAndRotationCalculation(ourSettings.randomShiftXY,
                                                                                     ourSettings.randomRotation,ourSettings.onlyOneRotationDirection);
                }

                // Add to append to TXT file
                saveSettingsOfRandomSettings tmpSettings;
                tmpSettings.randomShift = randomShiftAndRotation;
                tmpSettings.listN = this->ourSettings.listOfDimensions;
                tmpSettings.sizeVoxelData = DIMENSION_OF_VOXEL_DATA_FOR_MATCHING;


                if (this->ourSettings.useOcclusions) {
                    //if used, save % removed, save Overlap of both Occlusions 933013
                    std::random_device rd;

                    std::mt19937 gen(rd());

                    std::uniform_real_distribution<> dis(0.0, 1.0);







                    double patternCalc = dis(gen) * 3;
                    if (patternCalc < 1) {
                        tmpSettings.patternOcclusionFirst = 0;
                    } else {
                        if (patternCalc < 2) {
                            tmpSettings.patternOcclusionFirst = 1;
                        } else {
                            tmpSettings.patternOcclusionFirst = 2;
                        }
                    }
                    patternCalc = dis(gen) * 3;

                    if (patternCalc < 1) {
                        tmpSettings.patternOcclusionSecond = 0;
                    } else {
                        if (patternCalc < 2) {
                            tmpSettings.patternOcclusionSecond = 1;
                        } else {
                            tmpSettings.patternOcclusionSecond = 2;
                        }
                    }

                    tmpSettings.rotationFirstOcclusion = dis(gen) * M_PI * 2;
                    tmpSettings.rotationSecondOcclusion = dis(gen) * M_PI * 2;
                    tmpSettings.shiftFirstOcclusion = Eigen::Vector2d((dis(gen) - 0.5) * 2, (dis(gen) - 0.5) * 2);
                    tmpSettings.shiftSecondOcclusion = Eigen::Vector2d((dis(gen) - 0.5) * 2, (dis(gen) - 0.5) * 2);
                    tmpSettings.scalingOcclusionFirst = dis(gen) * 2.0;// was 1.5
                    tmpSettings.scalingOcclusionSecond = dis(gen) * 2.0;// was 1.5
                    tmpSettings.randomOcclusionParameterFirst = dis(gen);
                    tmpSettings.randomOcclusionParameterSecond = dis(gen);
                    double overlapOcclusions = this->calculateOverlap(tmpSettings.rotationFirstOcclusion,
                                                                      tmpSettings.rotationSecondOcclusion,
                                                                      tmpSettings.patternOcclusionFirst,
                                                                      tmpSettings.patternOcclusionSecond,
                                                                      tmpSettings.shiftFirstOcclusion,
                                                                      tmpSettings.shiftSecondOcclusion,
                                                                      tmpSettings.scalingOcclusionFirst,
                                                                      tmpSettings.scalingOcclusionSecond,
                                                                      tmpSettings.randomOcclusionParameterFirst,
                                                                      tmpSettings.randomOcclusionParameterSecond,
                                                                      1000);
//                    std::cout << "overlapOcclusions" << std::endl;
//                    std::cout << percentageOverlap1 << " " << percentageOverlap2 << " " << overlapOcclusions
//                              << std::endl;





                    tmpSettings.overlapOcclusions = overlapOcclusions;
                    if(dis(gen)>this->ourSettings.occlusionPercentage){
                        tmpSettings.overlapOcclusions = 0;
                    }
                }


                listOfSettings.push_back(tmpSettings);

                for (auto &numberOfPoints: this->ourSettings.listOfDimensions) {

                    double *voxelData1, *voxelData2;

                    voxelData1 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
                    voxelData2 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);

                    double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1,
                                                                                         indexEnd1,
                                                                                         numberOfPoints,
                                                                                         this->graphSaved,
                                                                                         IGNORE_DISTANCE_TO_ROBOT,
                                                                                         DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                         Eigen::Matrix4d::Identity());//get voxel
                    double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2, indexStart1,
                                                                                         indexEnd1,
                                                                                         numberOfPoints,
                                                                                         this->graphSaved,
                                                                                         IGNORE_DISTANCE_TO_ROBOT,
                                                                                         DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                         tmpSettings.randomShift);//get voxel
                    //makes voxel 1 all 1
//                    for (int i = 0; i < numberOfPoints * numberOfPoints; i++) {
//                        voxelData1[i] = maximumVoxel1;
//                    }
                    if (this->ourSettings.useOcclusions ) {
                        if(tmpSettings.overlapOcclusions>0) {
                            //if used, save % removed, save Overlap of both Occlusions
                            double percentageOverlap1 = this->applyContourToVoxelData(voxelData1,
                                                                                      listOfSettings.back().rotationFirstOcclusion,
                                                                                      listOfSettings.back().shiftFirstOcclusion,
                                                                                      listOfSettings.back().patternOcclusionFirst,
                                                                                      listOfSettings.back().scalingOcclusionFirst,
                                                                                      numberOfPoints,
                                                                                      listOfSettings.back().randomOcclusionParameterFirst);
                            double percentageOverlap2 = this->applyContourToVoxelData(voxelData2,
                                                                                      listOfSettings.back().rotationSecondOcclusion,
                                                                                      listOfSettings.back().shiftSecondOcclusion,
                                                                                      listOfSettings.back().patternOcclusionSecond,
                                                                                      listOfSettings.back().scalingOcclusionSecond,
                                                                                      numberOfPoints,
                                                                                      listOfSettings.back().randomOcclusionParameterSecond);
                        }
                    }
                    if (this->ourSettings.useNoise) {
                        this->addSaltPepperNoiseToVoxel(voxelData1, ourSettings.noiseSaltPepperPercentage,
                                                        maximumVoxel1, numberOfPoints);
                        this->addSaltPepperNoiseToVoxel(voxelData2, ourSettings.noiseSaltPepperPercentage,
                                                        maximumVoxel1, numberOfPoints);

                        this->addNoiseToVoxel(voxelData1, ourSettings.noiseGaussPercentage, maximumVoxel1,
                                              numberOfPoints);
                        this->addNoiseToVoxel(voxelData2, ourSettings.noiseGaussPercentage, maximumVoxel1,
                                              numberOfPoints);
                    }


                    //convert every scan to Point Cloud


                    pcl::PointCloud<pcl::PointXYZ> scan1Threshold = slamToolsRos::convertVoxelToPointcloud(voxelData1, 0.4,
                                                                                                   maximumVoxel1,
                                                                                                   numberOfPoints,DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);
                    pcl::PointCloud<pcl::PointXYZ> scan2Threshold = slamToolsRos::convertVoxelToPointcloud(voxelData2, 0.4,
                                                                                                   maximumVoxel1,
                                                                                                   numberOfPoints,DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);


                    pcl::io::savePCDFile(directoryOfInterest +
                                         std::string("scan1") + "_" + std::to_string(numberOfShifts) + "_" +
                                         std::string(std::to_string(numberOfPoints)) +
                                         std::string("_pcl.pcd"),
                                         scan1Threshold);
                    pcl::io::savePCDFile(directoryOfInterest +
                                         std::string("scan2") + "_" + std::to_string(numberOfShifts) + "_" +
                                         std::string(std::to_string(numberOfPoints)) +
                                         std::string("_pcl.pcd"),
                                         scan2Threshold);


                    std::ofstream outShifted1(directoryOfInterest +
                                              std::string("scan1") + "_" + std::to_string(numberOfShifts) + "_" +
                                              std::string(std::to_string(numberOfPoints)) + std::string(".csv"));
                    for (int j = 0; j < numberOfPoints; j++) {
                        for (int k = 0; k < numberOfPoints; k++) {
                            outShifted1 << voxelData1[j + numberOfPoints * k] << ',';
                        }
                        outShifted1 << '\n';
                    }
                    outShifted1.close();

                    std::ofstream outShifted2(directoryOfInterest +
                                              std::string("scan2") + "_" + std::to_string(numberOfShifts) + "_" +
                                              std::string(std::to_string(numberOfPoints)) + std::string(".csv"));
                    for (int j = 0; j < numberOfPoints; j++) {
                        for (int k = 0; k < numberOfPoints; k++) {
                            outShifted2 << voxelData2[j + numberOfPoints * k] << ',';
                        }
                        outShifted2 << '\n';
                    }
                    outShifted2.close();

                    free(voxelData1);
                    free(voxelData2);
                }
//                std::cout << tmpSettings.randomShift << std::endl;
            }

//            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2, indexStart2, indexEnd2,
//                                                                                 NUMBER_OF_POINTS_DIMENSION,
//                                                                                 this->graphSaved,
//                                                                                 IGNORE_DISTANCE_TO_ROBOT,
//                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
//                                                                                 Eigen::Matrix4d::Identity());//get voxel





            // Save settings of everything

            // still missing: occlusion values (% and so on)
            std::ofstream fileForSettings(directoryOfInterest + "settingsForThisDirectory" + std::string(".csv"));
            fileForSettings << listOfSettings.size() << '\n';
            fileForSettings << listOfSettings[0].sizeVoxelData << '\n';
            for (auto &sizeN: listOfSettings[0].listN) {
                fileForSettings << sizeN << ",";

            }
            fileForSettings << '\n';

            for (auto &tmp: listOfSettings) {

                fileForSettings << tmp.overlapOcclusions << '\n';


                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k < 4; k++) {
                        fileForSettings << tmp.randomShift(j, k) << ',';
                    }
                    fileForSettings << '\n';
                }
            }

            fileForSettings.close();


            Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
//            std::cout << "direct matching consecutive: " << std::endl;
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


    Eigen::Matrix4d
    createVoxelWithFollowingRandomChanges(double voxelData[], Eigen::Matrix4d movePosition, int typeOfMatching,
                                          int numberOfPoints, double percentageSaltAndPepper,
                                          double stdDivGaussianNoise, double percentageOcclusions,
                                          std::ofstream outShifted, int indexStart, int indexEnd) {

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
            for (int k = 0; k < numberOfPoints; k++) {
                outShifted << voxelData1[j + numberOfPoints * k] << ',';
            }
            outShifted << '\n';
        }
//        outShifted.close();


        return Eigen::Matrix4d::Identity();
    }


//    pcl::PointCloud<pcl::PointXYZ>
//    convertVoxelToPointcloud(double voxelData[], double thresholdFactor, double maximumVoxelData, int dimensionVoxel) {
//
//        pcl::PointCloud<pcl::PointXYZ> ourPCL;
//        for (int j = 0; j < dimensionVoxel; j++) {
//            for (int k = 0; k < dimensionVoxel; k++) {
//                if (voxelData[j + dimensionVoxel * k] > maximumVoxelData * thresholdFactor) {
//                    double xPosPoint = (j - dimensionVoxel / 2.0) * DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                                       ((double) dimensionVoxel);
//                    double yPosPoint = (k - dimensionVoxel / 2.0) * DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                                       ((double) dimensionVoxel);
//                    //mix X and Y for enu to ned
//                    ourPCL.push_back(pcl::PointXYZ(yPosPoint, xPosPoint, 0));
//
//                }
//            }
//        }
//
//
//        return ourPCL;
//    }

    void
    addNoiseToVoxel(double voxelData[], double stdDiviationGaussPercent, double maximumVoxelData, int dimensionVoxel) {
        std::random_device rd;

        std::mt19937 gen(rd());

        std::uniform_real_distribution<> dis(

                0.0, 1.0);

        std::normal_distribution<> disNormal(0.0, stdDiviationGaussPercent * maximumVoxelData);


        for (int j = 0; j < dimensionVoxel; j++) {
            for (int k = 0; k < dimensionVoxel; k++) {
                voxelData[j + dimensionVoxel * k] += disNormal(gen);
            }
        }
    }

    void addSaltPepperNoiseToVoxel(double voxelData[], double saltPepperNoisePercent, double maximumVoxelData,
                                   int dimensionVoxel) {
        std::random_device rd;

        std::mt19937 gen(rd());

        std::uniform_real_distribution<> dis(

                0.0, 1.0);

        std::normal_distribution<> disNormal(0.0, 1);


        for (int j = 0; j < dimensionVoxel; j++) {
            for (int k = 0; k < dimensionVoxel; k++) {
                if (saltPepperNoisePercent > dis(gen)) {
                    voxelData[j + dimensionVoxel * k] = dis(gen) * maximumVoxelData;
                }
            }
        }
    }


    Eigen::Matrix4d randomShiftAndRotationCalculation(double xyRange, double rotationRange,bool onlyOneRotationDirection) {

        std::random_device rd;

        std::mt19937 gen(rd());

        std::uniform_real_distribution<> disXY(
                -xyRange, xyRange);

        std::uniform_real_distribution<> disAngle(

                -rotationRange, rotationRange);
        if(onlyOneRotationDirection){
            disAngle = std::uniform_real_distribution<>(0,rotationRange);
        }







        double xDiff = disXY(gen);
        double yDiff = disXY(gen);
        double angleDiff = disAngle(gen);


        Eigen::Matrix4d returnMatrix = Eigen::Matrix4d::Identity();

        returnMatrix.block<3, 3>(0, 0) = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                             angleDiff).block<3, 3>(0,
                                                                                                                    0);
        returnMatrix(0, 3) = xDiff;
        returnMatrix(1, 3) = yDiff;

        return returnMatrix;
    }

    double applyContourToVoxelData(double voxelData[], double angle, Eigen::Vector2d shift, int pattern, double scaling,
                                   int dimensionPoints, double randomOcclusionParameter) {



        //square
        std::vector<cv::Point2f> testPointsCV = getPattern(scaling, randomOcclusionParameter, pattern, angle,
                                                           shift, dimensionPoints);


        int numberOfPointsRemoved = 0;
        for (int j = 0; j < dimensionPoints; j++) {
            for (int k = 0; k < dimensionPoints; k++) {
                int indexX =
                        (int) (j - dimensionPoints / 2);
                int indexY =
                        (int) (k - dimensionPoints / 2);
                if (cv::pointPolygonTest(testPointsCV, cv::Point2f(indexX, indexY), false) == 1) {
                    voxelData[j + dimensionPoints * k] = 0;
                    numberOfPointsRemoved++;
                }
            }
        }
//        std::cout << "percentage removed" << std::endl;
//        std::cout <<  << std::endl;
        return (double) numberOfPointsRemoved / (double) dimensionPoints / (double) dimensionPoints;
//        std::cout << cv::pointPolygonTest(testPointsCV,cv::Point2f(0,0),false) << std::endl;
//        std::cout << cv::pointPolygonTest(testPointsCV,cv::Point2f(30,30),false) << std::endl;
//        std::cout << cv::pointPolygonTest(testPointsCV,cv::Point2f(0.5,5),false) << std::endl;
    }

    double calculateOverlap(double angle1, double angle2, int pattern1, double pattern2, Eigen::Vector2d shift1,
                            Eigen::Vector2d shift2, double scaling1, double scaling2, double randomOcclusionParameter1,
                            double randomOcclusionParameter2, int dimensionPoints) {

        std::vector<cv::Point2f> testPoints1 = getPattern(scaling1, randomOcclusionParameter1, pattern1, angle1, shift1,
                                                          dimensionPoints);
        std::vector<cv::Point2f> testPoints2 = getPattern(scaling2, randomOcclusionParameter2, pattern2, angle2, shift2,
                                                          dimensionPoints);

        int overlapParameter = 0;
        for (int j = 0; j < dimensionPoints; j++) {
            for (int k = 0; k < dimensionPoints; k++) {
                int indexX =
                        (int) (j - dimensionPoints / 2);
                int indexY =
                        (int) (k - dimensionPoints / 2);
                int tmp = 0;
                if (cv::pointPolygonTest(testPoints1, cv::Point2f(indexX, indexY), false) == 1) {
                    tmp++;
                }
                if (cv::pointPolygonTest(testPoints2, cv::Point2f(indexX, indexY), false) == 1) {
                    tmp++;
                }
                if (tmp > 0) {
                    overlapParameter++;
                }
            }
        }

        return ((double) overlapParameter) / ((double) (dimensionPoints * dimensionPoints));
    }

    std::vector<cv::Point2f>
    getPattern(double scaling, double randomOcclusionParameter, int pattern, double angle, Eigen::Vector2d shift,
               int dimensionPoints) {
        double scalingDirect = scaling * dimensionPoints / 2;
        std::vector<Eigen::Vector2d> testPoints;
        //gleich schenkliges dreieck
        if (pattern == 0) {
            std::vector<Eigen::Vector2d> testPoints2{Eigen::Vector2d(0, scalingDirect),
                                                     Eigen::Vector2d(
                                                             cos(2.0 * M_PI / 3.0 - 2.0 * M_PI / 4.0) * scalingDirect,
                                                             -sin(2.0 * M_PI / 3.0 - 2.0 * M_PI / 4.0) * scalingDirect),
                                                     Eigen::Vector2d(
                                                             -cos(2.0 * M_PI / 3.0 - 2.0 * M_PI / 4.0) * scalingDirect,
                                                             -sin(2.0 * M_PI / 3.0 - 2.0 * M_PI / 4.0) *
                                                             scalingDirect)};
            testPoints = testPoints2;
        }

        if (pattern == 1) {
            std::vector<Eigen::Vector2d> testPoints2{Eigen::Vector2d(scalingDirect, scalingDirect),
                                                     Eigen::Vector2d(
                                                             -scalingDirect,
                                                             scalingDirect),
                                                     Eigen::Vector2d(
                                                             -scalingDirect,
                                                             -scalingDirect),
                                                     Eigen::Vector2d(
                                                             scalingDirect,
                                                             -scalingDirect)};
            testPoints = testPoints2;
        }

        if (pattern == 2) {

            double numberForCross = randomOcclusionParameter * (15 - 0.8) + 0.8;//0.8-15
            std::vector<Eigen::Vector2d> testPoints2{Eigen::Vector2d(scalingDirect / numberForCross, scalingDirect),
                                                     Eigen::Vector2d(-scalingDirect / numberForCross, scalingDirect),
                                                     Eigen::Vector2d(-scalingDirect / numberForCross,
                                                                     scalingDirect / numberForCross),
                                                     Eigen::Vector2d(-scalingDirect, scalingDirect / numberForCross),
                                                     Eigen::Vector2d(-scalingDirect, -scalingDirect / numberForCross),
                                                     Eigen::Vector2d(-scalingDirect / numberForCross,
                                                                     -scalingDirect / numberForCross),
                                                     Eigen::Vector2d(-scalingDirect / numberForCross, -scalingDirect),
                                                     Eigen::Vector2d(scalingDirect / numberForCross, -scalingDirect),
                                                     Eigen::Vector2d(scalingDirect / numberForCross,
                                                                     -scalingDirect / numberForCross),
                                                     Eigen::Vector2d(scalingDirect, -scalingDirect / numberForCross),
                                                     Eigen::Vector2d(scalingDirect, scalingDirect / numberForCross),
                                                     Eigen::Vector2d(scalingDirect / numberForCross,
                                                                     scalingDirect / numberForCross)};
            testPoints = testPoints2;
        }

        //rotation
        Eigen::Matrix2d rotationMatrixEigen;


        rotationMatrixEigen(0, 0) = cos(angle);
        rotationMatrixEigen(1, 0) = sin(angle);
        rotationMatrixEigen(0, 1) = -sin(angle);
        rotationMatrixEigen(1, 1) = cos(angle);
        std::cout << rotationMatrixEigen << std::endl;
        for (auto &tmp: testPoints) {
            tmp = shift * dimensionPoints / 2.0 + rotationMatrixEigen * tmp;
        }
        std::vector<cv::Point2f> testPointsCV;
        for (auto tmp: testPoints) {
            testPointsCV.push_back(cv::Point2f(tmp[0], tmp[1]));
        }
        return testPointsCV;
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
