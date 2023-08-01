//
// Created by jurobotics on 13.09.21.
//


#include "geometry_msgs/PoseStamped.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include "nav_msgs/OccupancyGrid.h"
//#include "scanRegistrationClass.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/SetBoolRequest.h"
#include <std_srvs/SetBool.h>
#include "commonbluerovmsg/staterobotforevaluation.h"

#define NUMBER_OF_POINTS_DIMENSION 256
#define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 50 // was 50 //tuhh tank 6
#define NUMBER_OF_POINTS_MAP 512//was 512
// 80 simulation 300 valentin 45.0 for Keller 10.0 TUHH TANK
#define DIMENSION_OF_MAP 80.0

#define IGNORE_DISTANCE_TO_ROBOT 0.2 // was 1.0 // TUHH 0.2
#define DEBUG_REGISTRATION false

#define ROTATION_SONAR 0 // sonar on robot M_PI // simulation 0
#define SHOULD_USE_ROSBAG true
#define FACTOR_OF_MATCHING 1.0 //1.5
#define THRESHOLD_FOR_TRANSLATION_MATCHING 0.1 // standard is 0.1, 0.05 und 0.01  // 0.05 for valentin Oben

#define INTEGRATED_NOISE_XY 0.005 // was 0.03  // TUHH 0.005
#define INTEGRATED_NOISE_YAW 0.005 // was 0.03 // TUHH 0.005

#define USE_INITIAL_TRANSLATION_LOOP_CLOSURE true
#define MAXIMUM_LOOP_CLOSURE_DISTANCE 0.2 // 0.2 TUHH 2.0 valentin Keller 4.0 Valentin Oben // 2.0 simulation
#define FACTOR_OF_THRESHOLD 0.4

#define TUHH_SYSTEM false
#define SIMULATION_SYSTEM true


#define HOME_LOCATION "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultsJournalFMS2D/resultsConsecutiveScans/"


#define NAME_OF_CURRENT_METHOD "GICP"
//#define NAME_OF_CURRENT_METHOD "_circle_dead_reckoning_wsm_"
//#define NAME_OF_CURRENT_METHOD "_circle_dynamic_slam_1_0_"
//#define NAME_OF_CURRENT_METHOD "_circle_dynamic_slam_4_0_"
//#define NAME_OF_CURRENT_METHOD "_video_4_0_"

//#define NAME_OF_CURRENT_METHOD "_TEST2_classical_slam_"



#define INVERSE_RESULT 1
#define WHICH_METHOD 1
#define USING_INITIAL_GUESS_CHANGE_VALENTIN 0
#define IGNORE_FIRST_STEPS 0
//occupancyMap(256, NUMBER_OF_POINTS_DIMENSION, 70, hilbertMap::HINGED_FEATURES)
class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_, const std::string &nameOfTheServiceCall) : graphSaved(3, INTENSITY_BASED_GRAPH),
                                                                               scanRegistrationObject(
                                                                                       NUMBER_OF_POINTS_DIMENSION,
                                                                                       NUMBER_OF_POINTS_DIMENSION / 2,
                                                                                       NUMBER_OF_POINTS_DIMENSION / 2,
                                                                                       NUMBER_OF_POINTS_DIMENSION / 2 -
                                                                                       1) {

        this->subscriberEKF = n_.subscribe("publisherPoseEkf", 10000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(2).sleep();
        this->subscriberIntensitySonar = n_.subscribe("sonar/intensity", 10000, &rosClassEKF::scanCallback, this);
        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);
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
    }


private:
    nav_msgs::OccupancyGrid map;


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
//        std::cout << intensityTMP.angle << std::endl;
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
//        std::cout << "huhu3: " << msg->header.stamp.toSec() << std::endl;
        bool waitingForMessages = waitForEKFMessagesToArrive(msg->header.stamp.toSec());
        if (!waitingForMessages) {
            return;
        }
//        std::cout << "huhu4" << std::endl;
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
//        std::cout << angleDiff << std::endl;
        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (abs(angleDiff) > 2 * M_PI / FACTOR_OF_MATCHING) {

//            std::cout << "tmp0: " << std::endl;
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
//
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//            std::cout << "tmp1: " << std::endl;
            //angleDiff = angleBetweenLastKeyframeAndNow(false);
//            indexOfLastKeyframe = slamToolsRos::getLastIntensityKeyframe(this->graphSaved);
            int indexStart1, indexEnd1, indexStart2, indexEnd2;
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);


            std::cout << "scanAcusitionTime: " << this->graphSaved.getVertexList()->at(indexStart2).getTimeStamp() -
                                                  this->graphSaved.getVertexList()->at(indexEnd2).getTimeStamp()
                      << std::endl;

            //we inverse the initial guess, because the registration creates a T from scan 1 to scan 2.
            // But the graph creates a transformation from 1 -> 2 by the robot, therefore inverse.
            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart1).getTransformation());

//            std::cout << "this->initialGuessTransformation.inverse()" << std::endl;
//            std::cout << this->initialGuessTransformation.inverse() << std::endl;

            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));

            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);

            double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());//get voxel


            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2, indexStart2, indexEnd2,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());//get voxel

            Eigen::Matrix3d covarianceEstimation = Eigen::Matrix3d::Zero();
            std::cout << "direct matching consecutive: " << std::endl;
            // result is matrix to transform scan 1 to scan 2 therefore later inversed + initial guess inversed

            pcl::PointCloud<pcl::PointXYZ> scan1Threshold = slamToolsRos::convertVoxelToPointcloud(voxelData1, 0.4,
                                                                                                   maximumVoxel1,
                                                                                                   NUMBER_OF_POINTS_DIMENSION,
                                                                                                   DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);
            pcl::PointCloud<pcl::PointXYZ> scan2Threshold = slamToolsRos::convertVoxelToPointcloud(voxelData2, 0.4,
                                                                                                   maximumVoxel1,
                                                                                                   NUMBER_OF_POINTS_DIMENSION,
                                                                                                   DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);
            pcl::PointCloud<pcl::PointXYZ> final;

            //@TODO here: add different Registration Methods
//            this->currentEstimatedTransformation = slamToolsRos::registrationOfTwoVoxels(voxelData1, voxelData2,
//                                                                                         this->initialGuessTransformation,
//                                                                                         covarianceEstimation, true,
//                                                                                         true,
//                                                                                         (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                                                                                         (double) NUMBER_OF_POINTS_DIMENSION,
//                                                                                         false, scanRegistrationObject,
//                                                                                         DEBUG_REGISTRATION,
//                                                                                         THRESHOLD_FOR_TRANSLATION_MATCHING);


            this->currentEstimatedTransformation = slamToolsRos::registrationOfDesiredMethod(scan1Threshold,
                                                                                             scan2Threshold,
                                                                                             final, voxelData1,
                                                                                             voxelData2,
                                                                                             this->initialGuessTransformation,
                                                                                             (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                                             (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                             WHICH_METHOD, true,
                                                                                             scanRegistrationObject);


            pcl::io::savePCDFileASCII(
                    "/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/final/firstPCL.pcd",
                    scan1Threshold);
            pcl::io::savePCDFileASCII(
                    "/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/final/secondPCL.pcd",
                    scan2Threshold);
            pcl::io::savePCDFileASCII(
                    "/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/final/aligned.pcd", final);


            if (INVERSE_RESULT) {
                Eigen::Matrix4d forInverseMatrix = this->currentEstimatedTransformation.inverse();
                this->currentEstimatedTransformation = forInverseMatrix;

            }


            slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1, indexEnd1, indexStart2, indexEnd2,
                                                           this->graphSaved, NUMBER_OF_POINTS_DIMENSION,
                                                           IGNORE_DISTANCE_TO_ROBOT,
                                                           DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                           DEBUG_REGISTRATION, this->currentEstimatedTransformation,
                                                           initialGuessTransformation);

//            std::cout << "this->currentEstimatedTransformation inv:" << std::endl;

//            std::cout << this->currentEstimatedTransformation.inverse() << std::endl;

//            slamToolsRos::saveResultingRegistration(indexStart1, indexStart2,
//                                                    this->graphSaved, NUMBER_OF_POINTS_DIMENSION,
//                                                    IGNORE_DISTANCE_TO_ROBOT, DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
//                                                    DEBUG_REGISTRATION, this->currentTransformation);

            // @TODO calculate difference between GT and Registration and Initial Guess


            this->graphSaved.getVertexList()->at(indexEnd1).getGroundTruthTransformation();

            this->graphSaved.getVertexList()->at(indexEnd2).getGroundTruthTransformation();
//            indexEnd1
//            indexEnd2

            Eigen::Matrix4d GTTransformation =
                    (this->graphSaved.getVertexList()->at(indexStart2).getGroundTruthTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart1).getGroundTruthTransformation());
            initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                           this->initialGuessTransformation(0, 0));
            double estimatedAngle = std::atan2(this->currentEstimatedTransformation(1, 0),
                                               this->currentEstimatedTransformation(0, 0));
            double gtAngle = std::atan2(GTTransformation(1, 0),
                                        GTTransformation(0, 0));

            std::cout << this->initialGuessTransformation << std::endl;
            std::cout << ":" << std::endl;
            std::cout << this->currentEstimatedTransformation << std::endl;
            std::cout << ":" << std::endl;
            std::cout << GTTransformation << std::endl;
            double IGError = (GTTransformation.block<3, 1>(0, 3) -
                              this->initialGuessTransformation.block<3, 1>(0, 3)).norm();
            double eTError = (GTTransformation.block<3, 1>(0, 3) -
                              this->currentEstimatedTransformation.block<3, 1>(0, 3)).norm();
            double differenceAngleIG = generalHelpfulTools::angleDiff(gtAngle, initialGuessAngle);
            double differenceAngleEstimation = generalHelpfulTools::angleDiff(gtAngle, estimatedAngle);


            std::cout << IGError << std::endl;
            std::cout << eTError << std::endl;
            std::cout << differenceAngleIG << std::endl;
            std::cout << differenceAngleEstimation << std::endl;

            // @TODO Create something to save GT Transformations etc.

            std::ofstream fileForSettings;
            fileForSettings.open(
                    std::string(HOME_LOCATION) + std::string(NAME_OF_CURRENT_METHOD) + std::string("_error.csv"),
                    std::ios_base::app);
            fileForSettings << IGError << "," << eTError << "," << differenceAngleIG << "," << differenceAngleEstimation
                            << '\n';
            fileForSettings.close();


            slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                    this->publisherMarkerArray, this->sigmaScaling,
                                                    this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);
            //            this->graphSaved.classicalOptimizeGraph(true);
            std::cout << "next: " << std::endl;

            free(voxelData1);
            free(voxelData2);
            if (SHOULD_USE_ROSBAG) {
                std_srvs::SetBool srv;
                srv.request.data = false;
                pauseRosbag.call(srv);
            }

        }
//        this->graphSaved.isam2OptimizeGraph(true,1);
        slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                this->publisherMarkerArray, this->sigmaScaling,
                                                this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);
//        std::cout << "huhu3" << std::endl;
    }

    void stateEstimationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
        double currentTimeStamp = msg->header.stamp.toSec();
//        std::cout << currentTimeStamp << std::endl;
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

    bool saveGraph(commonbluerovmsg::saveGraph::Request &req, commonbluerovmsg::saveGraph::Response &res) {
        this->createMapAndSaveToFile();
        std::cout << "test for saving1" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        std::cout << "test for saving2" << std::endl;

        std::ofstream myFile1, myFile2;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/positionEstimationOverTime" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        myFile2.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/groundTruthOverTime" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");


        for (int k = 0; k < this->graphSaved.getVertexList()->size(); k++) {

            Eigen::Matrix4d tmpMatrix1 = this->graphSaved.getVertexList()->at(k).getTransformation();
            Eigen::Matrix4d tmpMatrix2 = this->graphSaved.getVertexList()->at(k).getGroundTruthTransformation();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    myFile1 << tmpMatrix1(i, j) << " ";//number of possible rotations
                }
                myFile1 << "\n";
            }


            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    myFile2 << tmpMatrix2(i, j) << " ";//number of possible rotations
                }
                myFile2 << "\n";
            }

        }


        myFile1.close();
        myFile2.close();

        res.saved = true;
        return true;
    }

    bool waitForEKFMessagesToArrive(double timeUntilWait) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        while (this->timeVector.empty() && timeToCalculate < 10) {
//            std::cout << "tmp1: " <<  this->timeVector.empty() <<std::endl;
            ros::Duration(0.002).sleep();
            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }
//        std::cout << "tmp2" << std::endl;
//        std::cout << "tmp2: " <<  timeUntilWait << " : " << timeVector[timeVector.size() - 1] <<std::endl;

        while (timeUntilWait > timeVector[timeVector.size() - 1]) {
//            std::cout << "tmp2: " <<  timeUntilWait << " : " << timeVector[timeVector.size() - 1] <<std::endl;

            ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf", ros::Duration(10));
            ros::Duration(0.001).sleep();

            end = std::chrono::steady_clock::now();
            timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//            std::cout << timeToCalculate << std::endl;
            if (timeToCalculate > 10) {
//                std::cout << "here we break" << std::endl;
                break;
            }
        }
//        std::cout << "tmp3: " <<  timeUntilWait << " : " << timeVector[timeVector.size() - 1] <<std::endl;
        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
//        ros::Duration(0.002).sleep();
//        std::cout << "tmp4" << std::endl;
        end = std::chrono::steady_clock::now();
        timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
//        std::cout << timeToCalculate << std::endl;

//        if(timeToCalculate>8){
//            return false;
//        }else{
//            return true;
//        }
        if (timeUntilWait < timeVector[timeVector.size() - 1]) {
            return true;
        } else {
            return false;
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
//            std::cout << currentTimeStampOfInterest << std::endl;
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

    void createMapAndSaveToFile() {

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


        std::ofstream myFile1;
        myFile1.open(
                "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/currentMap" +
                std::string(NAME_OF_CURRENT_METHOD) + ".csv");
        for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
            for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
                myFile1 << mapData[j + NUMBER_OF_POINTS_MAP * i] << std::endl;//number of possible rotations
            }
        }

        myFile1.close();


    }


public:

    void createImageOfAllScans(double mapData[], int howOftenSkipping) {
        //homePosition is 0 0
        //size of mapData is defined in NUMBER_OF_POINTS_MAP

//        int whichMethod = WHICH_METHOD;
        int ignoreFirstNSteps = IGNORE_FIRST_STEPS;
        double correctionFactorMatchingYaw = 0;// M_PI/2.0 at real data
        double correctionFactorMatchingRoll = 0.001;// M_PI/2.0 at real data
        double correctionFactorCreationMapYaw = 0;// M_PI/2.0 at real data
        double correctionFactorCreationRoll = 0;// M_PI at real data


        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }


        //get array of all created connections of matching estimations
        std::vector<edge> listOfRegistratedEdges;
        for (const edge &tmpEdgeFromList: *this->graphSaved.getEdgeList()) {
            if (tmpEdgeFromList.getTypeOfEdge() == LOOP_CLOSURE) {
                edge tmpEdge(tmpEdgeFromList);
                listOfRegistratedEdges.push_back(tmpEdge);
            }
        }

//        int startPoint = 0;
        int startPoint = listOfRegistratedEdges.size() / 2;
        int lastPositionOfInterest = startPoint;
        //missing: just add the point cloud to map at startPoint
        Eigen::Matrix4d completeTransformationForCurrentScan = Eigen::Matrix4d::Identity();

        for (int currentEdgePosition = startPoint;
             currentEdgePosition < listOfRegistratedEdges.size(); currentEdgePosition =
                                                                          currentEdgePosition + howOftenSkipping) {


            int indexFirstPCL = listOfRegistratedEdges[lastPositionOfInterest].getToKey();
            int indexSecondPCL = listOfRegistratedEdges[currentEdgePosition].getToKey();

            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(indexFirstPCL).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->at(indexSecondPCL).getTransformation();
            if (currentEdgePosition == startPoint) {
                this->currentEstimatedTransformation = Eigen::Matrix4d::Identity();
            } else {
//                std::cout << "init guess angle: "
//                          << std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0))
//                          << std::endl;




//                this->currentEstimatedTransformation = slamToolsRos::registrationOfDesiredMethod(scan1Threshold,
//                                                                                                 scan2Threshold,
//                                                                                                 final, voxelData1,
//                                                                                                 voxelData2,
//                                                                                                 this->initialGuessTransformation,
//                                                                                                 (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                                                                                                 (double) NUMBER_OF_POINTS_DIMENSION,
//                                                                                                 WHICH_METHOD, true,
//                                                                                                 scanRegistrationObject);
                this->currentEstimatedTransformation = this->calculateRegistration(this->initialGuessTransformation,
                                                                                   WHICH_METHOD,
                                                                                   indexFirstPCL, indexSecondPCL,
                                                                                   correctionFactorMatchingYaw,
                                                                                   correctionFactorMatchingRoll);

                std::cout << "init guess:" << std::endl;
                std::cout << this->initialGuessTransformation << std::endl;
                std::cout << "current Transformation:" << std::endl;
                std::cout << this->currentEstimatedTransformation << std::endl;

            }


            completeTransformationForCurrentScan =
                    completeTransformationForCurrentScan * this->currentEstimatedTransformation;
//            std::cout << "complete:" << std::endl;
//            std::cout << completeTransformationForCurrentScan << std::endl;

            // method to create PCL in Map







            int indexStart = indexSecondPCL;
            this->mapCalculation(indexStart, completeTransformationForCurrentScan, correctionFactorCreationRoll,
                                 correctionFactorCreationMapYaw,
                                 voxelDataIndex, mapData);


            //make sure next iteration the correct registrationis calculated
            lastPositionOfInterest = currentEdgePosition;
        }
        //TO THE END
        //NOW: TO THE BEGINNING


        lastPositionOfInterest = startPoint;

        completeTransformationForCurrentScan = Eigen::Matrix4d::Identity();

        for (int currentEdgePosition = startPoint - howOftenSkipping;
             currentEdgePosition > ignoreFirstNSteps; currentEdgePosition = currentEdgePosition - howOftenSkipping) {
            int indexFirstPCL = listOfRegistratedEdges[currentEdgePosition].getToKey();
            int indexSecondPCL = listOfRegistratedEdges[lastPositionOfInterest].getToKey();

            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(indexFirstPCL).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->at(indexSecondPCL).getTransformation();


            this->currentEstimatedTransformation = this->calculateRegistration(
                    this->initialGuessTransformation, WHICH_METHOD, indexFirstPCL, indexSecondPCL,
                    correctionFactorMatchingYaw, correctionFactorMatchingRoll);


            std::cout << "init guess:" << std::endl;
            std::cout << this->initialGuessTransformation << std::endl;
            std::cout << "current Transformation:" << std::endl;
            std::cout << this->currentEstimatedTransformation << std::endl;
//            this->currentTransformation(0,3) = -this->currentTransformation(0,3);
//            std::cout <<"current Transformation:" << std::endl;
//            std::cout << this->currentTransformation << std::endl;
            completeTransformationForCurrentScan =
                    completeTransformationForCurrentScan * this->currentEstimatedTransformation.inverse();
            int indexStart = indexFirstPCL;
            this->mapCalculation(indexStart, completeTransformationForCurrentScan, correctionFactorCreationRoll,
                                 correctionFactorCreationMapYaw,
                                 voxelDataIndex, mapData);

            //make sure next iteration the correct registrationis calculated
            lastPositionOfInterest = currentEdgePosition;
        }


        double maximumOfVoxelData = 0;
        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
            if (voxelDataIndex[i] > 0) {
                mapData[i] = mapData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < mapData[i]) {
                    maximumOfVoxelData = mapData[i];
                }
                //std::cout << voxelData[i] << std::endl;
            }
        }// @TODO calculate the maximum and normalize "somehow"



        nav_msgs::OccupancyGrid occupanyMap;
        occupanyMap.info.height = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.width = NUMBER_OF_POINTS_MAP;
        occupanyMap.info.resolution = DIMENSION_OF_MAP / NUMBER_OF_POINTS_MAP;
        occupanyMap.info.origin.position.x = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.y = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.z = +0.5;
        for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
                //determine color:
                occupanyMap.data.push_back((int) (mapData[j + NUMBER_OF_POINTS_MAP * i] * 2));
            }
        }
        publisherOccupancyMap.publish(occupanyMap);


        free(voxelDataIndex);
    }

    void mapCalculation(int indexStart, Eigen::Matrix4d &completeTransformationForCurrentScan,
                        double correctionFactorCreationRoll, double correctionFactorCreationMap,
                        int voxelDataIndex[], double mapData[]) {

        int i = 0;
        do {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.


            //get position of current intensityRay
//                Eigen::Matrix4d transformationOfIntensityRay =
//                        this->graphSaved.getVertexList()->at(indexStart - i).getTransformation().inverse() *
//                        this->graphSaved.getVertexList()->at(indexStart).getTransformation();

            Eigen::Matrix4d transformationOfIntensityRay =
                    this->graphSaved.getVertexList()->at(indexStart).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->at(indexStart - i).getTransformation();
            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             this->graphSaved.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);

            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT / (this->graphSaved.getVertexList()->at(
                    indexStart - i).getIntensities().range / ((double) this->graphSaved.getVertexList()->at(
                    indexStart - i).getIntensities().intensities.size())));


            for (int j = ignoreDistance;
                 j <
                 this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) this->graphSaved.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().range);

                int incrementOfScan = this->graphSaved.getVertexList()->at(
                        indexStart - i).getIntensities().increment;
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

                    positionOfIntensity = completeTransformationForCurrentScan *
                                          generalHelpfulTools::getTransformationMatrixFromRPY(
                                                  correctionFactorCreationRoll, 0, correctionFactorCreationMap) *
                                          transformationOfIntensityRay *
                                          rotationOfSonarAngleMatrix * positionOfIntensity;
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
                                this->graphSaved.getVertexList()->at(
                                        indexStart - i).getIntensities().intensities[j];
                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                        //                    std::cout << "random: " << std::endl;
                    }
                }
            }
            i++;
        } while (this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);

    }


    Eigen::Matrix4d
    calculateRegistration(Eigen::Matrix4d &ourInitialGuess, int whichMethod, int indexFirstPCL, int indexSecondPCL,
                          double correctionFactorMatchingYaw, double correctionFactorPCLMatchingRoll) {
        if (USING_INITIAL_GUESS_CHANGE_VALENTIN) {
            Eigen::Matrix4d initialGuessTMP = ourInitialGuess;

            ourInitialGuess(0, 3) = initialGuessTMP(1, 3);
            ourInitialGuess(1, 3) = -initialGuessTMP(0, 3);
        }


        double *voxelData1;
        double *voxelData2;
        voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
        voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);


        pcl::PointCloud<pcl::PointXYZ> final;
        double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                indexFirstPCL,
                                                                generalHelpfulTools::getTransformationMatrixFromRPY(
                                                                        0, 0,
                                                                        correctionFactorMatchingYaw),
                                                                NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                IGNORE_DISTANCE_TO_ROBOT,
                                                                DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get
        // voxel
        double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                                indexSecondPCL,
                                                                generalHelpfulTools::getTransformationMatrixFromRPY(
                                                                        0, 0,
                                                                        correctionFactorMatchingYaw),
                                                                NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                IGNORE_DISTANCE_TO_ROBOT,
                                                                DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(correctionFactorPCLMatchingRoll,
                                                                                        0,
                                                                                        correctionFactorMatchingYaw);
        pcl::PointCloud<pcl::PointXYZ> cloudFirstScan;
        pcl::PointCloud<pcl::PointXYZ> cloudSecondScan;


        cloudFirstScan = slamToolsRos::createPCLFromGraphOnlyThreshold(indexFirstPCL, tmpMatrix,
                                                                       this->graphSaved,
                                                                       IGNORE_DISTANCE_TO_ROBOT,
                                                                       FACTOR_OF_THRESHOLD);

        cloudSecondScan = slamToolsRos::createPCLFromGraphOnlyThreshold(indexSecondPCL, tmpMatrix,
                                                                        this->graphSaved,
                                                                        IGNORE_DISTANCE_TO_ROBOT,
                                                                        FACTOR_OF_THRESHOLD);
//        cloudFirstScan = createPCLFromGraphOneValue(indexFirstPCL, tmpMatrix);
//        cloudSecondScan = createPCLFromGraphOneValue(indexSecondPCL, tmpMatrix);




        Eigen::Matrix4d Tout = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.1);

//        Tout(1,3) = 0.7;
//        Tout(0,3) = -0.2;
//        std::cout << "Tout" << std::endl;
//        std::cout << Tout << std::endl;
//        pcl::transformPointCloud(cloudFirstScan, cloudSecondScan, Tout);
//         ourInitialGuess = Eigen::Matrix4d::Identity();


        pcl::io::savePCDFileASCII("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.pcd",
                                  cloudFirstScan);
        pcl::io::savePCDFileASCII("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.pcd",
                                  cloudSecondScan);


        double fitnessScoreX;

        this->currentEstimatedTransformation = slamToolsRos::registrationOfDesiredMethod(cloudFirstScan,
                                                                                         cloudSecondScan,
                                                                                         final, voxelData1,
                                                                                         voxelData2,
                                                                                         this->initialGuessTransformation,
                                                                                         (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                                         (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                         WHICH_METHOD, true,
                                                                                         scanRegistrationObject);


        if (DEBUG_REGISTRATION) {
            double maximumVoxel1tmp = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                       indexFirstPCL,
                                                                       this->currentEstimatedTransformation.inverse() *
                                                                       generalHelpfulTools::getTransformationMatrixFromRPY(
                                                                               0, 0,
                                                                               correctionFactorMatchingYaw),
                                                                       NUMBER_OF_POINTS_DIMENSION,
                                                                       this->graphSaved,
                                                                       IGNORE_DISTANCE_TO_ROBOT,
                                                                       DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get
//            double maximumVoxel1tmp = createVoxelOfGraph(voxelData1,
//                                                         indexFirstPCL,
//                                                         ourInitialGuess.inverse() * generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
//                                                                                                                                           correctionFactorMatchingYaw),
//                                                         NUMBER_OF_POINTS_DIMENSION);//get


            double maximumVoxel2tmp = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                                       indexSecondPCL,

                                                                       generalHelpfulTools::getTransformationMatrixFromRPY(
                                                                               0, 0,
                                                                               correctionFactorMatchingYaw),
                                                                       NUMBER_OF_POINTS_DIMENSION,
                                                                       this->graphSaved,
                                                                       IGNORE_DISTANCE_TO_ROBOT,
                                                                       DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

            std::ofstream myFile1, myFile2;
            myFile1.open(
                    "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel1.csv");
            myFile2.open(
                    "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel2.csv");
            for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
                for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
                    myFile1 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // real part
                    myFile1 << "\n";
                    myFile2 << voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
                    myFile2 << "\n";
                }
            }
            myFile1.close();
            myFile2.close();
        }

        return this->currentEstimatedTransformation;
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


    ros::spin();


//    ros::Rate loop_rate(0.1);
//    ros::AsyncSpinner spinner(4); // Use 4 threads
//    spinner.start();
//    ros::Duration(10).sleep();

//    while (ros::ok()) {
//    ros::spinOnce();
//
    //rosClassForTests.updateHilbertMap();
//        rosClassForTests.updateMap();
//        rosClassForTests.createImageOfAllScans();

//        loop_rate.sleep();
//
//        //std::cout << ros::Time::now() << std::endl;
//    }


    return (0);
}
