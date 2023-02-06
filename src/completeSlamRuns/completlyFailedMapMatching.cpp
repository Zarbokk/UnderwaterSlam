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


#define NUMBER_OF_POINTS_DIMENSION 128
#define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 50 // was 50
// 80 simulation 120 valentin 45.0 for Keller
#define DIMENSION_OF_MAP 80.0

//#define this->numberOfPointsMap 512
#define IGNORE_DISTANCE_TO_ROBOT 1.0
#define DEBUG_REGISTRATION false

#define ROTATION_SONAR 0 // sonar on robot M_PI // simulation 0
#define SHOULD_USE_ROSBAG false
#define FACTOR_OF_MATCHING 4.0 //1.5
#define THRESHOLD_FOR_TRANSLATION_MATCHING 0.1 // standard is 0.1, 0.05 und 0.01 sorgt fuer

#define INTEGRATED_NOISE_XY 0.03 // was 0.03
#define INTEGRATED_NOISE_YAW 0.02 // was 0.03

#define USE_INITIAL_TRANSLATION_LOOP_CLOSURE true
#define MAXIMUM_LOOP_CLOSURE_DISTANCE 10.0

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
        this->cellSize = (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                         (double) NUMBER_OF_POINTS_DIMENSION;
        this->numberOfPointsMap = ceil(DIMENSION_OF_MAP / this->cellSize);
        subscriberEKF = n_.subscribe("publisherPoseEkf", 10000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(2).sleep();
        this->subscriberIntensitySonar = n_.subscribe("sonar/intensity", 10000, &rosClassEKF::scanCallback, this);
        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);

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
        this->maxTimeOptimization = 1.0;


//        map.info.height = this->numberOfPointsMap;
//        map.info.width = this->numberOfPointsMap;
//        map.info.resolution = this->cellSize;
//        map.info.origin.position.x = -this->numberOfPointsMap / 2;
//        map.info.origin.position.y = -this->numberOfPointsMap / 2;
//        map.info.origin.position.z = +0.5;
//        for (int i = 0; i < this->numberOfPointsMap; i++) {
//            for (int j = 0; j < this->numberOfPointsMap; j++) {
//                //determine color:
//                map.data.push_back(50);
//            }
//        }
    }


private:
//    nav_msgs::OccupancyGrid map;


    ros::Subscriber subscriberEKF, subscriberIntensitySonar;
    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
    ros::ServiceServer serviceSaveGraph;
    ros::ServiceClient pauseRosbag;
    std::mutex stateEstimationMutex;
    std::mutex intensityMutex;
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

    double sigmaScaling;
    double cellSize;
    int numberOfPointsMap;


    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, saveGraphStructure, firstCompleteSonarScan;
    std::string saveStringGraph;
    double maxTimeOptimization;
    int numberOfTimesFirstScan;

    void scanCallback(const commonbluerovmsg::SonarEcho2::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        intensityMeasurement intensityTMP;
        intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR, M_PI * 2);
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
            firstSonarInput = false;
            return;
        }
        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement

        waitForEKFMessagesToArrive(msg->header.stamp.toSec());
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
            //angleDiff = angleBetweenLastKeyframeAndNow(false);
            indexOfLastKeyframe = slamToolsRos::getLastIntensityKeyframe(this->graphSaved);
            int indexStart1, indexEnd1, indexStart2, indexEnd2;
            slamToolsRos::calculateStartAndEndIndexForVoxelCreation(
                    this->graphSaved.getVertexList()->back().getKey() - 5, indexStart1, indexEnd1, this->graphSaved);
            indexStart2 = indexEnd1;
            slamToolsRos::calculateEndIndexForVoxelCreationByStartIndex(indexStart2, indexEnd2, this->graphSaved);


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
            this->currentEstimatedTransformation = slamToolsRos::registrationOfTwoVoxels(voxelData1, voxelData2,
                                                                                         this->initialGuessTransformation,
                                                                                         covarianceEstimation, true,
                                                                                         true,
                                                                                (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                                (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                         false, scanRegistrationObject,
                                                                                         DEBUG_REGISTRATION,
                                                                                         THRESHOLD_FOR_TRANSLATION_MATCHING);

            slamToolsRos::saveResultingRegistrationTMPCOPY(indexStart1, indexEnd1, indexStart2, indexEnd2,
                                                           this->graphSaved, NUMBER_OF_POINTS_DIMENSION,
                                                           IGNORE_DISTANCE_TO_ROBOT,
                                                           DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                           DEBUG_REGISTRATION, this->currentEstimatedTransformation,
                                                           initialGuessTransformation);
//            slamToolsRos::saveResultingRegistration(indexStart1, indexStart2,
//                                                    this->graphSaved, NUMBER_OF_POINTS_DIMENSION,
//                                                    IGNORE_DISTANCE_TO_ROBOT, DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
//                                                    DEBUG_REGISTRATION, this->currentTransformation);


            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)),
                    initialGuessAngle);


//            std::cout << "currentTransformation:" << std::endl;
//            std::cout << this->currentTransformation << std::endl;
//            std::cout << "initial Guess Transformation:" << std::endl;
//            std::cout << this->initialGuessTransformation << std::endl;
//
            std::cout << "Initial guess angle: "
                      << initialGuessAngle * 180 / M_PI
                      << std::endl;
            std::cout << "Registration angle: "
                      << std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)) * 180 / M_PI
                      << std::endl;
            std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter * 180 / M_PI
                      << std::endl;
//
//            std::cout << "Input In Graph:" << std::endl;
//            std::cout << this->currentTransformation << std::endl;
//            std::cout << "Initial Guess From Graph:" << std::endl;
//            std::cout << this->graphSaved.getVertexList()->at(indexStart2).getTransformation().inverse() *
//                         this->graphSaved.getVertexList()->at(indexStart1).getTransformation()
//                      << std::endl;
//            std::cout << covarianceEstimation << std::endl;

            //only if angle diff is smaller than 40 degreece its ok
            if (abs(differenceAngleBeforeAfter) < 40.0 / 180.0 * M_PI) {
                //inverse the transformation because we want the robot transformation, not the scan transformation
                Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation;
                Eigen::Quaterniond qTMP(transformationEstimationRobot1_2.block<3, 3>(0, 0));
//                Eigen::Matrix3d covarianceMatrix = Eigen::Matrix4d::Zero();
                graphSaved.addEdge(indexStart2,
                                   indexStart1,
                                   transformationEstimationRobot1_2.block<3, 1>(0, 3), qTMP,
                                   covarianceEstimation,
                                   LOOP_CLOSURE);//@TODO still not sure about size

            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }
            std::cout << "loopClosure: " << std::endl;

            ////////////// look for loop closure  //////////////

            nav_msgs::OccupancyGrid map = this->createImageOfAllScans((indexStart1 - indexEnd1)/4);
            loopClosureSubMapByMap(map, indexStart1, indexEnd1);



//            slamToolsRos::loopDetectionByClosestPath(this->graphSaved, this->scanRegistrationObject,
//                                                     NUMBER_OF_POINTS_DIMENSION, IGNORE_DISTANCE_TO_ROBOT,
//                                                     DIMENSION_OF_VOXEL_DATA_FOR_MATCHING, DEBUG_REGISTRATION,
//                                                     USE_INITIAL_TRANSLATION_LOOP_CLOSURE,
//                                                     THRESHOLD_FOR_TRANSLATION_MATCHING, MAXIMUM_LOOP_CLOSURE_DISTANCE);
//            this->graphSaved.classicalOptimizeGraph(true);
            this->graphSaved.isam2OptimizeGraph(true, 5);
            slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                    this->publisherMarkerArray, this->sigmaScaling,
                                                    this->publisherPoseSLAM, this->publisherMarkerArrayLoopClosures);
            std::cout << "next: " << std::endl;
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

    bool saveGraph(commonbluerovmsg::saveGraph::Request &req, commonbluerovmsg::saveGraph::Response &res) {
        std::cout << "test for testing" << std::endl;
        this->saveGraphStructure = true;
        this->saveStringGraph = req.saveString;
        res.saved = true;
        return true;
    }

    void waitForEKFMessagesToArrive(double timeUntilWait) {
        while (this->timeVector.empty()) {
            ros::Duration(0.002).sleep();
        }
        while (timeUntilWait > timeVector[timeVector.size() - 1]) {
            ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
            ros::Duration(0.001).sleep();
        }
        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
        ros::Duration(0.001).sleep();
    }


public:

    nav_msgs::OccupancyGrid createImageOfAllScans(int ignoreLastNSteps) {

        std::vector<intensityValues> dataSet;
        double maximumIntensity = slamToolsRos::getDatasetFromGraphForMap(dataSet, this->graphSaved,
                                                                          this->intensityMutex);
        //homePosition is 0 0
        //size of mapData is defined in this->numberOfPointsMap

        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * this->numberOfPointsMap * this->numberOfPointsMap);
        double *mapData;
        mapData = (double *) malloc(sizeof(double) * this->numberOfPointsMap * this->numberOfPointsMap);
        //set zero voxel and index
        for (int i = 0; i < this->numberOfPointsMap * this->numberOfPointsMap; i++) {
            voxelDataIndex[i] = 0;
            mapData[i] = 0;
        }

        for (int currentPosition = 0;
             currentPosition < dataSet.size() - ignoreLastNSteps; currentPosition++) {
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
                            (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * this->numberOfPointsMap /
                                   2) +
                            this->numberOfPointsMap / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * this->numberOfPointsMap /
                                   2) +
                            this->numberOfPointsMap / 2;


                    if (indexX < this->numberOfPointsMap && indexY < this->numberOfPointsMap && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + this->numberOfPointsMap * indexY] =
                                voxelDataIndex[indexX + this->numberOfPointsMap * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        mapData[indexX + this->numberOfPointsMap * indexY] =
                                mapData[indexX + this->numberOfPointsMap * indexY] +
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

        for (int i = 0; i < this->numberOfPointsMap * this->numberOfPointsMap; i++) {
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


        for (int i = 0; i < this->numberOfPointsMap * this->numberOfPointsMap; i++) {

            mapData[i] = (mapData[i]-minimumOfVoxelData)/(maximumOfVoxelData-minimumOfVoxelData)*100;
        }
        nav_msgs::OccupancyGrid occupanyMap;
        occupanyMap.header.frame_id = "map_ned";
        occupanyMap.info.height = this->numberOfPointsMap;
        occupanyMap.info.width = this->numberOfPointsMap;
        occupanyMap.info.resolution = DIMENSION_OF_MAP / this->numberOfPointsMap;
        occupanyMap.info.origin.position.x = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.y = -DIMENSION_OF_MAP / 2;
        occupanyMap.info.origin.position.z = +0.5;
        for (int i = 0; i < this->numberOfPointsMap; i++) {
            for (int j = 0; j < this->numberOfPointsMap; j++) {
                //determine color:
                occupanyMap.data.push_back((int) (mapData[j + this->numberOfPointsMap * i] ));
            }
        }
        this->publisherOccupancyMap.publish(occupanyMap);
        free(voxelDataIndex);
        free(mapData);
        return occupanyMap;
    }

    void loopClosureSubMapByMap(nav_msgs::OccupancyGrid occupanyMap, int indexStart1, int indexEnd1) {
        // get submap, dependent on current location +N/2 -N/2

        double *voxelData1;
        double *voxelData2;
        voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
        voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);

        double maximumVoxel1 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData1, indexStart1, indexEnd1,
                                                                             NUMBER_OF_POINTS_DIMENSION,
                                                                             this->graphSaved,
                                                                             IGNORE_DISTANCE_TO_ROBOT,
                                                                             DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                             Eigen::Matrix4d::Identity());
        Eigen::Matrix4d currentPositionRobot = this->graphSaved.getVertexList()->back().getTransformation();
        std::cout << currentPositionRobot << std::endl;
        std::cout << currentPositionRobot(0, 3) << std::endl;
        std::cout << currentPositionRobot(1, 3) << std::endl;
//        currentPositionRobot(1,4);//x
//        currentPositionRobot(2,4);//y (j +NUMBER_OF_POINTS_DIMENSION/2 % NUMBER_OF_POINTS_DIMENSION)
        for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
            for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
                int xPosInMap =
                        i -NUMBER_OF_POINTS_DIMENSION/2  + ceil(this->numberOfPointsMap/2) + ceil(currentPositionRobot(0, 3) / this->cellSize);
                int yPosInMap =
                        j -NUMBER_OF_POINTS_DIMENSION/2 + ceil(this->numberOfPointsMap/2) + ceil(currentPositionRobot(1, 3) / this->cellSize);

                voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i] = occupanyMap.data[xPosInMap + this->numberOfPointsMap *
                                                                                              yPosInMap]; //changed x and y pos
            }
        }
        Eigen::Matrix3d covarianceEstimation;
        this->initialGuessTransformation = Eigen::Matrix4d::Identity();
        this->initialGuessTransformation.block<3, 3>(0, 0) = currentPositionRobot.block<3, 3>(0, 0);

        // calculate registration with initial Estimate of 0
        this->currentEstimatedTransformation = slamToolsRos::registrationOfTwoVoxels(voxelData1, voxelData2,
                                                                                     this->initialGuessTransformation,
                                                                                     covarianceEstimation, true,
                                                                                     true,
                                                                            (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                            (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                     false, scanRegistrationObject,
                                                                                     DEBUG_REGISTRATION,
                                                                                     THRESHOLD_FOR_TRANSLATION_MATCHING);

        std::cout << "currentTransformation:" << std::endl;
        std::cout << this->currentEstimatedTransformation << std::endl;
        std::cout << "initial Guess Transformation:" << std::endl;
        std::cout << this->initialGuessTransformation << std::endl;
        std::cout << "currentPositionRobot Transformation:" << std::endl;
        std::cout << currentPositionRobot << std::endl;

        std::cout << currentPositionRobot*this->currentEstimatedTransformation << std::endl;
        std::cout << this->currentEstimatedTransformation * currentPositionRobot << std::endl; // richtig glaybe uich
        std::cout << currentPositionRobot*this->currentEstimatedTransformation.inverse() << std::endl;
        std::cout << this->currentEstimatedTransformation.inverse() * currentPositionRobot << std::endl;
        std::cout << "end" << std::endl;


        Eigen::Matrix4d tmpTransformation = this->currentEstimatedTransformation * currentPositionRobot;
        Eigen::Quaterniond qTMP(tmpTransformation.block<3, 3>(0, 0));
        graphSaved.addEdge(0,
                           indexStart1,
                           tmpTransformation.block<3, 1>(0, 3), qTMP,
                           covarianceEstimation,
                           LOOP_CLOSURE);



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
//        ros::spinOnce();

        //rosClassForTests.updateHilbertMap();
//        rosClassForTests.updateMap();
//        rosClassForTests.createImageOfAllScans();

        loop_rate.sleep();

        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}
