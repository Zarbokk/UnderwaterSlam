//
// Created by jurobotics on 13.09.21.
//


#include "geometry_msgs/PoseStamped.h"
//#include "sensor_msgs/Imu.h"
//#include "mavros_msgs/Altitude.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "ping360_sonar/SonarEcho.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include "nav_msgs/OccupancyGrid.h"
#include "scanRegistrationClass.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/SetBoolRequest.h"
#include <std_srvs/SetBool.h>


#define NUMBER_OF_POINTS_DIMENSION 256
#define DIMENSION_OF_VOXEL_DATA_FOR_MATCHING 50
#define NUMBER_OF_POINTS_MAP 512
// 80 simulation 120 valentin
#define DIMENSION_OF_MAP 45.0
#define FACTOR_OF_THRESHOLD 0.9
#define IGNORE_DISTANCE_TO_ROBOT 1.5
#define DEBUG_REGISTRATION true


#define ROTATION_SONAR M_PI
#define NUMBER_OF_LOOP_CLOSURES 2.0// was 2.0
#define DISTANCE_LOOP_CLOUSURE_ALLOWED 3.0// was 3.0
//1: our 256 2:GICP 3:super 4: NDT d2d 5: NDT P2D 6: our global 256
#define WHICH_METHOD_USED 1
#define IGNORE_FIRST_STEPS 0
#define SHOULD_USE_ROSBAG true


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

        subscriberEKF = n_.subscribe("publisherPoseEkf", 10000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(3).sleep();
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



//        graphSaved.addVertex(1, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
//                             Eigen::Vector3d(0, 0, 0), 0, ros::Time::now().toSec(),
//                             FIRST_ENTRY);

//        std::deque<double> subgraphs{0.3, 2};
//        graphSaved.initiallizeSubGraphs(subgraphs, 10);
        this->sigmaScaling = 1.0;

//        this->maxTimeOptimization = 1.0;

        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;
        this->saveGraphStructure = false;

//        this->occupancyMap.createRandomMap();
        this->maxTimeOptimization = 1.0;


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


    ros::Subscriber subscriberEKF, subscriberIntensitySonar;
    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
    ros::ServiceServer serviceSaveGraph;
    ros::ServiceClient pauseRosbag;
    std::mutex stateEstimationMutex;
    std::mutex intensityMutex;
    std::mutex graphSlamMutex;
    //GraphSlam things
    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;

    //PCL
    //std::vector<ping360_sonar::SonarEcho> sonarIntensityList;
    //Matrices:
    Eigen::Matrix4d currentEstimatedTransformation;
    Eigen::Matrix4d initialGuessTransformation;


    //EKF savings
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
    std::deque<Eigen::Quaterniond> rotationVector;
    //int numberOfEdgesBetweenScans;
    int indexLastFullScan;
    //double timeCurrentFullScan;
    double fitnessScore;
    double sigmaScaling;

    //double beginningAngleOfRotation;
    //double lastAngle;
    //double startTimeOfCorrection;
    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan, saveGraphStructure;
    std::string saveStringGraph;
    double maxTimeOptimization;
    //hilbertMap occupancyMap;


    void scanCallback(const commonbluerovmsg::SonarEcho2::ConstPtr &msg) {
//        std::cout << "test" << std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        intensityMeasurement intensityTMP;
        intensityTMP.angle = std::fmod(msg->angle / 400.0 * M_PI * 2.0 + ROTATION_SONAR, M_PI * 2);
        intensityTMP.time = msg->header.stamp.toSec();
        intensityTMP.range = msg->range;
        //intensityTMP.size = msg->intensities.size();
        intensityTMP.increment = msg->step_size;
        std::vector<double> intensitiesVector;
        for (int i = 0; i < msg->intensities.size(); i++) {
            intensitiesVector.push_back(msg->intensities[i]);
        }
        intensityTMP.intensities = intensitiesVector;

        if (firstSonarInput) {

            this->graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                                       Eigen::Vector3d(0, 0, 0), 0, intensityTMP, msg->header.stamp.toSec(),
                                       FIRST_ENTRY);
            firstSonarInput = false;
            return;
        }
        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement



        waitForEKFMessagesToArrive();
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
                                   this->graphSaved.getVertexList()->back().getCovariancePosition(),
                                   this->graphSaved.getVertexList()->back().getCovarianceQuaternion(),
                                   intensityTMP,
                                   msg->header.stamp.toSec(),
                                   INTENSITY_SAVED);

        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
                                 this->graphSaved.getVertexList()->back().getKey(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 Eigen::Vector3d(0.06, 0.06, 0),
                                 0.25 * 0.06, INTEGRATED_POSE);


        int indexOfLastKeyframe;
        double angleDiff = slamToolsRos::angleBetweenLastKeyframeAndNow(this->graphSaved);// i think this is always true

        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (abs(angleDiff) > 2 * M_PI) {


            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
            if (firstCompleteSonarScan) {
                firstCompleteSonarScan = false;
                return;
            }

            if (SHOULD_USE_ROSBAG) {
                std_srvs::SetBool srv;
                srv.request.data = true;
                pauseRosbag.call(srv);
            }
            //angleDiff = angleBetweenLastKeyframeAndNow(false);
            indexOfLastKeyframe = slamToolsRos::getLastIntensityKeyframe(this->graphSaved);




            //we inverse the initial guess, because the registration creates a T from scan 1 to scan 2.
            // But the graph creates a transformation from 1 -> 2 by the robot, therefore inverse.
            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
                     this->graphSaved.getVertexList()->back().getTransformation()).inverse();
//            Eigen::Matrix4d initialGuessTransformation2 =
//                    this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
//                    this->graphSaved.getVertexList()->back().getTransformation();
//            std::cout << this->initialGuessTransformation<< std::endl;
//            std::cout << initialGuessTransformation2<< std::endl;
            std::cout << "this->initialGuessTransformation.inverse()" << std::endl;

            std::cout << this->initialGuessTransformation.inverse() << std::endl;
            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));
            double fitnessScoreX, fitnessScoreY;

            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);

            double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                    indexOfLastKeyframe,
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel


            double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                                    this->graphSaved.getVertexList()->size() -
                                                                    1,
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

            // transform from 1 to 2
//            this->currentTransformation = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(
//                    voxelData1, voxelData2,
//                    this->initialGuessTransformation, true,
//                    true, (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                          (double) NUMBER_OF_POINTS_DIMENSION, true, DEBUG_REGISTRATION);
            this->currentEstimatedTransformation = this->registrationOfTwoVoxels(voxelData1, voxelData2,
                                                                                 this->initialGuessTransformation, true,
                                                                                 true,
                                                                        (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                        (double) NUMBER_OF_POINTS_DIMENSION, false,
                                                                                 DEBUG_REGISTRATION);
            saveResultingRegistration(indexOfLastKeyframe, this->graphSaved.getVertexList()->size() - 1);


            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)),
                    initialGuessAngle);


            std::cout << "FitnessScore X: " << fitnessScoreX << " FitnessScore Y: " << fitnessScoreY << std::endl;

            std::cout << "currentTransformation:" << std::endl;
            std::cout << this->currentEstimatedTransformation << std::endl;

            std::cout << "initial Guess Transformation:" << std::endl;
            std::cout << this->initialGuessTransformation << std::endl;


            std::cout << "Initial guess angle: "
                      << initialGuessAngle * 180 / M_PI
                      << std::endl;
            std::cout << "Registration angle: "
                      << std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)) * 180 / M_PI
                      << std::endl;
            std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter * 180 / M_PI
                      << std::endl;
            //only if angle diff is smaller than 10 degreece its ok
            if (abs(differenceAngleBeforeAfter) < 45.0 / 180.0 * M_PI) {
                //inverse the transformation because we want the robot transformation, not the scan transformation
                Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation.inverse();
                Eigen::Quaterniond qTMP(transformationEstimationRobot1_2.block<3, 3>(0, 0));
                graphSaved.addEdge(slamToolsRos::getLastIntensityKeyframe(this->graphSaved),
                                   this->graphSaved.getVertexList()->size() - 1,
                                   transformationEstimationRobot1_2.block<3, 1>(0, 3), qTMP,
                                   Eigen::Vector3d(0.05 * fitnessScoreX, 0.05 * fitnessScoreY, 0),
                                   0.02,
                                   LOOP_CLOSURE);//@TODO still not sure about size
            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }

            std::cout << "NEW TEST:" << std::endl;
            std::cout << "Input In Graph:" << std::endl;
            std::cout << this->currentEstimatedTransformation.inverse() << std::endl;
            std::cout << "Initial Guess From Graph:" << std::endl;
            std::cout << this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
                         this->graphSaved.getVertexList()->back().getTransformation()
                      << std::endl;
            //

//            this->graphSaved.isam2OptimizeGraph(true);
//            for (int i = 0; i < 1; i++) {
////                this->graphSaved.print();
//                this->graphSaved.classicalOptimizeGraph(true);
//                updateRegistration(this->graphSaved.getEdgeList()->size() - 1);
//            }

            ////////////// look for loop closure  //////////////
            detectLoopClosureNEW(this->graphSaved, this->scanRegistrationObject);
//            this->graphSaved.classicalOptimizeGraph(true);
            this->graphSaved.isam2OptimizeGraph(true);
            std::chrono::steady_clock::time_point begin;
            std::chrono::steady_clock::time_point end;
            begin = std::chrono::steady_clock::now();


            end = std::chrono::steady_clock::now();

            double timeToCalculate = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            std::cout << "timeToCalculate: " << timeToCalculate << std::endl;//time

            std::cout << "next: " << std::endl;

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

    bool saveGraph(commonbluerovmsg::saveGraph::Request &req, commonbluerovmsg::saveGraph::Response &res) {
        std::cout << "test for testing" << std::endl;
        this->saveGraphStructure = true;
        this->saveStringGraph = req.saveString;
        res.saved = true;
        return true;
    }

    bool detectLoopClosure(graphSlamSaveStructure &tmpGraphSaved, scanRegistrationClass tmpregistrationClass) {
        Eigen::Vector3d estimatedPosLastPoint = tmpGraphSaved.getVertexList()->back().getPositionVertex();
        int ignoreStartLoopClosure = 450;
        int ignoreEndLoopClosure = 800;
        if (tmpGraphSaved.getVertexList()->size() < ignoreEndLoopClosure) {
            return false;
        }
        std::vector<int> potentialLoopClosure;
        for (int s = ignoreStartLoopClosure; s < tmpGraphSaved.getVertexList()->size() - ignoreEndLoopClosure; s++) {
            //dist.row(s) = (graphSaved.getVertexList()[s].getPositionVertex() - estimatedPosLastPoint).norm();
            double d = sqrt(
                    pow((estimatedPosLastPoint.x() - tmpGraphSaved.getVertexList()->at(s).getPositionVertex().x()), 2) +
                    pow((estimatedPosLastPoint.y() - tmpGraphSaved.getVertexList()->at(s).getPositionVertex().y()), 2));
            double r1 = (tmpGraphSaved.getVertexList()->at(s).getCovariancePosition().x() +
                         tmpGraphSaved.getVertexList()->at(s).getCovariancePosition().y()) / 2;
            double r2 = (tmpGraphSaved.getVertexList()->back().getCovariancePosition().x() +
                         tmpGraphSaved.getVertexList()->back().getCovariancePosition().y()) / 2;
            if ((d <= r1 - r2 || d <= r2 - r1 || d < r1 + r2 || d == r1 + r2) && d < DISTANCE_LOOP_CLOUSURE_ALLOWED &&
                tmpGraphSaved.getVertexList()->at(s).getTypeOfVertex() == INTENSITY_SAVED_AND_KEYFRAME) {
                potentialLoopClosure.push_back(tmpGraphSaved.getVertexList()->at(s).getKey());
            }
        }
        if (potentialLoopClosure.empty()) {
            return false;
        }


        std::shuffle(potentialLoopClosure.begin(), potentialLoopClosure.end(), std::mt19937(std::random_device()()));

        int loopclosureNumber = 0;
        bool foundLoopClosure = false;
        for (const auto &potentialKey: potentialLoopClosure) {
            double fitnessScore = 1;

            //create voxel
            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            int indexStart, indexEnd;

            if (!slamToolsRos::calculateStartAndEndIndexForVoxelCreation(potentialKey, indexStart, indexEnd,
                                                                         this->graphSaved)) {
                return false;
            }


            double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                    this->graphSaved.getVertexList()->back().getKey(),
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel
            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2,
                                                                                 indexStart, indexEnd,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());





//            double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
//                                                                    potentialKey,
//                                                                    Eigen::Matrix4d::Identity(),
//                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
//                                                                    IGNORE_DISTANCE_TO_ROBOT,
//                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->back().getTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart).getTransformation()).inverse();
            // transform from 1 to 2
//            this->currentTransformation = tmpregistrationClass.registrationOfTwoVoxelsSOFFTFast(voxelData1,
//                                                                                                voxelData2,
//                                                                                                this->initialGuessTransformation,
//                                                                                                true, false,
//                                                                                                (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                                                                                                (double) NUMBER_OF_POINTS_DIMENSION,
//                                                                                                true,
//                                                                                                DEBUG_REGISTRATION);
            this->currentEstimatedTransformation = this->registrationOfTwoVoxels(voxelData1,
                                                                                 voxelData2,
                                                                                 this->initialGuessTransformation,
                                                                                 true, false,
                                                                        (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                        (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                 true,
                                                                                 DEBUG_REGISTRATION);

//            std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
            std::cout << "Estimated Transformation:" << std::endl;
            std::cout << this->currentEstimatedTransformation << std::endl;

            std::cout << "initial Guess Transformation:" << std::endl;
            std::cout << this->initialGuessTransformation << std::endl;

            std::cout << "Initial guess angle: "
                      << std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)) *
                         180 / M_PI
                      << std::endl;
            std::cout << "Registration angle: "
                      <<
                      std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)) *
                      180 / M_PI << std::endl;

//            saveResultingRegistration(this->graphSaved.getVertexList()->back().getKey(), potentialKey);
            saveResultingRegistrationTMPCOPY(this->graphSaved.getVertexList()->back().getKey(), indexStart, indexEnd);

            //inverse the transformation because we want the robot transformation, not the scan transformation
            Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation.inverse();
            Eigen::Vector3d currentPosDiff;
            Eigen::Quaterniond currentRotDiff(transformationEstimationRobot1_2.block<3, 3>(0, 0));
            currentPosDiff.x() = transformationEstimationRobot1_2(0, 3);
            currentPosDiff.y() = transformationEstimationRobot1_2(1, 3);
            currentPosDiff.z() = 0;
            Eigen::Vector3d positionCovariance(fitnessScore, fitnessScore, 0);
            tmpGraphSaved.addEdge(tmpGraphSaved.getVertexList()->back().getKey(), indexStart, currentPosDiff,
                                  currentRotDiff, positionCovariance,
                                  1, LOOP_CLOSURE);
            foundLoopClosure = true;
            loopclosureNumber++;
            if (loopclosureNumber > NUMBER_OF_LOOP_CLOSURES) { break; }// break if multiple loop closures are found

        }
        if (foundLoopClosure) {
            return true;
        }
        return false;
    }

    bool detectLoopClosureNEW(graphSlamSaveStructure &tmpGraphSaved, scanRegistrationClass tmpregistrationClass) {
        Eigen::Vector3d estimatedPosLastPoint = tmpGraphSaved.getVertexList()->back().getPositionVertex();
        int ignoreStartLoopClosure = 450;
        int ignoreEndLoopClosure = 1500;
        if (tmpGraphSaved.getVertexList()->size() < ignoreEndLoopClosure) {
            return false;
        }
        std::vector<int> potentialLoopClosureVector;
        int potentialLoopClosure = ignoreStartLoopClosure;
        for (int s = ignoreStartLoopClosure; s < tmpGraphSaved.getVertexList()->size() - ignoreEndLoopClosure; s++) {
            //dist.row(s) = (graphSaved.getVertexList()[s].getPositionVertex() - estimatedPosLastPoint).norm();
            double d1 = sqrt(
                    pow((estimatedPosLastPoint.x() - tmpGraphSaved.getVertexList()->at(s).getPositionVertex().x()), 2) +
                    pow((estimatedPosLastPoint.y() - tmpGraphSaved.getVertexList()->at(s).getPositionVertex().y()), 2));
            double d2 = sqrt(
                    pow((estimatedPosLastPoint.x() -
                         tmpGraphSaved.getVertexList()->at(potentialLoopClosure).getPositionVertex().x()), 2) +
                    pow((estimatedPosLastPoint.y() -
                         tmpGraphSaved.getVertexList()->at(potentialLoopClosure).getPositionVertex().y()), 2));
            if (d2 > d1) {
                potentialLoopClosure = s;
            }
        }

        potentialLoopClosureVector.push_back(potentialLoopClosure);
//        if (potentialLoopClosureVector.empty()) {
//            return false;
//        }


//        std::shuffle(potentialLoopClosureVector.begin(), potentialLoopClosureVector.end(), std::mt19937(std::random_device()()));

        int loopclosureNumber = 0;
        bool foundLoopClosure = false;
        for (const auto &potentialKey: potentialLoopClosureVector) {
            double fitnessScore = 1;

            //create voxel
            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            int indexStart, indexEnd;

            if (!slamToolsRos::calculateStartAndEndIndexForVoxelCreation(potentialKey, indexStart, indexEnd,
                                                                         this->graphSaved)) {
                return false;
            }


            double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                    this->graphSaved.getVertexList()->back().getKey(),
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel
            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2,
                                                                                 indexStart, indexEnd,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());





//            double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
//                                                                    potentialKey,
//                                                                    Eigen::Matrix4d::Identity(),
//                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
//                                                                    IGNORE_DISTANCE_TO_ROBOT,
//                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

            this->initialGuessTransformation =
                    (this->graphSaved.getVertexList()->back().getTransformation().inverse() *
                     this->graphSaved.getVertexList()->at(indexStart).getTransformation()).inverse();
            // transform from 1 to 2
//            this->currentTransformation = tmpregistrationClass.registrationOfTwoVoxelsSOFFTFast(voxelData1,
//                                                                                                voxelData2,
//                                                                                                this->initialGuessTransformation,
//                                                                                                true, false,
//                                                                                                (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                                                                                                (double) NUMBER_OF_POINTS_DIMENSION,
//                                                                                                false,
//                                                                                                DEBUG_REGISTRATION);
            this->currentEstimatedTransformation = this->registrationOfTwoVoxels(voxelData1,
                                                                                 voxelData2,
                                                                                 this->initialGuessTransformation,
                                                                                 true, false,
                                                                        (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                        (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                 false,
                                                                                 DEBUG_REGISTRATION);

//            std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
            std::cout << "Estimated Transformation:" << std::endl;
            std::cout << this->currentEstimatedTransformation << std::endl;

            std::cout << "initial Guess Transformation:" << std::endl;
            std::cout << this->initialGuessTransformation << std::endl;

            std::cout << "Initial guess angle: "
                      << std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)) *
                         180 / M_PI
                      << std::endl;
            std::cout << "Registration angle: "
                      <<
                      std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)) *
                      180 / M_PI << std::endl;

//            saveResultingRegistration(this->graphSaved.getVertexList()->back().getKey(), potentialKey);
            saveResultingRegistrationTMPCOPY(this->graphSaved.getVertexList()->back().getKey(), indexStart, indexEnd);

            //inverse the transformation because we want the robot transformation, not the scan transformation
            Eigen::Matrix4d transformationEstimationRobot1_2 = this->currentEstimatedTransformation.inverse();
            Eigen::Vector3d currentPosDiff;
            Eigen::Quaterniond currentRotDiff(transformationEstimationRobot1_2.block<3, 3>(0, 0));
            currentPosDiff.x() = transformationEstimationRobot1_2(0, 3);
            currentPosDiff.y() = transformationEstimationRobot1_2(1, 3);
            currentPosDiff.z() = 0;
            Eigen::Vector3d positionCovariance(fitnessScore, fitnessScore, 0);
            tmpGraphSaved.addEdge(tmpGraphSaved.getVertexList()->back().getKey(), indexStart, currentPosDiff,
                                  currentRotDiff, positionCovariance,
                                  1, LOOP_CLOSURE);
            foundLoopClosure = true;
            loopclosureNumber++;
            if (loopclosureNumber > NUMBER_OF_LOOP_CLOSURES) { break; }// break if multiple loop closures are found

        }
        if (foundLoopClosure) {
            return true;
        }
        return false;
    }

    void saveResultingRegistration(int indexFirstKeyFrame, int indexSecondKeyFrame) {


        if (DEBUG_REGISTRATION) {
            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);


            double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                    indexFirstKeyFrame,
                                                                    this->currentEstimatedTransformation,
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

            double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                                    indexSecondKeyFrame,
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
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
            free(voxelData1);
            free(voxelData2);
        }
    }

    void saveResultingRegistrationTMPCOPY(int indexFirstKeyFrame, int indexStart, int indexEnd) {

        if (DEBUG_REGISTRATION) {
            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);


            double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                    indexFirstKeyFrame,
                                                                    this->currentEstimatedTransformation,
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

            double maximumVoxel2 = slamToolsRos::createVoxelOfGraphStartEndPoint(voxelData2,
                                                                                 indexStart, indexEnd,
                                                                                 NUMBER_OF_POINTS_DIMENSION,
                                                                                 this->graphSaved,
                                                                                 IGNORE_DISTANCE_TO_ROBOT,
                                                                                 DIMENSION_OF_VOXEL_DATA_FOR_MATCHING,
                                                                                 Eigen::Matrix4d::Identity());
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
            free(voxelData1);
            free(voxelData2);
        }
    }

    void waitForEKFMessagesToArrive() {
        while (this->timeVector.empty()) {
            ros::Duration(0.002).sleep();
        }
        while (this->graphSaved.getVertexList()->back().getTimeStamp() > timeVector[timeVector.size() - 1]) {
            ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
            ros::Duration(0.001).sleep();
        }
        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
        ros::Duration(0.001).sleep();
    }

    void updateRegistration(int numberOfEdge) {

        int tmpFromKey = this->graphSaved.getEdgeList()->at(numberOfEdge).getFromKey();
        int tmpToKey = this->graphSaved.getEdgeList()->at(numberOfEdge).getToKey();


        //match these voxels together
        this->initialGuessTransformation =
                this->graphSaved.getVertexList()->at(tmpFromKey).getTransformation().inverse() *
                this->graphSaved.getVertexList()->at(tmpToKey).getTransformation();

        double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                              this->initialGuessTransformation(0, 0));
        double fitnessScoreX, fitnessScoreY;

        double *voxelData1;
        double *voxelData2;
        voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
        voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);

        double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                tmpFromKey,
                                                                Eigen::Matrix4d::Identity(),
                                                                NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                IGNORE_DISTANCE_TO_ROBOT,
                                                                DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel


        double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                                tmpToKey,
                                                                Eigen::Matrix4d::Identity(),
                                                                NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                IGNORE_DISTANCE_TO_ROBOT,
                                                                DIMENSION_OF_VOXEL_DATA_FOR_MATCHING);//get voxel

        // transform from 1 to 2
//        this->currentTransformation = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(
//                voxelData1, voxelData2,
//                this->initialGuessTransformation, true,
//                true, (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
//                      (double) NUMBER_OF_POINTS_DIMENSION, true, DEBUG_REGISTRATION);
        this->currentEstimatedTransformation = this->registrationOfTwoVoxels(voxelData1, voxelData2,
                                                                             this->initialGuessTransformation, true,
                                                                             true,
                                                                    (double) DIMENSION_OF_VOXEL_DATA_FOR_MATCHING /
                                                                    (double) NUMBER_OF_POINTS_DIMENSION, true,
                                                                             DEBUG_REGISTRATION);
        saveResultingRegistration(tmpFromKey, tmpToKey);

        double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)),
                initialGuessAngle);


        std::cout << "FitnessScore X: " << fitnessScoreX << " FitnessScore Y: " << fitnessScoreY << std::endl;

        std::cout << "currentTransformation:" << std::endl;
        std::cout << this->currentEstimatedTransformation << std::endl;

        std::cout << "initial Guess Transformation:" << std::endl;
        std::cout << this->initialGuessTransformation << std::endl;


        std::cout << "Initial guess angle: "
                  << initialGuessAngle * 180 / M_PI
                  << std::endl;
        std::cout << "Registration angle: "
                  << std::atan2(this->currentEstimatedTransformation(1, 0), this->currentEstimatedTransformation(0, 0)) * 180 / M_PI
                  << std::endl;
        std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter * 180 / M_PI
                  << std::endl;
        //only if angle diff is smaller than 10 degreece its ok

//        this->graphSaved.getEdgeList()->at(numberOfEdge).setPositionDifference(this->currentTransformation.block<3, 1>(0, 3));
//        this->graphSaved.getEdgeList()->at(numberOfEdge).setRotationDifference(Eigen::Quaterniond(this->currentTransformation.block<3, 3>(0, 0)));
        this->graphSaved.setPoseDifferenceEdge(numberOfEdge, this->currentEstimatedTransformation);

        free(voxelData1);
        free(voxelData2);

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


            //get position of current intensityRay
//                Eigen::Matrix4d transformationOfIntensityRay =
//                        this->graphSaved.getVertexList()->at(indexStart - i).getTransformation().inverse() *
//                        this->graphSaved.getVertexList()->at(indexStart).getTransformation();


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
                occupanyMap.data.push_back((int) (mapData[j + NUMBER_OF_POINTS_MAP * i] * 2));
            }
        }
        this->publisherOccupancyMap.publish(occupanyMap);
        free(voxelDataIndex);
        free(mapData);
    }

    Eigen::Matrix4d registrationOfTwoVoxels(double voxelData1Input[],
                                            double voxelData2Input[],
                                            Eigen::Matrix4d initialGuess,
                                            bool useInitialAngle,
                                            bool useInitialTranslation,
                                            double cellSize,
                                            bool useGauss,
                                            bool debug, int registrationNoiseImpactFactor = 2,
                                            double ignorePercentageFactor = 0.1) {

        std::vector<transformationPeak> listOfTransformations = this->scanRegistrationObject.registrationOfTwoVoxelsSOFFTAllSoluations(
                voxelData1Input, voxelData2Input, cellSize, useGauss, debug, registrationNoiseImpactFactor,
                ignorePercentageFactor);
        double initialAngle = std::atan2(initialGuess(1, 0),
                                         initialGuess(0, 0));
        std::vector<transformationPeak> potentialTranslationsWithAngle;
        if (listOfTransformations.size() > 1) {
            potentialTranslationsWithAngle.push_back(listOfTransformations[0]);

            if (useInitialAngle) {
                for (int i = 1; i < listOfTransformations.size(); i++) {
                    if (abs(potentialTranslationsWithAngle[0].potentialRotation.angle - initialAngle) >
                        abs(listOfTransformations[i].potentialRotation.angle - initialAngle)) {
                        potentialTranslationsWithAngle[0] = listOfTransformations[i];
                    }
                }
            } else {
                potentialTranslationsWithAngle = potentialTranslationsWithAngle;
            }
        } else {
            potentialTranslationsWithAngle.push_back(listOfTransformations[0]);
        }

        Eigen::Vector2d posVector = initialGuess.block<2, 1>(0, 3);
        double maximumValue = 0;
        int indexMaximumAngle = 0;
        int indexMaximumTranslation = 0;
        if (useInitialTranslation) {
            for (int i = 0; i < potentialTranslationsWithAngle.size(); i++) {
                for (int j = 0; j < potentialTranslationsWithAngle[i].potentialTranslations.size(); j++) {
                    if ((potentialTranslationsWithAngle[i].potentialTranslations[j].translationSI - posVector).norm() <
                        (potentialTranslationsWithAngle[indexMaximumAngle].potentialTranslations[indexMaximumTranslation].translationSI -
                         posVector).norm()) {
                        indexMaximumAngle = i;
                        indexMaximumTranslation = j;
                    }
                }
            }

        } else {
            for (int i = 0; i < potentialTranslationsWithAngle.size(); i++) {
                for (int j = 0; j < potentialTranslationsWithAngle[i].potentialTranslations.size(); j++) {
                    if (potentialTranslationsWithAngle[i].potentialTranslations[j].peakHeight >
                        potentialTranslationsWithAngle[indexMaximumAngle].potentialTranslations[indexMaximumTranslation].peakHeight) {
                        indexMaximumAngle = i;
                        indexMaximumTranslation = j;
                    }
                }
            }
        }

        //create matrix4d
        Eigen::Matrix4d returnTransformation = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                   potentialTranslationsWithAngle[indexMaximumAngle].potentialRotation.angle);

        returnTransformation.block<2, 1>(0,
                                         3) = potentialTranslationsWithAngle[indexMaximumAngle].potentialTranslations[indexMaximumTranslation].translationSI;
        return returnTransformation;
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
        rosClassForTests.createImageOfAllScans();

        loop_rate.sleep();

        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}
