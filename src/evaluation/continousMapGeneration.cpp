//
// Created by tim-external on 27.07.22.
//


#include "mavros_msgs/Altitude.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "commonbluerovmsg/staterobotforevaluation.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include <opencv2/imgcodecs.hpp>
#include "std_srvs/SetBool.h"
#include <filesystem>
#include "scanRegistrationClass.h"
#include "nav_msgs/OccupancyGrid.h"


//
//#define NUMBER_OF_POINTS_MAP 256
//
//
//#define NUMBER_OF_SKIPPING_SCANS 1
//
//
//// 50 simulation 30 valentin
//#define NUMBER_OF_CONSECUTIVE_SCANS 1
//// 80 simulation 120 valentin
//#define DIMENSION_OF_MAP 80.0
//// 50 simulation 30 valentin
//#define USING_INITIAL_GUESS_CHANGE_VALENTIN false
//
//
//
//
////1: our 256 2:GICP 3:super 4: NDT d2d 5: NDT P2D 6: our global 256
//#define WHICH_METHOD_USED 2
//#define IGNORE_FIRST_STEPS 0
////Simulation : all 0
//#define ROLL_TRANSFORMATION_ANGLE 0
//#define YAW_TRANSFORMATION_ANGLE 0
//
//
//#define NUMBER_OF_POINTS_DIMENSION 256
//#define DIMENSION_OF_VOXEL_DATA 60
//
////StPere 0.6 seems fine
////Valentin 0.2 seems OK
////simulation 0.9
//#define FACTOR_OF_THRESHOLD 0.9
//#define IGNORE_DISTANCE_TO_ROBOT 1.5
//
//#define SHOULD_USE_ROSBAG true
//
//
//
////#define HOME_LOCATION "/home/tim-external/dataFolder/ValentinBunkerData/"
//////#define HOME_LOCATION "/home/tim-external/dataFolder/StPereDataset/"
////
////#define WHICH_FOLDER_SHOULD_BE_SAVED "4_7_Bunker_range_30_5_RandomShifts105/"
////#define WHICH_FOLDER_SHOULD_BE_SAVED "randomShifts105/"
//
//
//
//
//
//struct intensityValues {
//    Eigen::Matrix4d transformation;
//    intensityMeasurement intensity;
//};
//
//struct groundTruthPositionStruct {
//    double x;
//    double y;
//    double z;
//    double roll;
//    double pitch;
//    double yaw;
//};
//
//
////occupancyMap(256, numberOfPoints, 70, hilbertMap::HINGED_FEATURES)
//class rosClassEKF {
//public:
//    rosClassEKF(ros::NodeHandle n_, const std::string &nameOfTheServiceCall) : graphSaved(3, INTENSITY_BASED_GRAPH),
//                                                                               scanRegistrationObject(
//                                                                                       NUMBER_OF_POINTS_DIMENSION,
//                                                                                       NUMBER_OF_POINTS_DIMENSION / 2,
//                                                                                       NUMBER_OF_POINTS_DIMENSION / 2,
//                                                                                       NUMBER_OF_POINTS_DIMENSION / 2 -
//                                                                                       1) {
//
//
//        this->mapOfBunker = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
//        this->voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
//
//        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
//            this->mapOfBunker[i] = 0;
//            this->voxelDataIndex[i] = 0;
//        }
//
//
//        this->voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
//        this->voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
//
//
//        this->numberOfScan = 0;
//
//        subscriberEKF = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
//        ros::Duration(1).sleep();
//        subscriberIntensitySonar = n_.subscribe("sonar/intensity", 1000, &rosClassEKF::scanCallback, this);
//
////        subscriberGroundTruth = n_.subscribe("/magnetic_heading", 1000, &rosClassEKF::groundTruthCallbackStPere, this);
//        subscriberGroundTruth = n_.subscribe("/positionGT", 1000, &rosClassEKF::groundTruthCallbackGazebo, this);
//        publisherOccupancyMap = n_.advertise<nav_msgs::OccupancyGrid>("occupancyHilbertMap", 10);
//
//
//        if (SHOULD_USE_ROSBAG) {
//            pauseRosbag = n_.serviceClient<std_srvs::SetBoolRequest>(nameOfTheServiceCall);
//        }
//
//        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
//        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
//        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
//        transformationX180Degree(3, 3) = 1;
//
//        graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
//                             Eigen::Vector3d(0, 0, 0), 0, ros::Time::now().toSec(),
//                             FIRST_ENTRY);
//
//
//        this->sigmaScaling = 0.2;
//
//
//        this->firstSonarInput = true;
//        this->firstCompleteSonarScan = true;
//        Eigen::AngleAxisd rotation_vectorz(165 / 180.0 * M_PI, Eigen::Vector3d(0, 0, 1));
//
//
//        this->completeTransformationForCurrentScan = Eigen::Matrix4d::Identity();
//        this->completeTransformationForCurrentScan.block<3, 3>(0, 0) = rotation_vectorz.toRotationMatrix();
//
//
//    }
//
//
//private:
//    int numberOfScan;
//
//
//    ros::Subscriber subscriberEKF, subscriberIntensitySonar, subscriberGroundTruth;
//    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
//    ros::ServiceServer serviceSaveGraph;
//    ros::ServiceClient pauseRosbag;
//    std::mutex stateEstimationMutex;
//    std::mutex intensityMutex;
//    std::mutex graphSlamMutex;
//    std::mutex GTMutex;
//    //GraphSlam things
//    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;
//
//    //PCL
//    //std::vector<ping360_sonar::SonarEcho> sonarIntensityList;
//    //Matrices:
//    Eigen::Matrix4d currentTransformation;
//    Eigen::Matrix4d initialGuessTransformation;
//    Eigen::Matrix4d transformationX180Degree;
//    Eigen::Matrix4d completeTransformationForCurrentScan;
//
//    scanRegistrationClass scanRegistrationObject;
//    groundTruthPositionStruct currentGTPosition;
//    groundTruthPositionStruct lastGTPosition;
//    //EKF savings
//    std::deque<edge> posDiffOverTimeEdges;
//    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
//    std::deque<Eigen::Quaterniond> rotationVector;
//
//    double *voxelData1;
//    double *voxelData2;
//    double *mapOfBunker;
//    int *voxelDataIndex;
//
//    int indexLastFullScan;
//
//    double fitnessScore;
//    double sigmaScaling;
//
//    std::ofstream comparisonFile;
//
//
//    graphSlamSaveStructure graphSaved;
//    bool firstSonarInput, firstCompleteSonarScan;
//
//
//    void scanCallback(const commonbluerovmsg::SonarEcho2::ConstPtr &msg) {
//
//        std::lock_guard<std::mutex> lock2(this->GTMutex);
//        std::lock_guard<std::mutex> lock1(this->graphSlamMutex);
//        if (firstSonarInput) {
//
//            intensityMeasurement intensityTMP;
//            intensityTMP.angle = msg->angle / 400.0 * M_PI * 2.0;
//            intensityTMP.time = msg->header.stamp.toSec();
//            intensityTMP.increment = msg->step_size;
//            intensityTMP.range = msg->range;
//            std::vector<double> intensitiesVector;
//            for (int i = 0; i < msg->intensities.size(); i++) {
//                intensitiesVector.push_back(msg->intensities[i]);
//            }
//            intensityTMP.intensities = intensitiesVector;
//            this->graphSaved.getVertexList()->at(0).setTimeStamp(msg->header.stamp.toSec());
//            this->graphSaved.getVertexList()->at(0).setIntensities(intensityTMP);
//            firstSonarInput = false;
//            return;
//        }
//        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement
//        intensityMeasurement intensityTMP;
//        intensityTMP.angle = msg->angle / 400.0 * M_PI * 2.0;
//        intensityTMP.time = msg->header.stamp.toSec();
//        intensityTMP.range = msg->range;
//        //intensityTMP.size = msg->intensities.size();
//        intensityTMP.increment = msg->step_size;
//        std::vector<double> intensitiesVector;
//        for (int i = 0; i < msg->intensities.size(); i++) {
//            intensitiesVector.push_back(msg->intensities[i]);
//        }
//        intensityTMP.intensities = intensitiesVector;
//
//
//        edge differenceOfEdge = this->calculatePoseDiffByTimeDepOnEKF(
//                this->graphSaved.getVertexList()->back().getTimeStamp(), msg->header.stamp.toSec());
//
//
//        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
//        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
//        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
//        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
//        Eigen::Quaterniond rot(rotM);
//
//
//        this->graphSaved.addVertex(this->graphSaved.getVertexList()->back().getKey() + 1, pos, rot,
//                                   this->graphSaved.getVertexList()->back().getCovariancePosition(),
//                                   this->graphSaved.getVertexList()->back().getCovarianceQuaternion(),
//                                   intensityTMP,
//                                   msg->header.stamp.toSec(),
//                                   INTENSITY_SAVED);
//
//        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getKey() - 1,
//                                 this->graphSaved.getVertexList()->back().getKey(),
//                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
//                                 Eigen::Vector3d(0.06, 0.06, 0),
//                                 0.25 * 0.06, INTEGRATED_POSE,
//                                 10);
//
//
////        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
////        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
////        std::cout << "Time difference 4 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
//        //test if a scan matching should be done
//
//        int indexOfLastKeyframe;
//        double angleDiff = angleBetweenLastKeyframeAndNow();// i think this is always true
//        //std::cout << angleDiff << std::endl;
////        std::cout << msg->header.stamp.toSec() << std::endl;
////        std::cout << ros::Time::now().toSec() << std::endl;
//
//        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
//        if (abs(angleDiff) > 2 * M_PI) {
//
//
//            groundTruthPositionStruct currentGTlocalPosition = this->currentGTPosition;
//            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
//
//            if (firstCompleteSonarScan) {
//                this->lastGTPosition = currentGTlocalPosition;
//                firstCompleteSonarScan = false;
//                return;
//            }
//
//            if (SHOULD_USE_ROSBAG) {
//
//                std_srvs::SetBool srv;
//                srv.request.data = true;
//                pauseRosbag.call(srv);
//            }
//
//            //angleDiff = angleBetweenLastKeyframeAndNow(false);
//            indexOfLastKeyframe = getLastIntensityKeyframe();
//            // also get ground trouth transformation
//
//            //match these voxels together
//            this->initialGuessTransformation =
//                    this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
//                    this->graphSaved.getVertexList()->back().getTransformation();
////            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
////                                                  this->initialGuessTransformation(0, 0));
//
//
////            pcl::PointCloud<pcl::PointXYZ> scan1Threshold;
////            pcl::PointCloud<pcl::PointXYZ> scan2Threshold;
////            pcl::PointCloud<pcl::PointXYZ> finalThreshold;
////            pcl::PointCloud<pcl::PointXYZ> scan1OneValue;
////            pcl::PointCloud<pcl::PointXYZ> scan2OneValue;
////            pcl::PointCloud<pcl::PointXYZ> finalOneValue;
////
////            Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(M_PI, 0, 0);
////            scan1OneValue = generalHelpfulTools::createPCLFromGraphOneValue(indexOfLastKeyframe, tmpMatrix,
////                                                                            this->graphSaved,
////                                                                            IGNORE_DISTANCE_TO_ROBOT,
////                                                                            FACTOR_OF_THRESHOLD);
////            scan2OneValue = generalHelpfulTools::createPCLFromGraphOneValue(
////                    this->graphSaved.getVertexList()->size() - 1, tmpMatrix, this->graphSaved, IGNORE_DISTANCE_TO_ROBOT,
////                    FACTOR_OF_THRESHOLD);
////            scan1Threshold = generalHelpfulTools::createPCLFromGraphOnlyThreshold(indexOfLastKeyframe, tmpMatrix,
////                                                                                  this->graphSaved,
////                                                                                  IGNORE_DISTANCE_TO_ROBOT,
////                                                                                  FACTOR_OF_THRESHOLD);
////            scan2Threshold = generalHelpfulTools::createPCLFromGraphOnlyThreshold(
////                    this->graphSaved.getVertexList()->size() - 1, tmpMatrix, this->graphSaved, IGNORE_DISTANCE_TO_ROBOT,
////                    FACTOR_OF_THRESHOLD);
//
//
//            this->initialGuessTransformation.block<3, 1>(0, 3) =
//                    this->initialGuessTransformation.block<3, 1>(0, 3) + Eigen::Vector3d(0, -0, 0);
//
//            Eigen::Matrix4d GTTransformation =
//                    gtToTranformation(this->lastGTPosition).inverse() * gtToTranformation(currentGTlocalPosition);
//
//
//
//
//            //still missing
//
////            double maximumVoxel1 = createVoxelOfGraph(voxelData1,
////                                                      indexOfLastKeyframe,
////                                                      generalHelpfulTools::getTransformationMatrixFromRPY(0,0,M_PI/2.0),
////                                                      NUMBER_OF_POINTS_DIMENSION);//get
////                                                      // voxel
////            double maximumVoxel2 = createVoxelOfGraph(voxelData2,
////                                                      this->graphSaved.getVertexList()->size() - 1,
////                                                      generalHelpfulTools::getTransformationMatrixFromRPY(0,0,M_PI/2.0),
////                                                      NUMBER_OF_POINTS_DIMENSION);//get voxel
//            bool debug = false;
//
//
//            this->currentTransformation = this->initialGuessTransformation;
//            //here the matching method is used
//
//
////            std::cout << "init Guess:" << std::endl;
////            std::cout << this->initialGuessTransformation << std::endl;
////            std::cout << "GTTransformation:" << std::endl;
////            std::cout << GTTransformation << std::endl;
//
//
//            Eigen::Quaterniond qTMP(this->currentTransformation.block<3, 3>(0, 0));
//            this->graphSaved.addEdge(indexOfLastKeyframe,
//                                     graphSaved.getVertexList()->size() - 1,
//                                     this->currentTransformation.block<3, 1>(0, 3), qTMP,
//                                     Eigen::Vector3d(5.0, 5.0, 0),
//                                     0.05,
//                                     LOOP_CLOSURE,
//                                     10.0);//@TODO still not sure about size
//
//
////            std::cout << "test:" <<std::endl;
////            std::cout <<   generalHelpfulTools::getTransformationMatrixFromRPY(0,0,M_PI/2.0)* estimatedTransformation.block<4, 1>(0, 3) << std::endl;
////            std::cout <<   generalHelpfulTools::getTransformationMatrixFromRPY(0,0,-M_PI/2.0)* estimatedTransformation.block<4, 1>(0, 3) << std::endl;
////            std::cout <<   generalHelpfulTools::getTransformationMatrixFromRPY(0,0,M_PI)* estimatedTransformation.block<4, 1>(0, 3) << std::endl;
//
//
//            //take the estimated transformation and save it in a file.
//
//
//            //save the Picture of the voxel grid
//
//
//            if (debug) {
//                double maximumVoxel1tmp = generalHelpfulTools::createVoxelOfGraph(voxelData1,
//                                                                                  indexOfLastKeyframe,
//                                                                                  generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                                                          0, 0,
//                                                                                          M_PI /
//                                                                                          2.0),
//                                                                                  NUMBER_OF_POINTS_DIMENSION,
//                                                                                  this->graphSaved,
//                                                                                  IGNORE_DISTANCE_TO_ROBOT,
//                                                                                  DIMENSION_OF_VOXEL_DATA);//get
//                double maximumVoxel2tmp = generalHelpfulTools::createVoxelOfGraph(voxelData2,
//                                                                                  this->graphSaved.getVertexList()->size() -
//                                                                                  1,
//                                                                                  this->currentTransformation *
//                                                                                  generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                                                          0, 0,
//                                                                                          M_PI /
//                                                                                          2.0),
//                                                                                  NUMBER_OF_POINTS_DIMENSION,
//                                                                                  this->graphSaved,
//                                                                                  IGNORE_DISTANCE_TO_ROBOT,
//                                                                                  DIMENSION_OF_VOXEL_DATA);//get voxel
//
//                std::ofstream myFile1, myFile2;
//                myFile1.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel1.csv");
//                myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel2.csv");
//                for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
//                    for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
//                        myFile1 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // real part
//                        myFile1 << "\n";
//                        myFile2 << voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
//                        myFile2 << "\n";
//                    }
//                }
//                myFile1.close();
//                myFile2.close();
//            }
//
//
//            std::cout << "########################################################## " << this->numberOfScan
//                      << " NEW SCAN ##########################################################" << std::endl;
//            this->lastGTPosition = currentGTlocalPosition;
//            this->numberOfScan++;
//            if (numberOfScan > NUMBER_OF_CONSECUTIVE_SCANS) {
//                this->createImageOfAllScans();
//
//                std::ofstream outMatrix(
//                        "/home/tim-external/dataFolder/SimulationEnvironment/experimentForVideoICRA/consecutiveAll/consecutiveScanCreation_" +
//                        std::to_string(WHICH_METHOD_USED) + "_" + std::to_string(numberOfScan) + ".csv");
//
//                for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
//                    for (int k = 0; k < NUMBER_OF_POINTS_MAP; k++)
//                        outMatrix << this->mapOfBunker[j + NUMBER_OF_POINTS_MAP * k] << ',';
//                    outMatrix << '\n';
//                }
//                outMatrix.close();
////                exit(-1);
//            }
//
//            if (SHOULD_USE_ROSBAG) {
//
//                std_srvs::SetBool srv;
//                srv.request.data = false;
//                pauseRosbag.call(srv);
//            }
//
//
//        }
//
//
//    }
//
//    void stateEstimationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
//        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
//        double currentTimeStamp = msg->header.stamp.toSec();
//        // calculate where to put the current new message
//        int i = 0;
//        if (!this->timeVector.empty()) {
//            i = this->timeVector.size();
//            while (this->timeVector[i - 1] > currentTimeStamp) {
//                i--;
//            }
//        }
//
//        if (i == this->timeVector.size() || i == 0) {
//            Eigen::Quaterniond tmpQuad(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
//                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
//            this->rotationVector.push_back(tmpQuad);
//            this->timeVector.push_back(msg->header.stamp.toSec());
//            this->xPositionVector.push_back(msg->pose.pose.position.x);
//            this->yPositionVector.push_back(msg->pose.pose.position.y);
//            this->zPositionVector.push_back(msg->pose.pose.position.z);
//        } else {
//            std::cout << "we test it" << std::endl;
//            exit(0);
//        }
//    }
//
//    void clearSavingsOfPoses(double upToTime) {
//        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
//        while (this->timeVector[0] < upToTime - 2) {//two second puffer
//
//            this->rotationVector.pop_front();
//            this->timeVector.pop_front();
//            this->xPositionVector.pop_front();
//            this->yPositionVector.pop_front();
//            this->zPositionVector.pop_front();
//        }
//    }
//
//    edge calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd) {
//        //this is done to make sure 1 more message is coming from the EKF directly
//        //ros::Duration(0.001).sleep();
//        while (endTimeToAdd > this->timeVector[this->timeVector.size() - 1]) {
//            ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
//            ros::Duration(0.001).sleep();
//        }
//
//        //@TEST
//        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
//        //find index of start and end
//        int indexOfStart = 0;
//        while (this->timeVector[indexOfStart] < startTimetoAdd && this->timeVector.size() > indexOfStart) {
//            indexOfStart++;
//        }
//        if (indexOfStart > 0) {
//            indexOfStart--;
//        }
//
//        int indexOfEnd = 0;
//        while (this->timeVector[indexOfEnd] < endTimeToAdd && this->timeVector.size() > indexOfEnd) {
//            indexOfEnd++;
//        }
//        indexOfEnd--;
//
//        Eigen::Matrix4d transformationTMP = Eigen::Matrix4d::Identity();
//
//        if (indexOfStart > 0) {
//            double interpolationFactor = 1.0 - ((this->timeVector[indexOfStart + 1] - startTimetoAdd) /
//                                                (this->timeVector[indexOfStart + 1] - this->timeVector[indexOfStart]));
//
//            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
//            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[indexOfStart - 1].toRotationMatrix();
//            transformationOfEKFStart(0, 3) = this->xPositionVector[indexOfStart];
//            transformationOfEKFStart(1, 3) = this->yPositionVector[indexOfStart];
//            transformationOfEKFStart(2, 3) = this->zPositionVector[indexOfStart];
//
//            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
//            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[indexOfStart].toRotationMatrix();
//            transformationOfEKFEnd(0, 3) = this->xPositionVector[indexOfStart + 1];
//            transformationOfEKFEnd(1, 3) = this->yPositionVector[indexOfStart + 1];
//            transformationOfEKFEnd(2, 3) = this->zPositionVector[indexOfStart + 1];
//
//            transformationTMP = transformationTMP *
//                                generalHelpfulTools::interpolationTwo4DTransformations(transformationOfEKFStart,
//                                                                                       transformationOfEKFEnd,
//                                                                                       interpolationFactor).inverse() *
//                                transformationOfEKFEnd;
//        }
//
//
//        int i = indexOfStart + 1;
//        while (i < indexOfEnd) {
//            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
//            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[i].toRotationMatrix();
//            transformationOfEKFEnd(0, 3) = this->xPositionVector[i];
//            transformationOfEKFEnd(1, 3) = this->yPositionVector[i];
//            transformationOfEKFEnd(2, 3) = this->zPositionVector[i];
//
//            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
//            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[i - 1].toRotationMatrix();
//            transformationOfEKFStart(0, 3) = this->xPositionVector[i - 1];
//            transformationOfEKFStart(1, 3) = this->yPositionVector[i - 1];
//            transformationOfEKFStart(2, 3) = this->zPositionVector[i - 1];
//
//            transformationTMP = transformationTMP * (transformationOfEKFStart.inverse() * transformationOfEKFEnd);
//            i++;
//        }
//
//        if (indexOfEnd > 0) {
//            double interpolationFactor = ((endTimeToAdd - this->timeVector[indexOfEnd]) /
//                                          (this->timeVector[indexOfEnd + 1] - this->timeVector[indexOfEnd]));
//
//            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
//            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[indexOfEnd].toRotationMatrix();
//            transformationOfEKFStart(0, 3) = this->xPositionVector[indexOfEnd];
//            transformationOfEKFStart(1, 3) = this->yPositionVector[indexOfEnd];
//            transformationOfEKFStart(2, 3) = this->zPositionVector[indexOfEnd];
//
//            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
//            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[indexOfEnd + 1].toRotationMatrix();
//            transformationOfEKFEnd(0, 3) = this->xPositionVector[indexOfEnd + 1];
//            transformationOfEKFEnd(1, 3) = this->yPositionVector[indexOfEnd + 1];
//            transformationOfEKFEnd(2, 3) = this->zPositionVector[indexOfEnd + 1];
//
//            transformationTMP = transformationTMP * transformationOfEKFStart.inverse() *
//                                generalHelpfulTools::interpolationTwo4DTransformations(transformationOfEKFStart,
//                                                                                       transformationOfEKFEnd,
//                                                                                       interpolationFactor);
//        }
//        //std::cout << diffMatrix << std::endl;
//        Eigen::Vector3d tmpPosition = transformationTMP.block<3, 1>(0, 3);
//        //set z pos diff to zero
//        tmpPosition[2] = 0;
//        Eigen::Quaterniond tmpRot(transformationTMP.block<3, 3>(0, 0));
//        Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(tmpRot);
//        //set rp on zero only yaw interesting
//        tmpRot = generalHelpfulTools::getQuaternionFromRPY(0, 0, rpyTMP[2]);
//        edge tmpEdge(0, 0, tmpPosition, tmpRot, Eigen::Vector3d(0, 0, 0), 0, 3,
//                     INTEGRATED_POSE);
//
//        return tmpEdge;
//    }
//
//
//    double getDatasetFromGraphForMap(std::vector<intensityValues> &dataSet) {
//        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
////        std::vector<dataPointStruct> dataSet;
//
////        std::random_device rd;  // Will be used to obtain a seed for the random number engine
////        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
////        std::uniform_real_distribution<> dis(0.0, 1.0);
//        double maxOverall = 0;
//        for (int i = 0; i < this->graphSaved.getVertexList()->size(); i++) {
//            intensityValues tmpInt;
//            tmpInt.transformation = this->graphSaved.getVertexList()->at(i).getTransformation();
//            tmpInt.intensity = this->graphSaved.getVertexList()->at(i).getIntensities();
//
//
//            double it = *max_element(std::begin(tmpInt.intensity.intensities),
//                                     std::end(tmpInt.intensity.intensities)); // C++11
//            if (it > maxOverall) {
//                maxOverall = it;
//            }
//            dataSet.push_back(tmpInt);
//        }
//
//
//        return maxOverall;
//    }
//
//
//    int getLastIntensityKeyframe() {//the absolut last entry is ignored
//        int lastKeyframeIndex = this->graphSaved.getVertexList()->size() - 2;//ignore the last index
//        //find last keyframe
//        while (this->graphSaved.getVertexList()->at(lastKeyframeIndex).getTypeOfVertex() !=
//               INTENSITY_SAVED_AND_KEYFRAME &&
//               this->graphSaved.getVertexList()->at(lastKeyframeIndex).getTypeOfVertex() != FIRST_ENTRY) {
//            lastKeyframeIndex--;
//        }
//        return lastKeyframeIndex;
//    }
//
//    double angleBetweenLastKeyframeAndNow() {
//        double resultingAngleSonar = 0;
//        double resultingAngleMovement = 0;
//        int lastKeyframeIndex = getLastIntensityKeyframe();
//
//        for (int i = lastKeyframeIndex; i < this->graphSaved.getVertexList()->size() - 1; i++) {
//            Eigen::Quaterniond currentRot =
//                    this->graphSaved.getVertexList()->at(i).getRotationVertex().inverse() *
//                    this->graphSaved.getVertexList()->at(i + 1).getRotationVertex();
//
//
//            Eigen::Vector3d rpy = generalHelpfulTools::getRollPitchYaw(currentRot);
//            resultingAngleMovement += rpy(2);
//            resultingAngleSonar += generalHelpfulTools::angleDiff(
//                    this->graphSaved.getVertexList()->at(i + 1).getIntensities().angle,
//                    this->graphSaved.getVertexList()->at(i).getIntensities().angle);
//        }
//
//        return resultingAngleMovement + resultingAngleSonar;
//
//
//    }
//
//
//    void groundTruthCallbackGazebo(const commonbluerovmsg::staterobotforevaluation::ConstPtr &msg) {
//        std::lock_guard<std::mutex> lock(this->GTMutex);
//        this->currentGTPosition.x = msg->xPosition;
//        this->currentGTPosition.y = msg->yPosition;
//        this->currentGTPosition.z = msg->zPosition;
//        this->currentGTPosition.roll = msg->roll;
//        this->currentGTPosition.pitch = msg->pitch;
//        this->currentGTPosition.yaw = msg->yaw;
//
//    }
//
//    void groundTruthCallbackStPere(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
//        std::lock_guard<std::mutex> lock(this->GTMutex);
//        this->currentGTPosition.x = msg->vector.x;
//        this->currentGTPosition.y = msg->vector.y;
//        this->currentGTPosition.z = 0;
//        this->currentGTPosition.roll = 0;
//        this->currentGTPosition.pitch = 0;
//        this->currentGTPosition.yaw = msg->vector.z;
//
//    }
//
//    Eigen::Matrix4d gtToTranformation(groundTruthPositionStruct input) {
//        Eigen::Matrix4d output;
//        output = generalHelpfulTools::getTransformationMatrixFromRPY(input.roll, input.pitch, input.yaw);
//
//        output(0, 3) = input.x;
//        output(1, 3) = input.y;
//        output(2, 3) = input.z;
//
//        return output;
//    }
//
//
//    std::vector<std::string> get_directories(const std::string &s) {
//        std::vector<std::string> r;
//        for (auto &p: std::filesystem::directory_iterator(s))
//            if (p.is_directory())
//                r.push_back(p.path().string());
//        return r;
//    }
//
//public:
//    void createImageOfAllScans() {
//        //homePosition is 0 0
//        //size of mapData is defined in NUMBER_OF_POINTS_MAP
//
//        int whichMethod = WHICH_METHOD_USED;
//        int ignoreFirstNSteps = IGNORE_FIRST_STEPS;
//        double correctionFactorMatchingYaw = 0;// M_PI/2.0 at real data
//        double correctionFactorMatchingRoll = 0.001;// M_PI/2.0 at real data
//        double correctionFactorCreationMapYaw = 0;// M_PI/2.0 at real data
//        double correctionFactorCreationRoll = 0;// M_PI at real data
//
//
//
//        //re calculate mapData
//        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
//            if (voxelDataIndex[i] > 0) {
//                this->mapOfBunker[i] = this->mapOfBunker[i] * voxelDataIndex[i];
//            }
//        }
//
//
//        //get array of all created connections of matching estimations
//        std::vector<edge> listOfRegistratedEdges;
//        for (const edge &tmpEdgeFromList: *this->graphSaved.getEdgeList()) {
//            if (tmpEdgeFromList.getTypeOfEdge() == LOOP_CLOSURE) {
//                edge tmpEdge(tmpEdgeFromList);
//                listOfRegistratedEdges.push_back(tmpEdge);
//            }
//        }
//
////        int startPoint = 0;
//        //missing: just add the point cloud to map at startPoint
//
//
//
//
//
//        int indexFirstPCL = listOfRegistratedEdges[listOfRegistratedEdges.size() - 2].getToKey();
//        int indexSecondPCL = listOfRegistratedEdges[listOfRegistratedEdges.size() - 1].getToKey();
//
//
//        this->initialGuessTransformation =
//                this->graphSaved.getVertexList()->at(indexFirstPCL).getTransformation().inverse() *
//                this->graphSaved.getVertexList()->at(indexSecondPCL).getTransformation();
//
//        this->currentTransformation = this->calculateRegistration(this->initialGuessTransformation, whichMethod,
//                                                                  indexFirstPCL, indexSecondPCL,
//                                                                  correctionFactorMatchingYaw,
//                                                                  correctionFactorMatchingRoll);
//        std::cout << "init guess:" << std::endl;
//        std::cout << this->initialGuessTransformation << std::endl;
//        std::cout << "current Transformation:" << std::endl;
//        std::cout << this->currentTransformation << std::endl;
//
//
//        this->completeTransformationForCurrentScan =
//                this->completeTransformationForCurrentScan * this->currentTransformation;
//
//
//        // method to create PCL in Map
//        int indexStart = indexSecondPCL;
//        this->mapCalculation(indexStart, correctionFactorCreationRoll,
//                             correctionFactorCreationMapYaw);
//
//        //TO THE END
//
//        double maximumOfVoxelData = 0;
//        for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
//            if (voxelDataIndex[i] > 0) {
//                this->mapOfBunker[i] = this->mapOfBunker[i] / voxelDataIndex[i];
//                if (maximumOfVoxelData < this->mapOfBunker[i]) {
//                    maximumOfVoxelData = this->mapOfBunker[i];
//                }
//                //std::cout << voxelData[i] << std::endl;
//            }
//        }// @TODO calculate the maximum and normalize "somehow"
//
//
//
//        nav_msgs::OccupancyGrid occupanyMap;
//        occupanyMap.info.height = NUMBER_OF_POINTS_MAP;
//        occupanyMap.info.width = NUMBER_OF_POINTS_MAP;
//        occupanyMap.info.resolution = DIMENSION_OF_MAP / NUMBER_OF_POINTS_MAP;
//        occupanyMap.info.origin.position.x = -DIMENSION_OF_MAP / 2;
//        occupanyMap.info.origin.position.y = -DIMENSION_OF_MAP / 2;
//        occupanyMap.info.origin.position.z = +0.5;
//
//
//        for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {
//            for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
//                //determine color:
//                occupanyMap.data.push_back((int) (this->mapOfBunker[i + NUMBER_OF_POINTS_MAP * j] * 2));
//            }
//        }
//        publisherOccupancyMap.publish(occupanyMap);
//
//
//    }
//
//    Eigen::Matrix4d
//    calculateRegistration(Eigen::Matrix4d &ourInitialGuess, int whichMethod, int indexFirstPCL, int indexSecondPCL,
//                          double correctionFactorMatchingYaw, double correctionFactorPCLMatchingRoll) {
//        if (USING_INITIAL_GUESS_CHANGE_VALENTIN) {
//            Eigen::Matrix4d initialGuessTMP = ourInitialGuess;
//
//            ourInitialGuess(0, 3) = initialGuessTMP(1, 3);
//            ourInitialGuess(1, 3) = -initialGuessTMP(0, 3);
//        }
//
//
//        pcl::PointCloud<pcl::PointXYZ> final;
//        double maximumVoxel1 = generalHelpfulTools::createVoxelOfGraph(voxelData1,
//                                                                       indexFirstPCL,
//                                                                       generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                                               0, 0,
//                                                                               correctionFactorMatchingYaw),
//                                                                       NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
//                                                                       IGNORE_DISTANCE_TO_ROBOT,
//                                                                       DIMENSION_OF_VOXEL_DATA);//get
//
//        std::ofstream myFile1;
//        myFile1.open("/home/tim-external/dataFolder/SimulationEnvironment/experimentForVideoICRA/scanNumber" +
//                     std::to_string(this->numberOfScan) + ".csv");
//
//        for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
//            for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
//                myFile1 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // real part
//                myFile1 << "\n";
//            }
//        }
//        myFile1.close();
//
//        maximumVoxel1 = generalHelpfulTools::createVoxelOfGraph(voxelData1,
//                                                                indexSecondPCL,
//                                                                generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                                        0, 0,
//                                                                        correctionFactorMatchingYaw),
//                                                                NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
//                                                                IGNORE_DISTANCE_TO_ROBOT,
//                                                                DIMENSION_OF_VOXEL_DATA);//get
//
//
//        myFile1.open("/home/tim-external/dataFolder/SimulationEnvironment/experimentForVideoICRA/scanNumber" +
//                     std::to_string(this->numberOfScan + 1) + ".csv");
//
//        for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
//            for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
//                myFile1 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // real part
//                myFile1 << "\n";
//            }
//        }
//        myFile1.close();
//
//
//
//
//
//        // voxel
//        double maximumVoxel2 = generalHelpfulTools::createVoxelOfGraph(voxelData2,
//                                                                       indexSecondPCL,
//                                                                       generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                                               0, 0,
//                                                                               correctionFactorMatchingYaw),
//                                                                       NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
//                                                                       IGNORE_DISTANCE_TO_ROBOT,
//                                                                       DIMENSION_OF_VOXEL_DATA);//get voxel
//
//        Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(correctionFactorPCLMatchingRoll,
//                                                                                        0,
//                                                                                        correctionFactorMatchingYaw);
//        pcl::PointCloud<pcl::PointXYZ> cloudFirstScan;
//        pcl::PointCloud<pcl::PointXYZ> cloudSecondScan;
//
//
//        cloudFirstScan = generalHelpfulTools::createPCLFromGraphOnlyThreshold(indexFirstPCL, tmpMatrix,
//                                                                              this->graphSaved,
//                                                                              IGNORE_DISTANCE_TO_ROBOT,
//                                                                              FACTOR_OF_THRESHOLD);
//
//        cloudSecondScan = generalHelpfulTools::createPCLFromGraphOnlyThreshold(indexSecondPCL, tmpMatrix,
//                                                                               this->graphSaved,
//                                                                               IGNORE_DISTANCE_TO_ROBOT,
//                                                                               FACTOR_OF_THRESHOLD);
////        cloudFirstScan = createPCLFromGraphOneValue(indexFirstPCL, tmpMatrix);
////        cloudSecondScan = createPCLFromGraphOneValue(indexSecondPCL, tmpMatrix);
//
//
//
//
//        Eigen::Matrix4d Tout = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 0.1);
//
////        Tout(1,3) = 0.7;
////        Tout(0,3) = -0.2;
////        std::cout << "Tout" << std::endl;
////        std::cout << Tout << std::endl;
////        pcl::transformPointCloud(cloudFirstScan, cloudSecondScan, Tout);
////         ourInitialGuess = Eigen::Matrix4d::Identity();
//
//
//        pcl::io::savePLYFileBinary("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",
//                                   cloudFirstScan);
//        pcl::io::savePLYFileBinary("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",
//                                   cloudSecondScan);
//
//
//        double fitnessScoreX;
//
//
//        if (whichMethod == 1) {
//            Eigen::Matrix4d ourInitGuessTMP = ourInitialGuess;
////            ourInitGuessTMP(1, 3) = ourInitialGuess(0, 3);
////            ourInitGuessTMP(0, 3) = -ourInitialGuess(1, 3);
////            std::cout << "modified initial guess: " << std::endl;
////            std::cout << ourInitGuessTMP << std::endl;
//
//            //SOFFT OUR OWN 256 local
//            Eigen::Matrix4d tmpResultMatrix = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(this->voxelData1,
//                                                                                                      this->voxelData2,
//                                                                                                      ourInitGuessTMP,
//                                                                                                      true,
//                                                                                                      true,
//                                                                                                      (double) DIMENSION_OF_VOXEL_DATA /
//                                                                                                      (double) NUMBER_OF_POINTS_DIMENSION,
//                                                                                                      true, true);
//
//            this->currentTransformation = tmpResultMatrix;
//        }
//
//        if (whichMethod == 2) {
//            Eigen::Matrix4d ourInitGuessTMP = ourInitialGuess;
//            //GICP
//            double fitnessScoreX;
//            Eigen::Matrix4d tmpResultMatrix = scanRegistrationObject.generalizedIcpRegistration(cloudFirstScan,
//                                                                                                cloudSecondScan,
//                                                                                                final,
//                                                                                                fitnessScoreX,
//                                                                                                ourInitGuessTMP);
//            this->currentTransformation = tmpResultMatrix;
//
//        }
//        if (whichMethod == 3) {
//            //SUPER4PCS
//            Eigen::Matrix4d ourInitGuessTMP = ourInitialGuess;
//
//            Eigen::Matrix4d tmpResultMatrix = scanRegistrationObject.super4PCSRegistration(cloudFirstScan,
//                                                                                           cloudSecondScan,
//                                                                                           ourInitGuessTMP,
//                                                                                           true, false);
//            this->currentTransformation = tmpResultMatrix;
//        }
//        if (whichMethod == 4) {
//            //NDT D2D 2D,
//
//
//            Eigen::Matrix4d ourInitGuessTMP = ourInitialGuess;
////            Eigen::Matrix4d Tout = generalHelpfulTools::getTransformationMatrixFromRPY( 0.0001, 0, 0);
////
////
////            pcl::transformPointCloud(cloudFirstScan, cloudFirstScan, Tout);
////            pcl::transformPointCloud(cloudSecondScan, cloudSecondScan, Tout);
//
//
//
//
//
//
//            //pcl::transformPointCloud(cloudSecondScan, cloudSecondScan, Tout);
////
////            pcl::io::savePLYFileBinary("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",
////                                       cloudFirstScan);
////            pcl::io::savePLYFileBinary("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",
////                                       cloudSecondScan);
//////
////            pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",
////                                 cloudFirstScan);
////            pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",
////                                 cloudSecondScan);
////            std::cout << "ourInitGuessTMP" << std::endl;
////            std::cout << ourInitGuessTMP << std::endl;
//            Eigen::Matrix4d tmpResultMatrix = scanRegistrationObject.ndt_d2d_2d(cloudFirstScan,
//                                                                                cloudSecondScan,
//                                                                                ourInitGuessTMP,
//                                                                                true);
////            std::cout << "tmpResultMatrix" << std::endl;
////            std::cout << tmpResultMatrix << std::endl;
////            Eigen::Matrix4d tmpResultMatrix = scanRegistrationObject.ndt_d2d_2d(
////                    cloudFirstScan,cloudSecondScan ,
////                    Eigen::Matrix4d::Identity(),
////                    true);
//            this->currentTransformation = tmpResultMatrix;
//
//
//        }
//        if (whichMethod == 5) {
//
//
//            Eigen::Matrix4d ourInitGuessTMP = ourInitialGuess;
////            Eigen::Matrix4d Tout = generalHelpfulTools::getTransformationMatrixFromRPY(0.0001, 0, 0);
////
////
////            pcl::transformPointCloud(cloudFirstScan, cloudFirstScan, Tout);
////            pcl::transformPointCloud(cloudSecondScan, cloudSecondScan, Tout);
//
////            std::cout << "ourInitGuessTMP" << std::endl;
////            std::cout << ourInitGuessTMP << std::endl;
//            //NDT P2D
//            Eigen::Matrix4d tmpResultMatrix = scanRegistrationObject.ndt_p2d(cloudFirstScan,
//                                                                             cloudSecondScan,
//                                                                             ourInitGuessTMP,
//                                                                             true);
////            std::cout << "tmpResultMatrix" << std::endl;
////            std::cout << tmpResultMatrix << std::endl;
//
//            this->currentTransformation = tmpResultMatrix;
//
//
////            this->currentTransformation(0, 1) = -this->currentTransformation(0, 1);
////            this->currentTransformation(1, 0) = -this->currentTransformation(1, 0);
////            this->currentTransformation(1, 3) = -this->currentTransformation(1, 3);
//        }
//        if (whichMethod == 6) {
//            //SOFFT OUR OWN 256 global
//            this->currentTransformation = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(voxelData1,
//                                                                                                  voxelData2,
//                                                                                                  this->initialGuessTransformation,
//                                                                                                  true,
//                                                                                                  false,
//                                                                                                  (double) DIMENSION_OF_VOXEL_DATA /
//                                                                                                  (double) NUMBER_OF_POINTS_DIMENSION,
//                                                                                                  false, true);
//        }
//
//        if (true) {
//            double maximumVoxel1tmp = generalHelpfulTools::createVoxelOfGraph(voxelData1,
//                                                                              indexFirstPCL,
//                                                                              this->currentTransformation.inverse() *
//                                                                              generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                                                      0, 0,
//                                                                                      correctionFactorMatchingYaw),
//                                                                              NUMBER_OF_POINTS_DIMENSION,
//                                                                              this->graphSaved,
//                                                                              IGNORE_DISTANCE_TO_ROBOT,
//                                                                              DIMENSION_OF_VOXEL_DATA);//get
////            double maximumVoxel1tmp = createVoxelOfGraph(voxelData1,
////                                                         indexFirstPCL,
////                                                         ourInitialGuess.inverse() * generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
////                                                                                                                                           correctionFactorMatchingYaw),
////                                                         NUMBER_OF_POINTS_DIMENSION);//get
//
//
//            double maximumVoxel2tmp = generalHelpfulTools::createVoxelOfGraph(voxelData2,
//                                                                              indexSecondPCL,
//
//                                                                              generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                                                      0, 0,
//                                                                                      correctionFactorMatchingYaw),
//                                                                              NUMBER_OF_POINTS_DIMENSION,
//                                                                              this->graphSaved,
//                                                                              IGNORE_DISTANCE_TO_ROBOT,
//                                                                              DIMENSION_OF_VOXEL_DATA);//get voxel
//
//            std::ofstream myFile1, myFile2;
//            myFile1.open(
//                    "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel1.csv");
//            myFile2.open(
//                    "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/resultVoxel2.csv");
//            for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
//                for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
//                    myFile1 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // real part
//                    myFile1 << "\n";
//                    myFile2 << voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
//                    myFile2 << "\n";
//                }
//            }
//            myFile1.close();
//            myFile2.close();
//        }
//
//        return this->currentTransformation;
//    }
//
//
//    void mapCalculation(int indexStart, double correctionFactorCreationRoll, double correctionFactorCreationMap) {
//
//        int i = 0;
//        do {
//            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
//
//
//            //get position of current intensityRay
////                Eigen::Matrix4d transformationOfIntensityRay =
////                        this->graphSaved.getVertexList()->at(indexStart - i).getTransformation().inverse() *
////                        this->graphSaved.getVertexList()->at(indexStart).getTransformation();
//
//            Eigen::Matrix4d transformationOfIntensityRay =
//                    this->graphSaved.getVertexList()->at(indexStart).getTransformation().inverse() *
//                    this->graphSaved.getVertexList()->at(indexStart - i).getTransformation();
//            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
//            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
//                                                                                                             this->graphSaved.getVertexList()->at(
//                                                                                                                     indexStart -
//                                                                                                                     i).getIntensities().angle);
//
//            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT / (this->graphSaved.getVertexList()->at(
//                    indexStart - i).getIntensities().range / ((double) this->graphSaved.getVertexList()->at(
//                    indexStart - i).getIntensities().intensities.size())));
//
//
//            for (int j = ignoreDistance;
//                 j <
//                 this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
//                double distanceOfIntensity =
//                        j / ((double) this->graphSaved.getVertexList()->at(
//                                indexStart - i).getIntensities().intensities.size()) *
//                        ((double) this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().range);
//
//                int incrementOfScan = this->graphSaved.getVertexList()->at(
//                        indexStart - i).getIntensities().increment;
//                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
//                    Eigen::Vector4d positionOfIntensity(
//                            distanceOfIntensity,
//                            0,
//                            0,
//                            1);
//                    double rotationOfPoint = l / 400.0;
//                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0,
//                                                                                                                0,
//                                                                                                                rotationOfPoint);
//                    positionOfIntensity = rotationForBetterView * positionOfIntensity;
//
//                    positionOfIntensity = this->completeTransformationForCurrentScan *
//                                          generalHelpfulTools::getTransformationMatrixFromRPY(
//                                                  correctionFactorCreationRoll, 0, correctionFactorCreationMap) *
//                                          transformationOfIntensityRay *
//                                          rotationOfSonarAngleMatrix * positionOfIntensity;
//                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
//                    int indexX =
//                            (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
//                                   2) +
//                            NUMBER_OF_POINTS_MAP / 2;
//                    int indexY =
//                            (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
//                                   2) +
//                            NUMBER_OF_POINTS_MAP / 2;
//
//
//                    if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
//                        indexX >= 0) {
//                        //                    std::cout << indexX << " " << indexY << std::endl;
//                        //if index fits inside of our data, add that data. Else Ignore
//                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] =
//                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY] + 1;
//                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
//                        this->mapOfBunker[indexX + NUMBER_OF_POINTS_MAP * indexY] =
//                                this->mapOfBunker[indexX + NUMBER_OF_POINTS_MAP * indexY] +
//                                this->graphSaved.getVertexList()->at(
//                                        indexStart - i).getIntensities().intensities[j];
//                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
//                        //                    std::cout << "random: " << std::endl;
//                    }
//                }
//            }
//            i++;
//        } while (this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
//                 this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
//                 INTENSITY_SAVED_AND_KEYFRAME);
//
//    }
//
//};
//
//
//bool getNodes(ros::V_string &nodes) {
//    XmlRpc::XmlRpcValue args, result, payload;
//    args[0] = ros::this_node::getName();
//
//    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
//        return false;
//    }
//
//    ros::S_string node_set;
//    for (int i = 0; i < payload.size(); ++i) {
//        for (int j = 0; j < payload[i].size(); ++j) {
//            XmlRpc::XmlRpcValue val = payload[i][j][1];
//            for (int k = 0; k < val.size(); ++k) {
//                std::string name = payload[i][j][1][k];
//                node_set.insert(name);
//            }
//        }
//    }
//
//    nodes.insert(nodes.end(), node_set.begin(), node_set.end());
//
//    return true;
//}


int main(int argc, char **argv) {
    ros::init(argc, argv, "creationOfImageValentin");
    ros::NodeHandle n;
//
    ros::V_string nodes;
    int i = 0;
    std::string stringForRosClass;
//    if (SHOULD_USE_ROSBAG) {
//        getNodes(nodes);
//
//
//        for (i = 0; i < nodes.size(); i++) {
//
//            if (nodes[i].substr(1, 4) == "play") {
//                //            std::cout << "we found it" << std::endl;
//                break;
//            }
//        }
//        //    std::cout << nodes[i]+"/pause_playback" << std::endl;
//        ros::ServiceServer serviceResetEkf;
//
//
//        if (ros::service::exists(nodes[i] + "/pause_playback", true)) {
//
//        } else {
//            exit(-1);
//        }
//
//        stringForRosClass = nodes[i] + "/pause_playback";
//    }

    ros::start();
    ros::NodeHandle n_;
//    rosClassEKF rosClassForTests(n_, stringForRosClass);


    ros::Rate loop_rate(0.1);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Duration(2).sleep();

    while (ros::ok()) {
//        ros::spinOnce();

        //rosClassForTests.updateHilbertMap();
        //rosClassForTests.updateMap();


        loop_rate.sleep();
        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}


