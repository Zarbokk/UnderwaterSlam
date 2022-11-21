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






#define NUMBER_OF_POINTS_DIMENSION 128
#define DIMENSION_OF_VOXEL_DATA 60
#define NUMBER_OF_POINTS_MAP 256
// 80 simulation 120 valentin
#define DIMENSION_OF_MAP 80.0
#define FACTOR_OF_THRESHOLD 0.9
#define IGNORE_DISTANCE_TO_ROBOT 1.5

//1: our 256 2:GICP 3:super 4: NDT d2d 5: NDT P2D 6: our global 256
#define WHICH_METHOD_USED 1
#define IGNORE_FIRST_STEPS 0
#define SHOULD_USE_ROSBAG true

struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
};


//occupancyMap(256, NUMBER_OF_POINTS_DIMENSION, 70, hilbertMap::HINGED_FEATURES)
class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_ , const std::string &nameOfTheServiceCall) : graphSaved(3, INTENSITY_BASED_GRAPH),
                                      scanRegistrationObject(NUMBER_OF_POINTS_DIMENSION, NUMBER_OF_POINTS_DIMENSION / 2,
                                                             NUMBER_OF_POINTS_DIMENSION / 2,
                                                             NUMBER_OF_POINTS_DIMENSION / 2 - 1) {

        subscriberEKF = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(3).sleep();
        subscriberIntensitySonar = n_.subscribe("sonar/intensity", 1000, &rosClassEKF::scanCallback, this);
        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);

        publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
        publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        publisherLastPCL = n_.advertise<sensor_msgs::PointCloud2>("lastPCL", 10);
        publisherRegistrationPCL = n_.advertise<sensor_msgs::PointCloud2>("registratedPCL", 10);
        publisherBeforeCorrection = n_.advertise<sensor_msgs::PointCloud2>("beforeCorrection", 10);
        publisherAfterCorrection = n_.advertise<sensor_msgs::PointCloud2>("afterCorrection", 10);
        publisherOccupancyMap = n_.advertise<nav_msgs::OccupancyGrid>("occupancyHilbertMap", 10);

        publisherPoseSLAM = n_.advertise<geometry_msgs::PoseStamped>("slamEndPose", 10);

        if (SHOULD_USE_ROSBAG) {
            pauseRosbag = n_.serviceClient<std_srvs::SetBoolRequest>(nameOfTheServiceCall);
        }

        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
        transformationX180Degree(3, 3) = 1;

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
    Eigen::Matrix4d currentTransformation;
    Eigen::Matrix4d initialGuessTransformation;
    Eigen::Matrix4d transformationX180Degree;

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

        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        intensityMeasurement intensityTMP;
        intensityTMP.angle = msg->angle / 400.0 * M_PI * 2.0;
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




        edge differenceOfEdge = this->calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved.getVertexList()->back().getTimeStamp(), msg->header.stamp.toSec());


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
        double angleDiff = angleBetweenLastKeyframeAndNow();// i think this is always true

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
            indexOfLastKeyframe = getLastIntensityKeyframe();



            //match these voxels together
            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->back().getTransformation();

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
                                                                    DIMENSION_OF_VOXEL_DATA);//get voxel


            double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                                    this->graphSaved.getVertexList()->size() -
                                                                    1,
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA);//get voxel

            // transform from 1 to 2
            this->currentTransformation = scanRegistrationObject.registrationOfTwoVoxelsSOFFTFast(
                    voxelData1, voxelData2,
                    this->initialGuessTransformation, true,
                    true, (double) DIMENSION_OF_VOXEL_DATA /
                          (double) NUMBER_OF_POINTS_DIMENSION, true, true);

            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)),
                    initialGuessAngle);


            std::cout << "FitnessScore X: " << fitnessScoreX << " FitnessScore Y: " << fitnessScoreY << std::endl;

            std::cout << "currentTransformation:" << std::endl;
            std::cout << this->currentTransformation << std::endl;

            std::cout << "initial Guess Transformation:" << std::endl;
            std::cout << this->initialGuessTransformation << std::endl;


            std::cout << "Initial guess angle: "
                      << initialGuessAngle
                      << std::endl;
            std::cout << "Registration angle: "
                      << std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)) << std::endl;
            std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter * 180 / M_PI
                      << std::endl;
            //only if angle diff is smaller than 10 degreece its ok
            if (abs(differenceAngleBeforeAfter) < 45.0 / 180.0 * M_PI) {
                Eigen::Quaterniond qTMP(this->currentTransformation.block<3, 3>(0, 0));
                graphSaved.addEdge(this->getLastIntensityKeyframe(),
                                   graphSaved.getVertexList()->size() - 1,
                                   this->currentTransformation.block<3, 1>(0, 3), qTMP,
                                   Eigen::Vector3d(0.05 * fitnessScoreX, 0.05 * fitnessScoreY, 0),
                                   0.02,
                                   LOOP_CLOSURE);//@TODO still not sure about size
            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }

////////////// look for loop closure  //////////////
            detectLoopClosure(this->graphSaved,this->scanRegistrationObject);





            std::chrono::steady_clock::time_point begin;
            std::chrono::steady_clock::time_point end;
            begin = std::chrono::steady_clock::now();

            this->graphSaved.optimizeGraph(true);
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
                                                this->publisherPoseSLAM,this->publisherMarkerArrayLoopClosures);
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

    void clearSavingsOfPoses(double upToTime) {
        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
        while (this->timeVector[0] < upToTime - 2) {//two second puffer

            this->rotationVector.pop_front();
            this->timeVector.pop_front();
            this->xPositionVector.pop_front();
            this->yPositionVector.pop_front();
            this->zPositionVector.pop_front();
        }
    }

    edge calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd) {
        //this is done to make sure 1 more message is coming from the EKF directly
        //ros::Duration(0.001).sleep();

        while (this->timeVector.empty()){
            ros::Duration(0.002).sleep();
        }


        while (endTimeToAdd > this->timeVector[this->timeVector.size() - 1]) {
            ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf");
            ros::Duration(0.001).sleep();
        }

        //@TEST
        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
        //find index of start and end
        int indexOfStart = 0;
        while (this->timeVector[indexOfStart] < startTimetoAdd && this->timeVector.size() > indexOfStart) {
            indexOfStart++;
        }
        if (indexOfStart > 0) {
            indexOfStart--;
        }

        int indexOfEnd = 0;
        while (this->timeVector[indexOfEnd] < endTimeToAdd && this->timeVector.size() > indexOfEnd) {
            indexOfEnd++;
        }
        indexOfEnd--;

        Eigen::Matrix4d transformationTMP = Eigen::Matrix4d::Identity();

        if (indexOfStart > 0) {
            double interpolationFactor = 1.0 - ((this->timeVector[indexOfStart + 1] - startTimetoAdd) /
                                                (this->timeVector[indexOfStart + 1] - this->timeVector[indexOfStart]));

            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[indexOfStart - 1].toRotationMatrix();
            transformationOfEKFStart(0, 3) = this->xPositionVector[indexOfStart];
            transformationOfEKFStart(1, 3) = this->yPositionVector[indexOfStart];
            transformationOfEKFStart(2, 3) = this->zPositionVector[indexOfStart];

            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[indexOfStart].toRotationMatrix();
            transformationOfEKFEnd(0, 3) = this->xPositionVector[indexOfStart + 1];
            transformationOfEKFEnd(1, 3) = this->yPositionVector[indexOfStart + 1];
            transformationOfEKFEnd(2, 3) = this->zPositionVector[indexOfStart + 1];

            transformationTMP = transformationTMP *
                                generalHelpfulTools::interpolationTwo4DTransformations(transformationOfEKFStart,
                                                                                       transformationOfEKFEnd,
                                                                                       interpolationFactor).inverse() *
                                transformationOfEKFEnd;
        }


        int i = indexOfStart + 1;
        while (i < indexOfEnd) {
            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[i].toRotationMatrix();
            transformationOfEKFEnd(0, 3) = this->xPositionVector[i];
            transformationOfEKFEnd(1, 3) = this->yPositionVector[i];
            transformationOfEKFEnd(2, 3) = this->zPositionVector[i];

            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[i - 1].toRotationMatrix();
            transformationOfEKFStart(0, 3) = this->xPositionVector[i - 1];
            transformationOfEKFStart(1, 3) = this->yPositionVector[i - 1];
            transformationOfEKFStart(2, 3) = this->zPositionVector[i - 1];

            transformationTMP = transformationTMP * (transformationOfEKFStart.inverse() * transformationOfEKFEnd);
            i++;
        }

        if (indexOfEnd > 0) {
            double interpolationFactor = ((endTimeToAdd - this->timeVector[indexOfEnd]) /
                                          (this->timeVector[indexOfEnd + 1] - this->timeVector[indexOfEnd]));

            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[indexOfEnd].toRotationMatrix();
            transformationOfEKFStart(0, 3) = this->xPositionVector[indexOfEnd];
            transformationOfEKFStart(1, 3) = this->yPositionVector[indexOfEnd];
            transformationOfEKFStart(2, 3) = this->zPositionVector[indexOfEnd];

            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[indexOfEnd + 1].toRotationMatrix();
            transformationOfEKFEnd(0, 3) = this->xPositionVector[indexOfEnd + 1];
            transformationOfEKFEnd(1, 3) = this->yPositionVector[indexOfEnd + 1];
            transformationOfEKFEnd(2, 3) = this->zPositionVector[indexOfEnd + 1];

            transformationTMP = transformationTMP * transformationOfEKFStart.inverse() *
                                generalHelpfulTools::interpolationTwo4DTransformations(transformationOfEKFStart,
                                                                                       transformationOfEKFEnd,
                                                                                       interpolationFactor);
        }
        //std::cout << diffMatrix << std::endl;
        Eigen::Vector3d tmpPosition = transformationTMP.block<3, 1>(0, 3);
        //set z pos diff to zero
        tmpPosition[2] = 0;
        Eigen::Quaterniond tmpRot(transformationTMP.block<3, 3>(0, 0));
        Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(tmpRot);
        //set rp on zero only yaw interesting
        tmpRot = generalHelpfulTools::getQuaternionFromRPY(0, 0, rpyTMP[2]);
        edge tmpEdge(0, 0, tmpPosition, tmpRot, Eigen::Vector3d(0, 0, 0), 0, 3,
                     INTEGRATED_POSE);

        return tmpEdge;
    }

//    pcl::PointCloud<pcl::PointXYZ> createPointCloudFromIntensities() {
//
//        //stupid way to find pointcloud:
//
//        double averageIntensity = 0;
//        for (int i = 0; i < this->sonarIntensityList.size(); i++) {
//            double currentAverage = 0;
//            for (int j = 0; j < this->sonarIntensityList[i].intensities.size(); j++) {
//                currentAverage += this->sonarIntensityList[i].intensities[j];
//            }
//            currentAverage = currentAverage / (double) this->sonarIntensityList[i].intensities.size();
//            averageIntensity += currentAverage;
//        }
//        averageIntensity = averageIntensity / (double) this->sonarIntensityList.size();
//
//        double threshHoldIntensity = averageIntensity * 4;
//        pcl::PointCloud<pcl::PointXYZ> returnCloud;
//        for (int i = 0; i < this->sonarIntensityList.size(); i++) {
//            for (int j = 0; j < this->sonarIntensityList[i].intensities.size(); j++) {
//                if (this->sonarIntensityList[i].intensities[j] > threshHoldIntensity && j > 4) {
//                    //calculate position of the point in xy coordinates
//                    double distanceFromRobot = (double) j * this->sonarIntensityList[i].range /
//                                               this->sonarIntensityList[i].number_of_samples;
//                    pcl::PointXYZ tmpPoint(distanceFromRobot * cos(this->sonarIntensityList[i].angle / 400 * 2 * M_PI),
//                                           distanceFromRobot * sin(this->sonarIntensityList[i].angle / 400 * 2 * M_PI),
//                                           0);
//                    returnCloud.push_back(tmpPoint);
//                }
//            }
//        }
//        return (returnCloud);
//    }

    bool saveGraph(commonbluerovmsg::saveGraph::Request &req, commonbluerovmsg::saveGraph::Response &res) {
        std::cout << "test for testing" << std::endl;
        this->saveGraphStructure = true;
        this->saveStringGraph = req.saveString;
        res.saved = true;
        return true;
    }

    double getDatasetFromGraphForMap(std::vector<intensityValues> &dataSet) {
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
//        std::vector<dataPointStruct> dataSet;

//        std::random_device rd;  // Will be used to obtain a seed for the random number engine
//        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
//        std::uniform_real_distribution<> dis(0.0, 1.0);
        double maxOverall = 0;
        for (int i = 0; i < this->graphSaved.getVertexList()->size(); i++) {
            intensityValues tmpInt;
            tmpInt.transformation = this->graphSaved.getVertexList()->at(i).getTransformation();
            tmpInt.intensity = this->graphSaved.getVertexList()->at(i).getIntensities();


            double it = *max_element(std::begin(tmpInt.intensity.intensities),
                                     std::end(tmpInt.intensity.intensities)); // C++11
            if (it > maxOverall) {
                maxOverall = it;
            }
            dataSet.push_back(tmpInt);
        }


        return maxOverall;
    }


    int getLastIntensityKeyframe() {//the absolut last entry is ignored
        int lastKeyframeIndex = this->graphSaved.getVertexList()->size() - 2;//ignore the last index
        //find last keyframe
        while (this->graphSaved.getVertexList()->at(lastKeyframeIndex).getTypeOfVertex() !=
               INTENSITY_SAVED_AND_KEYFRAME &&
               this->graphSaved.getVertexList()->at(lastKeyframeIndex).getTypeOfVertex() != FIRST_ENTRY) {
            lastKeyframeIndex--;
        }
        return lastKeyframeIndex;
    }

    double angleBetweenLastKeyframeAndNow() {
        double resultingAngleSonar = 0;
        double resultingAngleMovement = 0;
        int lastKeyframeIndex = getLastIntensityKeyframe();

        for (int i = lastKeyframeIndex; i < this->graphSaved.getVertexList()->size() - 1; i++) {
            Eigen::Quaterniond currentRot =
                    this->graphSaved.getVertexList()->at(i).getRotationVertex().inverse() *
                    this->graphSaved.getVertexList()->at(i + 1).getRotationVertex();


            Eigen::Vector3d rpy = generalHelpfulTools::getRollPitchYaw(currentRot);
            resultingAngleMovement += rpy(2);
            resultingAngleSonar += generalHelpfulTools::angleDiff(
                    this->graphSaved.getVertexList()->at(i + 1).getIntensities().angle,
                    this->graphSaved.getVertexList()->at(i).getIntensities().angle);
        }

        return resultingAngleMovement + resultingAngleSonar;


    }

    bool detectLoopClosure(graphSlamSaveStructure &tmpGraphSaved, scanRegistrationClass tmpregistrationClass) {
        Eigen::Vector3d estimatedPosLastPoint = tmpGraphSaved.getVertexList()->back().getPositionVertex();
        int ignoreStartLoopClosure = 450;
        int ignoreEndLoopClosure = 800;
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
            if ((d <= r1 - r2 || d <= r2 - r1 || d < r1 + r2 || d == r1 + r2) && d<100.0) {
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
            Eigen::Matrix4d potentialLoopClosureTransformation;

            //create voxel
            double *voxelData1;
            double *voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
            double maximumVoxel1 = slamToolsRos::createVoxelOfGraph(voxelData1,
                                                                    potentialKey,
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA);//get voxel

            double maximumVoxel2 = slamToolsRos::createVoxelOfGraph(voxelData2,
                                                                    this->graphSaved.getVertexList()->back().getKey(),
                                                                    Eigen::Matrix4d::Identity(),
                                                                    NUMBER_OF_POINTS_DIMENSION, this->graphSaved,
                                                                    IGNORE_DISTANCE_TO_ROBOT,
                                                                    DIMENSION_OF_VOXEL_DATA);//get voxel

            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(potentialKey).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->back().getTransformation();

            // transform from 1 to 2
            potentialLoopClosureTransformation = tmpregistrationClass.registrationOfTwoVoxelsSOFFTFast(voxelData1,
                                                                                                    voxelData2,
                                                                                                    this->initialGuessTransformation,
                                                                                                    true, true,
                                                                                                    (double) DIMENSION_OF_VOXEL_DATA /
                                                                                                    (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                                    true,
                                                                                                    true);
//            fitnessScore = scalingAllg * sqrt(fitnessScore);
            // fitnessScore < scalingAllg * 4 * 0.1
            if (true) {
                std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
//                if (fitnessScore < 0.01) {
//                    std::cout << "FitnessScore Very Low: " << fitnessScore << std::endl;
//                    fitnessScore = 0.01;
//                }
                Eigen::Vector3d currentPosDiff;
                Eigen::Quaterniond currentRotDiff(potentialLoopClosureTransformation.block<3, 3>(0, 0));
                currentPosDiff.x() = potentialLoopClosureTransformation(0, 3);
                currentPosDiff.y() = potentialLoopClosureTransformation(1, 3);
                currentPosDiff.z() = 0;
                Eigen::Vector3d positionCovariance(fitnessScore, fitnessScore, 0);
                tmpGraphSaved.addEdge(potentialKey, tmpGraphSaved.getVertexList()->back().getKey(), currentPosDiff,
                                   currentRotDiff, positionCovariance,
                        1, LOOP_CLOSURE);
                foundLoopClosure = true;
                loopclosureNumber++;
                if (loopclosureNumber > 1) { break; }// break if multiple loop closures are found
            }
        }
        if (foundLoopClosure) {
            return true;
        }
        return false;
    }


public:

    void updateMap() {

        if (this->graphSaved.getVertexList()->size() < 10) {
            return;
        }

        std::vector<intensityValues> dataSet;
        double maximumIntensity = this->getDatasetFromGraphForMap(dataSet);


        //update map
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(0.01, 0.99);

        int indexTMP = (int) (dis(gen) * dataSet.size());


        if (dataSet.size() - indexTMP < 201) {
            return;
        }
        //for (int i = 0; i < dataSet.size(); i++) {
        for (int i = indexTMP; i < indexTMP + 200; i++) {

            Eigen::Matrix4d transformationIntensity = dataSet[i].transformation;
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             dataSet[i].intensity.angle);
            double sumIntensity = 0;
            for (int j = 0; j < dataSet[i].intensity.intensities.size(); j++) {
                //determine color:
                double distanceOfIntensity =
                        j / ((double) dataSet[i].intensity.intensities.size()) *
                        ((double) dataSet[i].intensity.range);
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);

                positionOfIntensity = transformationIntensity * rotationOfSonarAngleMatrix * positionOfIntensity;
                //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and NUMBER_OF_POINTS_DIMENSION the middle
                int indexX =
                        (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                               2) +
                        (int) NUMBER_OF_POINTS_MAP / 2;
                int indexY =
                        (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                               2) +
                        (int) NUMBER_OF_POINTS_MAP / 2;

                if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                    indexX >= 0) {

                    double intensityImportance = ((double) (j)) / ((double) dataSet[i].intensity.intensities.size());

                    this->map.data[indexX + NUMBER_OF_POINTS_MAP * indexY] =
                            this->map.data[indexX + NUMBER_OF_POINTS_MAP * indexY] *
                            (0.5 - (1.0 - intensityImportance)) +
                            dataSet[i].intensity.intensities[j] / maximumIntensity * 100 *
                            (0.5 + (1.0 - intensityImportance));//normalization with /maximumIntensity*100
                }


                sumIntensity = sumIntensity + dataSet[i].intensity.intensities[j] / maximumIntensity * 100 + 1;

//                if (sumIntensity > 400) {
//                    break;
//                }
            }
            i++;
        }

        map.header.stamp = ros::Time::now();
        map.header.frame_id = "map_ned";
        this->publisherOccupancyMap.publish(map);
    }

    void createImageOfAllScans() {

        std::vector<intensityValues> dataSet;
        double maximumIntensity = this->getDatasetFromGraphForMap(dataSet);
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

            Eigen::Matrix4d transformationOfIntensityRay =
                    generalHelpfulTools::getTransformationMatrixFromRPY(0, 0, 90.0 / 180.0 * M_PI) *
                    generalHelpfulTools::getTransformationMatrixFromRPY(180.0 / 180.0 * M_PI, 0, 0) *
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
