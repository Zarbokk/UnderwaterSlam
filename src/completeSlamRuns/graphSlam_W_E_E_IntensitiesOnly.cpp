//
// Created by jurobotics on 13.09.21.
//

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "ping360_sonar/SonarEcho.h"
#include "../slamTools/generalHelpfulTools.h"
#include "../slamTools/slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include <hilbertMap.h>


#define NUMBER_OF_POINTS_DIMENSION 128
#define DIMENSION_OF_VOXEL_DATA 60
#define NUMBER_OF_POINTS_MAP 512.0
#define DIMENSION_OF_MAP 200.0


struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
};


//occupancyMap(256, NUMBER_OF_POINTS_DIMENSION, 70, hilbertMap::HINGED_FEATURES)
class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_) : graphSaved(3, INTENSITY_BASED_GRAPH),
                                      scanRegistrationObject(NUMBER_OF_POINTS_DIMENSION, NUMBER_OF_POINTS_DIMENSION / 2,
                                                             NUMBER_OF_POINTS_DIMENSION / 2,
                                                             NUMBER_OF_POINTS_DIMENSION / 2 - 1) {

        subscriberEKF = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(1).sleep();
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




        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
        transformationX180Degree(3, 3) = 1;

        graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                             Eigen::Vector3d(0, 0, 0), 0, ros::Time::now().toSec(),
                             FIRST_ENTRY);

//        std::deque<double> subgraphs{0.3, 2};
//        graphSaved.initiallizeSubGraphs(subgraphs, 10);
        this->sigmaScaling = 0.2;

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


    void scanCallback(const ping360_sonar::SonarEcho::ConstPtr &msg) {

        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        if (firstSonarInput) {

            intensityMeasurement intensityTMP;
            intensityTMP.angle = msg->angle / 400.0 * M_PI * 2.0;
            intensityTMP.time = msg->header.stamp.toSec();
            intensityTMP.increment = msg->step_size;
            intensityTMP.range = msg->range;
            std::vector<double> intensitiesVector;
            for (int i = 0; i < msg->intensities.size(); i++) {
                intensitiesVector.push_back(msg->intensities[i]);
            }
            intensityTMP.intensities = intensitiesVector;
            this->graphSaved.getVertexList()->at(0).setTimeStamp(msg->header.stamp.toSec());
            this->graphSaved.getVertexList()->at(0).setIntensities(intensityTMP);
            firstSonarInput = false;
            return;
        }
        //add a new edge and vertex to the graph defined by EKF and Intensity Measurement
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


        edge differenceOfEdge = this->calculatePoseDiffByTimeDepOnEKF(
                this->graphSaved.getVertexList()->back().getTimeStamp(), msg->header.stamp.toSec());


        Eigen::Matrix4d tmpTransformation = this->graphSaved.getVertexList()->back().getTransformation();
        tmpTransformation = tmpTransformation * differenceOfEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);


        this->graphSaved.addVertex(this->graphSaved.getVertexList()->back().getVertexNumber() + 1, pos, rot,
                                   this->graphSaved.getVertexList()->back().getCovariancePosition(),
                                   this->graphSaved.getVertexList()->back().getCovarianceQuaternion(),
                                   intensityTMP,
                                   msg->header.stamp.toSec(),
                                   INTENSITY_SAVED);

        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getVertexNumber() - 1,
                                 this->graphSaved.getVertexList()->back().getVertexNumber(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 Eigen::Vector3d(0.06, 0.06, 0),
                                 0.25 * 0.06, INTEGRATED_POSE,
                                 maxTimeOptimization);


//        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//        std::cout << "Time difference 4 = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        //test if a scan matching should be done

        int indexOfLastKeyframe;
        double angleDiff = angleBetweenLastKeyframeAndNow();// i think this is always true
        //std::cout << angleDiff << std::endl;
//        std::cout << msg->header.stamp.toSec() << std::endl;
//        std::cout << ros::Time::now().toSec() << std::endl;

        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (abs(angleDiff) > 2 * M_PI) {

            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);

            if (firstCompleteSonarScan) {
                firstCompleteSonarScan = false;
                return;
            }
            //angleDiff = angleBetweenLastKeyframeAndNow(false);
            indexOfLastKeyframe = getLastIntensityKeyframe();



            //match these voxels together
            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->back().getTransformation();
//            std::cout << "start Transformation:" << std::endl;
//            std::cout << this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation() << std::endl;
//            std::cout << "end Transformation:" << std::endl;
//            std::cout << this->graphSaved.getVertexList()->back().getTransformation() << std::endl;
//            std::cout << "initial Guess Transformation:" << std::endl;
//            std::cout << this->initialGuessTransformation << std::endl;

            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
                                                  this->initialGuessTransformation(0, 0));
            double fitnessScoreX, fitnessScoreY;

            // transform from 1 to 2
            this->currentTransformation = this->registrationOfTwoVoxels(indexOfLastKeyframe,
                                                                        this->graphSaved.getVertexList()->size() - 1,
                                                                        fitnessScoreX,
                                                                        fitnessScoreY,
                                                                        this->initialGuessTransformation, true, true,
                                                                        true);
//            double xValue = this->currentTransformation(0, 3);
//            double yValue = this->currentTransformation(1, 3);
//            this->currentTransformation(0, 3) = yValue;
//            this->currentTransformation(1, 3) = xValue;

            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)),
                    initialGuessAngle);




//            std::cout << "currentTransformation.inverse()" << std::endl;
//            std::cout << this->currentTransformation.inverse() << std::endl;

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
                graphSaved.addEdge(getLastIntensityKeyframe(),
                                   graphSaved.getVertexList()->size() - 1,
                                   this->currentTransformation.block<3, 1>(0, 3), qTMP,
                                   Eigen::Vector3d(0.05 * fitnessScoreX, 0.05 * fitnessScoreY, 0),
                                   0.02,
                                   LOOP_CLOSURE,
                                   maxTimeOptimization);//@TODO still not sure about size
            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }

            std::vector<int> holdStill{0};
            graphSaved.optimizeGraphWithSlam(false, holdStill,10.0);


            //add position and optimize/publish everything

//            std::deque<double> subgraphs{0.3, 2};
//            graphSaved.initiallizeSubGraphs(subgraphs, 10);
//
//            graphSaved.optimizeGraphWithSlamTopDown(false, 0.05, 1.0);
//            graphSaved.calculateCovarianceInCloseProximity(1.0);
//            graphSaved.resetHierachicalGraph();


            //this->resetMap();

            std::cout << "next: " << std::endl;
        }


        slamToolsRos::visualizeCurrentPoseGraph(this->graphSaved, this->publisherPathOverTime,
                                                this->publisherMarkerArray, this->sigmaScaling,this->publisherPoseSLAM);

//        this->lastAngle = msg->angle;

//        if (this->saveGraphStructure) {
//            std::cout << "saving graph " << std::endl;
//            this->graphSaved.saveGraphJson(this->saveStringGraph);
//            this->saveGraphStructure = false;
//        }
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
//        double threashHoldIntensity = averageIntensity * 4;
//        pcl::PointCloud<pcl::PointXYZ> returnCloud;
//        for (int i = 0; i < this->sonarIntensityList.size(); i++) {
//            for (int j = 0; j < this->sonarIntensityList[i].intensities.size(); j++) {
//                if (this->sonarIntensityList[i].intensities[j] > threashHoldIntensity && j > 4) {
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

    double createVoxelOfGraph(double voxelData[], int indexStart, Eigen::Matrix4d transformationInTheEndOfCalculation) {
        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
        //set zero voxel and index
        for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION; i++) {
            voxelDataIndex[i] = 0;
            voxelData[i] = 0;
        }


        int i = 0;
        do {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.


            //get position of current intensityRay
            Eigen::Matrix4d transformationOfIntensityRay =
                    this->graphSaved.getVertexList()->at(indexStart).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->at(indexStart - i).getTransformation();

            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             this->graphSaved.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);

            for (int j = 0;
                 j < this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) this->graphSaved.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().range);
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);


                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and NUMBER_OF_POINTS_DIMENSION the middle
                int indexX =
                        (int) (positionOfIntensity.x() / (DIMENSION_OF_VOXEL_DATA / 2) * NUMBER_OF_POINTS_DIMENSION /
                               2) +
                        NUMBER_OF_POINTS_DIMENSION / 2;
                int indexY =
                        (int) (positionOfIntensity.y() / (DIMENSION_OF_VOXEL_DATA / 2) * NUMBER_OF_POINTS_DIMENSION /
                               2) +
                        NUMBER_OF_POINTS_DIMENSION / 2;


                if (indexX < NUMBER_OF_POINTS_DIMENSION && indexY < NUMBER_OF_POINTS_DIMENSION && indexY >= 0 &&
                    indexX >= 0) {
//                    std::cout << indexX << " " << indexY << std::endl;
                    //if index fits inside of our data, add that data. Else Ignore
                    voxelDataIndex[indexX + NUMBER_OF_POINTS_DIMENSION * indexY] =
                            voxelDataIndex[indexX + NUMBER_OF_POINTS_DIMENSION * indexY] + 1;
//                    std::cout << "Index: " << voxelDataIndex[indexY + NUMBER_OF_POINTS_DIMENSION * indexX] << std::endl;
                    voxelData[indexX + NUMBER_OF_POINTS_DIMENSION * indexY] =
                            voxelData[indexX + NUMBER_OF_POINTS_DIMENSION * indexY] +
                            this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities[j];
//                    std::cout << "Intensity: " << voxelData[indexY + NUMBER_OF_POINTS_DIMENSION * indexX] << std::endl;
//                    std::cout << "random: " << std::endl;
                }
            }
            i++;
        } while (this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);
        double maximumOfVoxelData = 0;
        for (i = 0; i < NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION; i++) {
            if (voxelDataIndex[i] > 0) {
                voxelData[i] = voxelData[i] / voxelDataIndex[i];
                if (maximumOfVoxelData < voxelData[i]) {
                    maximumOfVoxelData = voxelData[i];
                }
                //std::cout << voxelData[i] << std::endl;

            }
        }// @TODO calculate the maximum and normalize "somehow"




        free(voxelDataIndex);
        return maximumOfVoxelData;
    }

    Eigen::Matrix4d registrationOfTwoVoxels(int indexVoxel1,
                                            int indexVoxel2,
                                            double &fitnessX, double &fitnessY, Eigen::Matrix4d initialGuess,
                                            bool useInitialAngle, bool useInitialTranslation,
                                            bool debug = false) {
        double goodGuessAlpha=-100;
        if(useInitialAngle){
            goodGuessAlpha= std::atan2(initialGuess(1, 0),
                                       initialGuess(0, 0));
        }

        //create a voxel of current scan (last rotation) and voxel of the rotation before that
        double *voxelData1;
        double *voxelData2;
        voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
        voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION);
        //still missing
        double maximumVoxel1 = createVoxelOfGraph(voxelData1, indexVoxel1, Eigen::Matrix4d::Identity());//get voxel
        double maximumVoxel2 = createVoxelOfGraph(voxelData2, indexVoxel2, Eigen::Matrix4d::Identity());//get voxel
//        double normalizationValue = 1;
//        for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION * NUMBER_OF_POINTS_DIMENSION; i++) {
//            voxelData1[i] = normalizationValue * voxelData1[i] / maximumVoxel1;
//            voxelData2[i] = normalizationValue * voxelData2[i] / maximumVoxel2;
//        }

        double estimatedAngle = this->scanRegistrationObject.sofftRegistrationVoxel2DRotationOnly(voxelData1,
                                                                                                  voxelData2,
                                                                                                  goodGuessAlpha,
                                                                                                  debug);
        Eigen::Matrix4d rotationMatrixTMP = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd tmpRotVec(estimatedAngle, Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d tmpMatrix3d = tmpRotVec.toRotationMatrix();
        rotationMatrixTMP.block<3, 3>(0, 0) = tmpMatrix3d;
        maximumVoxel1 = createVoxelOfGraph(voxelData1, indexVoxel1, Eigen::Matrix4d::Identity());//get voxel
        maximumVoxel2 = createVoxelOfGraph(voxelData2, indexVoxel2, rotationMatrixTMP);//get voxel

        if (true) {
            cv::Mat magTMP1(NUMBER_OF_POINTS_DIMENSION, NUMBER_OF_POINTS_DIMENSION, CV_64F, voxelData1);
            //add gaussian blur
            cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
            cv::Mat magTMP2(NUMBER_OF_POINTS_DIMENSION, NUMBER_OF_POINTS_DIMENSION, CV_64F, voxelData2);
            //add gaussian blur
            cv::GaussianBlur(magTMP2, magTMP2, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
//        cv::GaussianBlur(magTMP1, magTMP1, cv::Size(9, 9), 0);
        }

//        if (debug) {
//            std::ofstream myFile3, myFile6;
//            myFile3.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW1.csv");
//            myFile6.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW2.csv");
//            for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
//                for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
//
//                    myFile3 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
//                    myFile3 << "\n";
//                    myFile6 << voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
//                    myFile6 << "\n";
//                }
//            }
//            myFile3.close();
//            myFile6.close();
//        }

        Eigen::Vector2d translation = this->scanRegistrationObject.sofftRegistrationVoxel2DTranslation(voxelData1,
                                                                                                       voxelData2,
                                                                                                       fitnessX,
                                                                                                       fitnessY,
                                                                                                       (double) DIMENSION_OF_VOXEL_DATA /
                                                                                                       (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                                       initialGuessTransformation.block<3, 1>(0, 3),
                                                                                                       useInitialTranslation,
                                                                                                       debug);

        Eigen::Matrix4d estimatedRotationScans;//from second scan to first
        //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd rotation_vectorTMP(estimatedAngle, Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d tmpRotMatrix3d = rotation_vectorTMP.toRotationMatrix();
        estimatedRotationScans.block<3, 3>(0, 0) = tmpRotMatrix3d;
        estimatedRotationScans(0, 3) = translation.x();
        estimatedRotationScans(1, 3) = translation.y();
        estimatedRotationScans(2, 3) = 0;
        estimatedRotationScans(3, 3) = 1;


        maximumVoxel1 = createVoxelOfGraph(voxelData1, indexVoxel1, Eigen::Matrix4d::Identity());//get voxel
        maximumVoxel2 = createVoxelOfGraph(voxelData2, indexVoxel2, estimatedRotationScans);//get voxel

        if (debug) {
            std::ofstream myFile1, myFile2;
            myFile1.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel1.csv");
            myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel2.csv");
            for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
                for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
                    myFile1 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // real part
                    myFile1 << "\n";
                    myFile2 << voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
                    myFile2 << "\n";
                }
            }
            myFile1.close();
            myFile2.close();
        }


//        maximumVoxel1 = createVoxelOfGraph(voxelData1, indexVoxel1, estimatedRotationScans.inverse());//get voxel
//        maximumVoxel2 = createVoxelOfGraph(voxelData2, indexVoxel2, Eigen::Matrix4d::Identity());//get voxel
//
//        if (debug) {
//            std::ofstream myFile1, myFile2;
//            myFile1.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel1.csv");
//            myFile2.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/resultVoxel2.csv");
//            for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
//                for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
//                    myFile1 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION* i]; // real part
//                    myFile1 << "\n";
//                    myFile2 << voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
//                    myFile2 << "\n";
//                }
//            }
//            myFile1.close();
//            myFile2.close();
//        }


        return estimatedRotationScans;//should be the transformation matrix from 1 to 2
    }

    void resetMap(){
        for (int i = 0; i < NUMBER_OF_POINTS_MAP*NUMBER_OF_POINTS_MAP; i++) {
            //determine color:
            map.data.at(i) = 50;
        }
    }

public:
//    void updateHilbertMap() {
//        int numberOfPointsDataset = 5000;
//        std::cout << "started Hilbert Shift:" << std::endl;
//        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//        std::vector<dataPointStruct> dataSet;
//        bool foundDataset = this->getDatasetFromGraphForMap(numberOfPointsDataset / 5, dataSet, 5);
//
//        //return if dataset cannot be found
//        if (!foundDataset) {
//            return;
//        }
//        //@TODO normalization of the dataset
////        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
////        std::cout << "Time it takes to get the dataset = "
////                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
////                  << "[ms]" << std::endl;
////
////        std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
//        this->occupancyMap.trainClassifier(dataSet, (int) (numberOfPointsDataset));
//        nav_msgs::OccupancyGrid map = this->occupancyMap.createOccupancyMapOfHilbert(0, 0, 70, true);
////        std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
////        std::cout << "Time it takes to train the dataset = "
////                  << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count()
////                  << "[ms]" << std::endl;
//
//        map.header.stamp = ros::Time::now();
//        map.header.frame_id = "map_ned";
//        this->publisherOccupancyMap.publish(map);
//    }

    void updateMap() {
        //int numberOfPointsDataset = 5000;
        //std::cout << "started Hilbert Shift:" << std::endl;
        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        if (this->graphSaved.getVertexList()->size() < 10) {
            return;
        }
        std::vector<intensityValues> dataSet;
        double maximumIntensity = this->getDatasetFromGraphForMap(dataSet);


        //update map
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(0.01, 0.99);

        int indexTMP = (int)(dis(gen)*dataSet.size());


        if(dataSet.size()-indexTMP<201){
            return;
        }
        //for (int i = 0; i < dataSet.size(); i++) {
        for (int i = indexTMP; i < indexTMP+200; i++) {

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
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekfwithros");
    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_);


//    ros::spin();


    ros::Rate loop_rate(0.1);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Duration(2).sleep();

    while (ros::ok()) {
//        ros::spinOnce();

        //rosClassForTests.updateHilbertMap();
        rosClassForTests.updateMap();

        loop_rate.sleep();

        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}
