//
// Created by jurobotics on 13.09.21.
//
#include "ekf.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "ping360_sonar/SonarEcho.h"
#include "generalHelpfulTools.h"
#include <slamToolsRos.h>
#include "commonbluerovmsg/saveGraph.h"
#include <hilbertMap.h>


#define NUMBER_OF_POINTS_DIMENSION 128
class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_) : graphSaved(3, INTENSITY_BASED_GRAPH),
                                      scanRegistrationObject(),
                                      occupancyMap(256, NUMBER_OF_POINTS_DIMENSION, 70, hilbertMap::HINGED_FEATURES) {

        subscriberEKF = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
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

        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
        transformationX180Degree(3, 3) = 1;

        graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                             Eigen::Vector3d(0, 0, 0), 0, ros::Time::now().toSec(),
                             FIRST_ENTRY);

        std::deque<double> subgraphs{1, 3};
        graphSaved.initiallizeSubGraphs(subgraphs, 10);
        this->sigmaScaling = 1.0;

//        this->maxTimeOptimization = 1.0;

        this->firstSonarInput = true;
        this->saveGraphStructure = false;

        this->occupancyMap.createRandomMap();
        this->maxTimeOptimization = 1.0;

    }


private:

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
    bool firstSonarInput, saveGraphStructure;
    std::string saveStringGraph;
    double maxTimeOptimization;
    hilbertMap occupancyMap;


    void scanCallback(const ping360_sonar::SonarEcho::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        if (firstSonarInput) {

            intensityMeasurement intensityTMP;
            intensityTMP.angle = msg->angle / 400.0 * M_PI * 2.0;
            intensityTMP.time = msg->header.stamp.toSec();
            intensityTMP.size = msg->intensities.size();
            intensityTMP.increment = msg->step_size;
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
        intensityTMP.size = msg->intensities.size();
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
                                   msg->header.stamp.toSec(),
                                   INTENSITY_SAVED);

        this->graphSaved.addEdge(this->graphSaved.getVertexList()->back().getVertexNumber() - 1,
                                 this->graphSaved.getVertexList()->back().getVertexNumber(),
                                 differenceOfEdge.getPositionDifference(), differenceOfEdge.getRotationDifference(),
                                 Eigen::Vector3d(0.06, 0.06, 0),
                                 0.25 * 0.06, INTEGRATED_POSE,
                                 maxTimeOptimization);
        //test if a scan matching should be done

        int indexOfLastKeyframe;
        double angleDiff = angleBetweenLastKeyframeAndNow();
        // best would be scan matching between this angle and transformation based last angle( i think this is currently done)
        if (angleDiff > 2*M_PI) {
            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);
            //create a voxel of current scan (last rotation) and voxel of the rotation before that
            double* voxelData1;
            double* voxelData2;
            voxelData1 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION*NUMBER_OF_POINTS_DIMENSION);
            voxelData2 = (double *) malloc(sizeof(double) * NUMBER_OF_POINTS_DIMENSION*NUMBER_OF_POINTS_DIMENSION);
            //still missing
            createVoxelOfGraph(voxelData1,this->graphSaved.getVertexList()->size()-1);//get voxel
            createVoxelOfGraph(voxelData2,indexOfLastKeyframe);//get voxel


            //match these voxels together
            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(getLastIntensityKeyframe()).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->back().getTransformation();


            double fitnessScoreX, fitnessScoreY;

            //we need inverse transformation
            this->currentTransformation = scanRegistrationObject.sofftRegistrationVoxel(
                    voxelData1, voxelData2,
                    fitnessScoreX, fitnessScoreY,
                    std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)),
                    false).inverse();

            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                    std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)),
                    std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)));


            std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter << std::endl;
            //only if angle diff is smaller than 10 degreece its ok
            if (abs(differenceAngleBeforeAfter) < 10.0 / 180.0 * M_PI) {
                Eigen::Quaterniond qTMP(this->currentTransformation.block<3, 3>(0, 0));
                graphSaved.addEdge(getLastIntensityKeyframe(),
                                   graphSaved.getVertexList()->size() - 1,
                                   this->currentTransformation.block<3, 1>(0, 3), qTMP,
                                   Eigen::Vector3d(fitnessScoreX, fitnessScoreY, 0),
                                   0.1,
                                   LOOP_CLOSURE,
                                   maxTimeOptimization);//@TODO still not sure about size
            } else {
                std::cout << "we just skipped that registration" << std::endl;
            }


            //add position and optimize/publish everything

            graphSaved.optimizeGraphWithSlamTopDown(false, 0.05, 1.0);
            graphSaved.calculateCovarianceInCloseProximity(1.0);
            //@TODO visualization has to be created anew
//            slamToolsRos::visualizeCurrentGraph(this->graphSaved, this->publisherPathOverTime,
//                                                this->publisherKeyFrameClouds,
//                                                this->publisherMarkerArray, this->sigmaScaling,
//                                                this->publisherPathOverTimeGT,
//                                                NULL, this->publisherMarkerArrayLoopClosures,
//                                                NULL, this->numberOfEdgesBetweenScans);
            std::cout << "next: " << std::endl;
        }


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
            double interpolationFactor = (this->timeVector[indexOfStart - 1] - startTimetoAdd) /
                                         (this->timeVector[indexOfStart] - this->timeVector[indexOfStart - 1]);

            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[indexOfStart - 1].toRotationMatrix();
            transformationOfEKFStart(0, 3) = this->xPositionVector[indexOfStart - 1];
            transformationOfEKFStart(1, 3) = this->yPositionVector[indexOfStart - 1];
            transformationOfEKFStart(2, 3) = this->zPositionVector[indexOfStart - 1];

            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[indexOfStart].toRotationMatrix();
            transformationOfEKFEnd(0, 3) = this->xPositionVector[indexOfStart];
            transformationOfEKFEnd(1, 3) = this->yPositionVector[indexOfStart];
            transformationOfEKFEnd(2, 3) = this->zPositionVector[indexOfStart];

            transformationTMP = transformationTMP *
                                generalHelpfulTools::interpolationTwo4DTransformations(transformationOfEKFStart,
                                                                                       transformationOfEKFEnd,
                                                                                       interpolationFactor);
        }


        int i = indexOfStart;
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
            double interpolationFactor = (endTimeToAdd - this->timeVector[indexOfEnd - 1]) /
                                         (this->timeVector[indexOfEnd] - this->timeVector[indexOfEnd - 1]);

            Eigen::Matrix4d transformationOfEKFStart = Eigen::Matrix4d::Identity();
            transformationOfEKFStart.block<3, 3>(0, 0) = this->rotationVector[indexOfStart - 1].toRotationMatrix();
            transformationOfEKFStart(0, 3) = this->xPositionVector[indexOfStart - 1];
            transformationOfEKFStart(1, 3) = this->yPositionVector[indexOfStart - 1];
            transformationOfEKFStart(2, 3) = this->zPositionVector[indexOfStart - 1];

            Eigen::Matrix4d transformationOfEKFEnd = Eigen::Matrix4d::Identity();
            transformationOfEKFEnd.block<3, 3>(0, 0) = this->rotationVector[indexOfStart].toRotationMatrix();
            transformationOfEKFEnd(0, 3) = this->xPositionVector[indexOfStart];
            transformationOfEKFEnd(1, 3) = this->yPositionVector[indexOfStart];
            transformationOfEKFEnd(2, 3) = this->zPositionVector[indexOfStart];

            transformationTMP = transformationTMP *
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

    bool getDatasetFromGraphForHilbertMap(int numberOfPointsInDataset, std::vector<dataPointStruct> &dataSet) {
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
//        std::vector<dataPointStruct> dataSet;

        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(0.0, 1.0);

        for (int i = 0; i < numberOfPointsInDataset; i++) {

            //get some random number indexVertex and Random Number of Point
            int indexVertex = (int) (dis(gen) *
                                     (double) (this->graphSaved.getVertexList()->size() - 1));

            int indexOfPointInIntensitieList = (int) (dis(gen) *
                                                      (double) this->graphSaved.getVertexList()->at(
                                                          indexVertex).getIntensities().size-1);


            Eigen::Matrix4d transformationOfVertex = this->graphSaved.getVertexList()->at(
                    indexVertex).getTransformation();

            double distanceOfIntensity = indexOfPointInIntensitieList*this->graphSaved.getVertexList()->at(indexVertex).getIntensities().increment;


            Eigen::Vector4d pointPos(
                    distanceOfIntensity,
                    0,
                    0,
                    1);

            //pointPos has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Quaterniond rotationOfSonarAngleQuaternion = generalHelpfulTools::getQuaternionFromRPY(0,0,this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle);
            pointPos = transformationOfVertex * pointPos;
            dataPointStruct tmpDP;
            tmpDP.x = pointPos.x();
            tmpDP.y = pointPos.y();
            tmpDP.z = pointPos.z();
            tmpDP.occupancy = this->graphSaved.getVertexList()->at(indexVertex).getIntensities().intensities.at(indexOfPointInIntensitieList);
            dataSet.push_back(tmpDP);

            //create 3 additional point where occupancy = -1
            for (int j = 0; j < 1; j++) {
                Eigen::Vector4d pointPosTwo(
                        this->graphSaved.getVertexList()->at(
                                indexVertex).getPointCloudCorrected()->points[indexOfPointInIntensitieList].x,
                        this->graphSaved.getVertexList()->at(
                                indexVertex).getPointCloudCorrected()->points[indexOfPointInIntensitieList].y,
                        0,
                        1);


                //double randomNumber = dis(gen);// should be between 0 and 1
                pointPosTwo = transformationOfVertex * dis(gen) * pointPosTwo;
                tmpDP.x = pointPosTwo.x();
                tmpDP.y = pointPosTwo.y();
                tmpDP.z = pointPosTwo.z();
                tmpDP.occupancy = -1;
                dataSet.push_back(tmpDP);
            }
        }
        return true;
    }


    int getLastIntensityKeyframe(){//the absolut last entry is ignored
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
        double resultingAngle = 0;

        int lastKeyframeIndex = getLastIntensityKeyframe();

        for (int i = lastKeyframeIndex; i < this->graphSaved.getVertexList()->size() - 1; i++) {
            Eigen::Quaterniond currentRot =
                    this->graphSaved.getVertexList()->at(lastKeyframeIndex).getRotationVertex().inverse() *
                    this->graphSaved.getVertexList()->at(lastKeyframeIndex + 1).getRotationVertex();
            Eigen::Vector3d rpy = generalHelpfulTools::getRollPitchYaw(currentRot);
            resultingAngle+=rpy(2)+this->graphSaved.getVertexList()->at(lastKeyframeIndex+1).getIntensities().increment;
        }

        return resultingAngle;
    }

    void createVoxelOfGraph(double voxelData[],int indexStart){

    }

public:
    void updateHilbertMap() {
        int numberOfPointsDataset = 5000;
        std::cout << "started Hilbert Shift:" << std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::vector<dataPointStruct> dataSet;
        bool foundDataset = this->getDatasetFromGraphForHilbertMap(numberOfPointsDataset, dataSet);
        //return if dataset cannot be found
        if (!foundDataset) {
            return;
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time it takes to get the dataset = "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << "[ms]" << std::endl;

        std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
        this->occupancyMap.trainClassifier(dataSet, (int) (numberOfPointsDataset * 1.8));
        nav_msgs::OccupancyGrid map = this->occupancyMap.createOccupancyMapOfHilbert(0, 0, 70, true);
        std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
        std::cout << "Time it takes to train the dataset = "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count()
                  << "[ms]" << std::endl;

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
    ros::Duration(15).sleep();

    while (ros::ok()) {
//        ros::spinOnce();

        rosClassForTests.updateHilbertMap();
        loop_rate.sleep();

        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}
