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

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_) : graphSaved(3), scanRegistrationObject() {

//        lastUpdateEkf = currentEkf.copyEKF();
//        subscriberIMU = n_.subscribe("mavros/imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        subscriberEKF = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
//        subscriberDepth = n_.subscribe("mavros/altitude_frd", 1000, &rosClassEKF::depthCallback, this);
        subscriberIntensitySonar = n_.subscribe("sonar/intensity", 1000, &rosClassEKF::scanCallback, this);
        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);


//        publisherPoseEkf = n_.advertise<geometry_msgs::PoseStamped>("publisherPoseEkf", 10);
//        publisherTwistEkf = n_.advertise<geometry_msgs::TwistStamped>("publisherTwistEkf", 10);
        publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
        publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        publisherLastPCL = n_.advertise<sensor_msgs::PointCloud2>("lastPCL", 10);
        publisherRegistrationPCL = n_.advertise<sensor_msgs::PointCloud2>("registratedPCL", 10);
        publisherBeforeCorrection = n_.advertise<sensor_msgs::PointCloud2>("beforeCorrection", 10);
        publisherAfterCorrection = n_.advertise<sensor_msgs::PointCloud2>("afterCorrection", 10);


        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
        transformationX180Degree(3, 3) = 1;
//        std::cout << ros::Time::now().toSec() << std::endl;
        graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                             Eigen::Vector3d(0, 0, 0), 0, ros::Time::now().toSec(),
                             graphSlamSaveStructure::FIRST_ENTRY);

        std::deque<double> subgraphs{1, 3};
        graphSaved.initiallizeSubGraphs(subgraphs, 10);
        this->sigmaScaling = 1.0;
        this->timeLastFullScan = 0;
//        this->maxTimeOptimization = 1.0;
        this->numberOfEdgesBetweenScans = 30;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL3(new pcl::PointCloud<pcl::PointXYZ>);
        this->currentScan = tmpPCL1;
        this->previousScan = tmpPCL2;
        this->Final = tmpPCL3;
        this->startTimeOfCorrection = 0;
        this->firstScan = true;
        this->saveGraphStructure = false;
        this->numberOfScan = 0;
    }


private:

    ros::Subscriber subscriberEKF, subscriberIntensitySonar;
    ros::Publisher publisherPoseSLAM;
    ros::ServiceServer serviceSaveGraph;
    std::mutex stateEstimationMutex;

    //GraphSlam things
    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;

    //PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previousScan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final;
    std::vector<ping360_sonar::SonarEcho> sonarIntensityList;
    //Matrices:
    Eigen::Matrix4d currentTransformation;
    Eigen::Matrix4d initialGuessTransformation;
    Eigen::Matrix4d transformationX180Degree;

    //EKF savings
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
    std::deque<Eigen::Quaterniond> rotationVector;
    int numberOfEdgesBetweenScans;
    double timeLastFullScan;
    double timeCurrentFullScan;
    double fitnessScore;
    double sigmaScaling;
    //double maxTimeOptimization;
    double lastAngle;
    double startTimeOfCorrection;
    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstScan, saveGraphStructure;
    std::string saveStringGraph;
    int numberOfScan;

    void slamCallback(const geometry_msgs::PoseStamped &msg) {
        Eigen::Quaterniond tmpRot;
        tmpRot.x() = msg.pose.orientation.x;
        tmpRot.y() = msg.pose.orientation.y;
        tmpRot.z() = msg.pose.orientation.z;
        tmpRot.w() = msg.pose.orientation.w;

        Eigen::Vector3d orientationRPY = generalHelpfulTools::getRollPitchYaw(tmpRot);
    }

    void scanCallback(const ping360_sonar::SonarEcho::ConstPtr &msg) {

        if (this->startTimeOfCorrection == 0) {
            this->startTimeOfCorrection = msg->header.stamp.toSec();
            this->graphSaved.getVertexByIndex(0)->setTimeStamp(msg->header.stamp.toSec());
        }

        if (this->timeVector.empty()) {
            return;
        }
        //we always assume positive turning sonars for now
        if (this->lastAngle > msg->angle) {
            double timeDiffScans = msg->header.stamp.toSec() - this->startTimeOfCorrection;
            double lastTimeofScan = this->startTimeOfCorrection;//msg->header.stamp.toSec()-this->startTimeOfCorrection;
            this->startTimeOfCorrection = msg->header.stamp.toSec();
            //call slam in thread.
            std::deque<edge> posDiffOverTimeByTime = calculatePoseDiffByTimeDepOnEKF(lastTimeofScan,
                                                                                     this->startTimeOfCorrection);
            //this->posDiffOverTimeEdges.clear();
            slamToolsRos::appendEdgesToGraph(this->graphSaved, posDiffOverTimeByTime, 0.06, 0.25, timeDiffScans * 0.1,
                                             this->numberOfEdgesBetweenScans);

            //create point cloud based on scan
            *this->currentScan = createPointCloudFromIntensities();


            this->graphSaved.getVertexList().back().setPointCloudRaw(this->currentScan);

            //correct the scan depending on the EKF callback
            slamToolsRos::correctPointCloudAtPos(this->graphSaved.getVertexList().back().getVertexNumber(),
                                                 this->graphSaved, 0, 2 * M_PI, false,
                                                 Eigen::Matrix4d::Identity());

            //find position of last pcl entry
            int positionLastPcl = 1;
            int sizeOfVertexList = this->graphSaved.getVertexList().size();
            while (true) {
                if (this->graphSaved.getVertexList()[sizeOfVertexList - positionLastPcl - 1].getTypeOfVertex() ==
                    graphSlamSaveStructure::POINT_CLOUD_USAGE ||
                    this->graphSaved.getVertexList()[sizeOfVertexList - positionLastPcl - 1].getTypeOfVertex() ==
                    graphSlamSaveStructure::FIRST_ENTRY) {
                    break;
                }
                positionLastPcl++;
            }
            positionLastPcl++;
            if (this->firstScan) {
//                *this->previousScan = *this->graphSaved.getVertexList().back().getPointCloudCorrected();
                this->firstScan = false;

            } else {


                //make scan matching with last scan, only if not the first scan available
                this->initialGuessTransformation =
                        this->graphSaved.getVertexList()[this->graphSaved.getVertexList().size() -
                                                         positionLastPcl].getTransformation().inverse() *
                        this->graphSaved.getVertexList().back().getTransformation();


//                std::cout << std::atan2(this->initialGuessTransformation(1,0),this->initialGuessTransformation(0,0))*180/M_PI << std::endl;
//                std::cout << this->initialGuessTransformation << std::endl;
//                double cellSize = 0.5;
//                this->currentTransformation = scanRegistrationObject.sofftRegistration(
//                        this->previousScan, this->graphSaved.getVertexList().back().getPointCloudCorrected(), cellSize,
//                        std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)) *
//                        180 / M_PI);
//                std::cout << this->initialGuessTransformation << std::endl;
//                std::cout << this->currentTransformation << std::endl;
//                std::cout << this->currentTransformation.inverse() << std::endl;
////                this->currentTransformation = scanRegistrationClass::generalizedIcpRegistration(
////                        this->graphSaved.getVertexList().back().getPointCloudCorrected(), this->previousScan, this->Final,
////                        this->fitnessScore,
////                        this->initialGuessTransformation);
//                //std::cout << "current Fitness Score: " << sqrt(this->fitnessScore) << std::endl;
//
//                Eigen::Quaterniond qTMP(this->currentTransformation.block<3, 3>(0, 0));
//                graphSaved.addEdge(this->graphSaved.getVertexList().size() - positionLastPcl,
//                                   graphSaved.getVertexList().size() - 1,
//                                   this->currentTransformation.block<3, 1>(0, 3), qTMP,
//                                   Eigen::Vector3d(sqrt(this->fitnessScore), sqrt(this->fitnessScore), 0),
//                                   0.25 * sqrt(this->fitnessScore),
//                                   graphSlamSaveStructure::POINT_CLOUD_USAGE,
//                                   timeDiffScans * 0.1);//@TODO still not sure about size


                pcl::io::savePCDFileASCII("/home/tim-linux/dataFolder/gazeboDataScansPCL/scanNumber_" + std::to_string(this->numberOfScan) + ".pcd",
                                          *this->graphSaved.getVertexList().back().getPointCloudCorrected());
                this->numberOfScan++;
//                pcl::io::savePCDFileASCII("/home/tim-linux/dataFolder/savingRandomPCL/secondPCL.pcd", *this->previousScan);
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloudPlotOnly(
                        new pcl::PointCloud<pcl::PointXYZ>);
                *tmpCloudPlotOnly = *currentScan;

                //pcl::transformPointCloud(*tmpCloudPlotOnly, *tmpCloudPlotOnly, transformationImu2PCL);
                slamToolsRos::debugPlotting(this->previousScan, this->Final, tmpCloudPlotOnly,
                                            this->graphSaved.getVertexList().back().getPointCloudCorrected(),
                                            this->publisherLastPCL, this->publisherRegistrationPCL,
                                            this->publisherBeforeCorrection, this->publisherAfterCorrection);
                slamToolsRos::detectLoopClosure(this->graphSaved, this->sigmaScaling, 1.0,
                                                timeDiffScans * 0.1);//was 1.0
            }
            //add position and optimize/publish everything


//            std::vector<int> holdStill{0};
//            graphSaved.optimizeGraphWithSlam(false, holdStill,timeDiffScans * 0.1);

            graphSaved.optimizeGraphWithSlamTopDown(false, 0.05, timeDiffScans * 0.1);
            graphSaved.calculateCovarianceInCloseProximity(timeDiffScans * 0.1);

            slamToolsRos::visualizeCurrentGraph(this->graphSaved, this->publisherPathOverTime,
                                                this->publisherKeyFrameClouds,
                                                this->publisherMarkerArray, this->sigmaScaling,
                                                this->publisherPathOverTimeGT,
                                                NULL, this->publisherMarkerArrayLoopClosures,
                                                NULL, this->numberOfEdgesBetweenScans);
            std::cout << "next: " << std::endl;
//            std::cout << this->timeCurrentFullScan << std::endl;
//            this->timeLastFullScan = this->timeCurrentFullScan;
            *this->previousScan = *this->graphSaved.getVertexList().back().getPointCloudCorrected();

            sonarIntensityList.clear();
            clearSavingsOfPoses(msg->header.stamp.toSec());

        }


        this->lastAngle = msg->angle;
        this->sonarIntensityList.push_back(*msg);

        geometry_msgs::PoseStamped newMsg;
        newMsg.header.stamp = msg->header.stamp;
        newMsg.pose.position.x = this->graphSaved.getVertexList().back().getPositionVertex().x();
        newMsg.pose.position.y = this->graphSaved.getVertexList().back().getPositionVertex().y();
        newMsg.pose.position.z = 0;

        newMsg.pose.orientation.x = this->graphSaved.getVertexList().back().getRotationVertex().x();
        newMsg.pose.orientation.y = this->graphSaved.getVertexList().back().getRotationVertex().y();
        newMsg.pose.orientation.z = this->graphSaved.getVertexList().back().getRotationVertex().z();
        newMsg.pose.orientation.w = this->graphSaved.getVertexList().back().getRotationVertex().w();

        this->slamCallback(newMsg);

        if (this->saveGraphStructure) {
            std::cout << "saving graph " << std::endl;
            this->graphSaved.saveGraphJson(this->saveStringGraph);
            this->saveGraphStructure = false;
        }

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

    std::deque<edge> calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd) {
        std::cout << startTimetoAdd << " : " << endTimeToAdd << std::endl;
        if (endTimeToAdd - startTimetoAdd > 20) {
            std::cout << endTimeToAdd - startTimetoAdd << std::endl;
        }

        std::lock_guard<std::mutex> lock(this->stateEstimationMutex);
        std::deque<edge> tmpEdgeList;
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


        int i = indexOfStart;
        while (i < indexOfEnd) {
            Eigen::Matrix4d transformationStart = Eigen::Matrix4d::Zero();
            transformationStart.block<3, 3>(0, 0) = this->rotationVector[i].toRotationMatrix();
            transformationStart(0, 3) = this->xPositionVector[i];
            transformationStart(1, 3) = this->yPositionVector[i];
            transformationStart(2, 3) = this->zPositionVector[i];
            transformationStart(3, 3) = 1;

            Eigen::Matrix4d transformationEnd = Eigen::Matrix4d::Zero();
            transformationEnd.block<3, 3>(0, 0) = this->rotationVector[i + 1].toRotationMatrix();
            transformationEnd(0, 3) = this->xPositionVector[i + 1];
            transformationEnd(1, 3) = this->yPositionVector[i + 1];
            transformationEnd(2, 3) = this->zPositionVector[i + 1];
            transformationEnd(3, 3) = 1;
            Eigen::Matrix4d diffMatrix = transformationStart.inverse() * transformationEnd;

            Eigen::Vector3d tmpPosition = diffMatrix.block<3, 1>(0, 3);
            //set z pos diff to zero
            tmpPosition[2] = 0;
            Eigen::Quaterniond tmpRot(diffMatrix.block<3, 3>(0, 0));
            Eigen::Vector3d rpyTMP = generalHelpfulTools::getRollPitchYaw(tmpRot);
            //set rp on zero only yaw interesting
            tmpRot = generalHelpfulTools::getQuaternionFromRPY(0, 0, rpyTMP[2]);
            edge tmpEdge(0, 0, tmpPosition, tmpRot, Eigen::Vector3d(0, 0, 0), 0, 3,
                         graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            tmpEdge.setTimeStamp(this->timeVector[i + 1]);
            tmpEdgeList.push_back(tmpEdge);
            i++;
        }
        return tmpEdgeList;
    }

    pcl::PointCloud<pcl::PointXYZ> createPointCloudFromIntensities() {

        //stupid way to find pointcloud:

        double averageIntensity = 0;
        for (int i = 0; i < this->sonarIntensityList.size(); i++) {
            double currentAverage = 0;
            for (int j = 0; j < this->sonarIntensityList[i].intensities.size(); j++) {
                currentAverage += this->sonarIntensityList[i].intensities[j];
            }
            currentAverage = currentAverage / (double) this->sonarIntensityList[i].intensities.size();
            averageIntensity += currentAverage;
        }
        averageIntensity = averageIntensity / (double) this->sonarIntensityList.size();

        double threashHoldIntensity = averageIntensity * 4;
        pcl::PointCloud<pcl::PointXYZ> returnCloud;
        for (int i = 0; i < this->sonarIntensityList.size(); i++) {
            for (int j = 0; j < this->sonarIntensityList[i].intensities.size(); j++) {
                if (this->sonarIntensityList[i].intensities[j] > threashHoldIntensity && j > 4) {
                    //calculate position of the point in xy coordinates
                    double distanceFromRobot = (double) j * this->sonarIntensityList[i].range /
                                               this->sonarIntensityList[i].number_of_samples;
                    pcl::PointXYZ tmpPoint(distanceFromRobot * cos(this->sonarIntensityList[i].angle / 400 * 2 * M_PI),
                                           distanceFromRobot * sin(this->sonarIntensityList[i].angle / 400 * 2 * M_PI),
                                           0);
                    returnCloud.push_back(tmpPoint);
                }
            }
        }
        return (returnCloud);
    }

    bool saveGraph(commonbluerovmsg::saveGraph::Request &req, commonbluerovmsg::saveGraph::Response &res) {
        std::cout << "test for testing" << std::endl;
        this->saveGraphStructure = true;
        this->saveStringGraph = req.saveString;
        res.saved = true;
        return true;
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekfwithros");
    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_);

    double cellSize = 0.25;


    ros::spin();

    return (0);
}
