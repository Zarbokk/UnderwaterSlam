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
#include "gazebo_msgs/ModelStates.h"

std::string DataFolder = "/home/tim-linux/dataFolder/gazeboCorrectedEvenAnglesPCLs_2_75/";

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_,bool scanDirectionReversed = false) : graphSaved(3),
                                      scanRegistrationObject(),
                                      occupancyMap(256, 128, 70, hilbertMap::HINGED_FEATURES) {

//        lastUpdateEkf = currentEkf.copyEKF();
//        subscriberIMU = n_.subscribe("mavros/imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        subscriberDVL = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
//        subscriberDepth = n_.subscribe("mavros/altitude_frd", 1000, &rosClassEKF::depthCallback, this);
        subscriberIntensitySonar = n_.subscribe("sonar/intensity", 1000, &rosClassEKF::scanCallback, this);
        this->serviceSaveGraph = n_.advertiseService("saveGraphOfSLAM", &rosClassEKF::saveGraph, this);
        this->subscriberGroundTruth = n_.subscribe("gazebo/model_states", 1000, &rosClassEKF::callbackGT, this);

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
        publisherOccupancyMap = n_.advertise<nav_msgs::OccupancyGrid>("occupancyHilbertMap", 10);

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
        this->occupancyMap.createRandomMap();
        this->scanDirectionReversed = scanDirectionReversed;

//        hilbertMap mapRepresentation(256,128,
//                                     60,hilbertMap::HINGED_FEATURES);


    }


private:

    ros::Subscriber subscriberDVL, subscriberIntensitySonar, subscriberGroundTruth;
    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
    ros::ServiceServer serviceSaveGraph;
    std::mutex stateEstimationMutex;
    std::mutex graphSlamMutex;
    std::mutex groundTruthMutex;
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
    Eigen::Vector3d currentGTPos;
    Eigen::Quaterniond currentGTRot;

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
    double beginningAngleOfRotation;
    double lastAngle;
    double startTimeOfCorrection;
    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstScan, saveGraphStructure;
    std::string saveStringGraph;
    int numberOfScan;
    hilbertMap occupancyMap;
    bool scanDirectionReversed;

    void slamCallback(const geometry_msgs::PoseStamped &msg) {
        Eigen::Quaterniond tmpRot;
        tmpRot.x() = msg.pose.orientation.x;
        tmpRot.y() = msg.pose.orientation.y;
        tmpRot.z() = msg.pose.orientation.z;
        tmpRot.w() = msg.pose.orientation.w;

        Eigen::Vector3d orientationRPY = generalHelpfulTools::getRollPitchYaw(tmpRot);
    }

    void scanCallback(const ping360_sonar::SonarEcho::ConstPtr &msg) {


        //std::cout << "Scan comes in: "<< std::endl;
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
        if (this->startTimeOfCorrection == 0) {
            this->lastAngle = msg->angle/400.0*M_PI*2.0;
            this->beginningAngleOfRotation = msg->angle/400.0*M_PI*2.0;
            this->startTimeOfCorrection = msg->header.stamp.toSec();
            this->graphSaved.getVertexByIndex(0)->setTimeStamp(msg->header.stamp.toSec());
        }

        if (this->timeVector.empty()) {
            return;
        }


        bool shouldTheScanBeAdded;
        if(scanDirectionReversed){
            //negative rotation
            shouldTheScanBeAdded=this->lastAngle < msg->angle/400.0*M_PI*2.0;
        }else{
            //positive rotation
            shouldTheScanBeAdded = this->lastAngle > msg->angle/400.0*M_PI*2.0;
        }
        if (shouldTheScanBeAdded) {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
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
            *this->currentScan = createPointCloudFromIntensitiesMatlab(10);


            this->graphSaved.getVertexList()->back().setPointCloudRaw(this->currentScan);
            double endAngle;
            if(scanDirectionReversed){
                endAngle = 0;
            }else{
                endAngle = 2*M_PI;
            }
            //correct the scan depending on the EKF callback
            slamToolsRos::correctPointCloudAtPos(this->graphSaved.getVertexList()->back().getVertexNumber(),
                                                 this->graphSaved, this->beginningAngleOfRotation, endAngle, this->scanDirectionReversed,
                                                 Eigen::Matrix4d::Identity());

            pcl::io::savePCDFileASCII( DataFolder + "pclKeyFrame" + std::to_string(this->numberOfScan) + ".pcd",
                                      *this->graphSaved.getVertexList()->back().getPointCloudCorrected());

            saveCurrentPose(this->numberOfScan);



            //find position of last pcl entry
            int positionLastPcl = 1;
            int sizeOfVertexList = this->graphSaved.getVertexList()->size();
            while (true) {
                if (this->graphSaved.getVertexList()->at(sizeOfVertexList - positionLastPcl - 1).getTypeOfVertex() ==
                    graphSlamSaveStructure::POINT_CLOUD_USAGE ||
                    this->graphSaved.getVertexList()->at(sizeOfVertexList - positionLastPcl - 1).getTypeOfVertex() ==
                    graphSlamSaveStructure::FIRST_ENTRY) {
                    break;
                }
                positionLastPcl++;
            }
            positionLastPcl++;


            if (this->firstScan) {
//                *this->previousScan = *this->graphSaved.getVertexList()->back().getPointCloudCorrected();
                if (numberOfScan > 0) {
                    this->firstScan = false;
                }
                if(scanDirectionReversed){
                    this->beginningAngleOfRotation = 2*M_PI;
                }else{
                    this->beginningAngleOfRotation = 0;
                }
            } else {


                //make scan matching with last scan, only if not the first scan available
                this->initialGuessTransformation =
                        this->graphSaved.getVertexList()->at(this->graphSaved.getVertexList()->size() -
                                                         positionLastPcl).getTransformation().inverse() *
                        this->graphSaved.getVertexList()->back().getTransformation();


//                std::cout << std::atan2(this->initialGuessTransformation(1,0),this->initialGuessTransformation(0,0))*180/M_PI << std::endl;
//                std::cout << this->initialGuessTransformation << std::endl;
//                double cellSize = 0.5;
                std::cout<< "Transofrmation: "<< std::endl;
                std::cout<< this->graphSaved.getVertexList()->at(sizeOfVertexList - positionLastPcl).getTransformation() << std::endl;
                std::cout << " Last" << std::endl;
                std::cout << this->graphSaved.getVertexList()->back().getTransformation() << std::endl;
                std::cout << "interesting rotation: "<< std::endl;
                Eigen::Matrix4d printingResult4d = this->graphSaved.getVertexList()->at(sizeOfVertexList - positionLastPcl).getTransformation().inverse()*this->graphSaved.getVertexList()->back().getTransformation();
                std::cout << printingResult4d<< std::endl;
                double fitnessScoreX, fitnessScoreY;
                std::cout << "Initial Guess Angle: " << std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)) << std::endl;
                std::cout << "Initial Guess:" << std::endl;
                std::cout << this->initialGuessTransformation << std::endl;
                //we need inverse transformation
//                this->currentTransformation = scanRegistrationObject.sofftRegistration(
//                        *this->previousScan, *this->graphSaved.getVertexList()->back().getPointCloudCorrected(),
//                        fitnessScoreX, fitnessScoreY,
//                        -100,
//                        true).inverse();

                double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
                        std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)),
                        std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)));

//                std::cout << "Estimation Registration:" << std::endl;
//                std::cout << this->currentTransformation << std::endl;
//                std::cout << "current Fitness Score X: " << fitnessScoreX << std::endl;
//                std::cout << "current Fitness Score Y: " << fitnessScoreY << std::endl;






//                std::cout << this->currentTransformation.inverse() << std::endl;
////                this->currentTransformation = scanRegistrationClass::generalizedIcpRegistration(
////                        this->graphSaved.getVertexList()->back().getPointCloudCorrected(), this->previousScan, this->Final,
////                        this->fitnessScore,
////                        this->initialGuessTransformation);
//                //std::cout << "current Fitness Score: " << sqrt(this->fitnessScore) << std::endl;
//



                std::cout << "difference of angle after Registration: " << differenceAngleBeforeAfter << std::endl;

//                if (abs(differenceAngleBeforeAfter) < 10.0 / 180.0 * M_PI) {
//                    Eigen::Quaterniond qTMP(this->currentTransformation.block<3, 3>(0, 0));
//                    graphSaved.addEdge(this->graphSaved.getVertexList()->size() - positionLastPcl,
//                                       graphSaved.getVertexList()->size() - 1,
//                                       this->currentTransformation.block<3, 1>(0, 3), qTMP,
//                                       Eigen::Vector3d(fitnessScoreX, fitnessScoreY, 0),
//                                       0.1,
//                                       graphSlamSaveStructure::POINT_CLOUD_USAGE,
//                                       timeDiffScans * 0.1);//@TODO still not sure about size
//                } else {
//                    std::cout << "we just skipped that registration" << std::endl;
//                }



//                pcl::io::savePCDFileASCII("/home/tim-linux/dataFolder/savingRandomPCL/secondPCL.pcd", *this->previousScan);
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloudPlotOnly(
                        new pcl::PointCloud<pcl::PointXYZ>);
                *tmpCloudPlotOnly = *currentScan;
                if(scanDirectionReversed){
                    this->beginningAngleOfRotation = 2*M_PI;
                }else{
                    this->beginningAngleOfRotation = 0;
                }





                //pcl::transformPointCloud(*tmpCloudPlotOnly, *tmpCloudPlotOnly, transformationImu2PCL);
                slamToolsRos::debugPlotting(this->previousScan, this->Final, tmpCloudPlotOnly,
                                            this->graphSaved.getVertexList()->back().getPointCloudCorrected(),
                                            this->publisherLastPCL, this->publisherRegistrationPCL,
                                            this->publisherBeforeCorrection, this->publisherAfterCorrection);
                std::cout << "After debug" << std::endl;
//                slamToolsRos::detectLoopClosureSOFFT(this->graphSaved, this->sigmaScaling, timeDiffScans * 0.1,
//                                                     this->scanRegistrationObject);//was 1.0

//                slamToolsRos::detectLoopClosureIPC(this->graphSaved, this->sigmaScaling, 1.0, timeDiffScans * 0.1,
//                                                   this->scanRegistrationObject);//was 1.0
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
            this->numberOfScan++;
            *this->previousScan = *this->graphSaved.getVertexList()->back().getPointCloudCorrected();

            sonarIntensityList.clear();
            clearSavingsOfPoses(msg->header.stamp.toSec());
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference of complete slam operation = "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                      << "[ms]" << std::endl;

            // ###################################### Hilbert MAP EXPERIMENT ######################################
//            std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
//
//            std::cout << "calculate the Hilbert Map:" << std::endl;
//            if(this->graphSaved.getVertexList().size()>100){





//            }
//            std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();
//            std::cout << "Time difference of HilbertMapThings = "
//                      << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - begin1).count()
//                      << "[ms]" << std::endl;

        }


        this->lastAngle = msg->angle/400.0*M_PI*2.0;
        this->sonarIntensityList.push_back(*msg);

        geometry_msgs::PoseStamped newMsg;
        newMsg.header.stamp = msg->header.stamp;
        newMsg.pose.position.x = this->graphSaved.getVertexList()->back().getPositionVertex().x();
        newMsg.pose.position.y = this->graphSaved.getVertexList()->back().getPositionVertex().y();
        newMsg.pose.position.z = 0;

        newMsg.pose.orientation.x = this->graphSaved.getVertexList()->back().getRotationVertex().x();
        newMsg.pose.orientation.y = this->graphSaved.getVertexList()->back().getRotationVertex().y();
        newMsg.pose.orientation.z = this->graphSaved.getVertexList()->back().getRotationVertex().z();
        newMsg.pose.orientation.w = this->graphSaved.getVertexList()->back().getRotationVertex().w();

        this->slamCallback(newMsg);

        if (this->saveGraphStructure) {
            std::cout << "saving graph " << std::endl;
            this->graphSaved.saveGraphJson(this->saveStringGraph);
            this->saveGraphStructure = false;
        }

    }

    void stateEstimationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        //std::cout << "EKF ComesIn: "<< std::endl;
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
        //std::cout << startTimetoAdd << " : " << endTimeToAdd << std::endl;
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
            //std::cout << diffMatrix << std::endl;
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

    pcl::PointCloud<pcl::PointXYZ> createPointCloudFromIntensitiesMatlab(int ignoreFirstStepsOfIntensity = 10){
        struct intensityMeasurement {
            double timeStamp;
            double angle;//in rad
            double increment;
            int size;
            std::vector<double> intensities;
        };

        //stupid way to find pointcloud:
        std::vector<intensityMeasurement> intensityMeasurementsList;
        //convert to intensityMeasurement
        for (int i = 0; i < this->sonarIntensityList.size(); i++) {
            intensityMeasurement tmpMeasurement;
            tmpMeasurement.angle = this->sonarIntensityList[i].angle;
            tmpMeasurement.timeStamp = this->sonarIntensityList[i].header.stamp.toSec();
            tmpMeasurement.size = this->sonarIntensityList[i].intensities.size();
            tmpMeasurement.increment = (double)this->sonarIntensityList[i].range/(double)this->sonarIntensityList[i].intensities.size();

            std::vector<double> linearSubstract = slamToolsRos::linspace(0,30,this->sonarIntensityList[i].intensities.size());
            for (int j = 0 ; j<this->sonarIntensityList[i].intensities.size();j++){
                if(j<10){
                    tmpMeasurement.intensities.push_back(0);
                }else{
                    double tmpIntensity = this->sonarIntensityList[i].intensities[j]-linearSubstract[j];
                    if(tmpIntensity<0){
                        tmpIntensity = 0;
                    }
                    tmpMeasurement.intensities.push_back(tmpIntensity);
                }
            }
            intensityMeasurementsList.push_back(tmpMeasurement);
        }

        for(int i = 0 ; i< intensityMeasurementsList.size();i++){
            std::vector<double> emptyVector;
            generalHelpfulTools::smooth_curve(intensityMeasurementsList[i].intensities,emptyVector,3);
            intensityMeasurementsList[i].intensities = emptyVector;
            emptyVector.clear();

            for(int j = 0 ; j<intensityMeasurementsList[i].intensities.size();j++){
                if(intensityMeasurementsList[i].intensities[j]<25){
                    intensityMeasurementsList[i].intensities[j] = 0;
                }
            }
        }




        double threashHoldIntensity = 45;
        pcl::PointCloud<pcl::PointXYZ> returnCloud;
        for (int i = 0; i < intensityMeasurementsList.size(); i++) {
            for (int j = 0; j < intensityMeasurementsList[i].intensities.size(); j++) {
                if (intensityMeasurementsList[i].intensities[j] > threashHoldIntensity && j > ignoreFirstStepsOfIntensity) {
                    //calculate position of the point in xy coordinates
                    double distanceFromRobot = (double) (j+1) * intensityMeasurementsList[i].increment;
                    //std::cout << "angle: "<< intensityMeasurementsList[i].angle << std::endl;
                    if(!std::isfinite(distanceFromRobot)){
                        std::cout << "we got a problem" << std::endl;
                    }
                    pcl::PointXYZ tmpPoint(distanceFromRobot * cos(intensityMeasurementsList[i].angle / 400 * 2 * M_PI),
                                           distanceFromRobot * sin(intensityMeasurementsList[i].angle / 400 * 2 * M_PI),
                                           0);
                    returnCloud.push_back(tmpPoint);
                }
            }
        }
        return (returnCloud);
    }

    pcl::PointCloud<pcl::PointXYZ> createPointCloudFromIntensities(int ignoreFirstStepsOfIntensity = 4) {

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
                if (this->sonarIntensityList[i].intensities[j] > threashHoldIntensity && j > ignoreFirstStepsOfIntensity) {
                    //calculate position of the point in xy coordinates
                    double distanceFromRobot = (double) j * this->sonarIntensityList[i].range/this->sonarIntensityList[i].number_of_samples;
                    if(!std::isfinite(distanceFromRobot)){
                        std::cout << "we got a problem" << std::endl;
                    }
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

    bool getDatasetFromGraph(int numberOfPointsInDataset,std::vector<dataPointStruct> &dataSet) {
        std::lock_guard<std::mutex> lock(this->graphSlamMutex);
//        std::vector<dataPointStruct> dataSet;

        if(this->graphSaved.getVertexList()->size()<this->numberOfEdgesBetweenScans){
            return false;
        }
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> dis(0.0, 1.0);

        for (int i = 0; i < numberOfPointsInDataset; i++) {

            //get some random number indexPointCloud and Random Number of Point
            int indexPointCloud = (int) (dis(gen) *
                                         (double) (this->graphSaved.getVertexList()->size() - 1));

            int addingValueIndex = 1;
            if (this->graphSaved.getVertexList()->size() - indexPointCloud < 10 &&
                this->graphSaved.getVertexList()->size() > 50) {
                addingValueIndex = -1;
            }


            while (true) {
                indexPointCloud += addingValueIndex;


                if (this->graphSaved.getVertexList()->at(indexPointCloud).getTypeOfVertex() ==
                    graphSaved.POINT_CLOUD_USAGE) {
                    break;
                }
            }
            int indexOfPointInPointcloud = (int) (dis(gen) *
                                                  (double) this->graphSaved.getVertexList()->at(indexPointCloud).getPointCloudCorrected()->points.size());


            Eigen::Matrix4d transformationOfPointcloud = this->graphSaved.getVertexList()->at(indexPointCloud).getTransformation();


            Eigen::Vector4d pointPos(
                    this->graphSaved.getVertexList()->at(indexPointCloud).getPointCloudCorrected()->points[indexOfPointInPointcloud].x,
                    this->graphSaved.getVertexList()->at(indexPointCloud).getPointCloudCorrected()->points[indexOfPointInPointcloud].y,
                    0,
                    1);
            pointPos = transformationOfPointcloud * pointPos;
            dataPointStruct tmpDP;
            tmpDP.x = pointPos.x();
            tmpDP.y = pointPos.y();
            tmpDP.z = pointPos.z();
            tmpDP.occupancy = 1;
            dataSet.push_back(tmpDP);

            //create 3 additional point where occupancy = -1
            for (int j = 0; j < 1; j++) {
                Eigen::Vector4d pointPosTwo(
                        this->graphSaved.getVertexList()->at(indexPointCloud).getPointCloudCorrected()->points[indexOfPointInPointcloud].x,
                        this->graphSaved.getVertexList()->at(indexPointCloud).getPointCloudCorrected()->points[indexOfPointInPointcloud].y,
                        0,
                        1);


                //double randomNumber = dis(gen);// should be between 0 and 1
                pointPosTwo = transformationOfPointcloud * dis(gen) * pointPosTwo;
                tmpDP.x = pointPosTwo.x();
                tmpDP.y = pointPosTwo.y();
                tmpDP.z = pointPosTwo.z();
                tmpDP.occupancy = -1;
                dataSet.push_back(tmpDP);
            }
        }
        return true;
    }

    void callbackGT(const gazebo_msgs::ModelStates::ConstPtr &msg){

        int indexOfBlueROV=0;
        for(int i = 0 ; i<msg->name.size();i++){
            if(msg->name.at(i)=="uuv_bluerov2_heavy"){
                indexOfBlueROV = i;
                break;
            }
        }
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
//        Eigen::Vector3d currentGTPos;
//        Eigen::Quaterniond currentGTRot;
        this->currentGTPos.x()= msg->pose.at(indexOfBlueROV).position.x;
        this->currentGTPos.y()= msg->pose.at(indexOfBlueROV).position.y;
        this->currentGTPos.z()= msg->pose.at(indexOfBlueROV).position.z;

        this->currentGTRot.x()= msg->pose.at(indexOfBlueROV).orientation.x;
        this->currentGTRot.y()= msg->pose.at(indexOfBlueROV).orientation.y;
        this->currentGTRot.z()= msg->pose.at(indexOfBlueROV).orientation.z;
        this->currentGTRot.w()= msg->pose.at(indexOfBlueROV).orientation.w;


    }

    void saveCurrentPose(int currentNumberOfScan){
        std::lock_guard<std::mutex> lock(this->groundTruthMutex);
        std::ofstream myFile1;
        myFile1.open(DataFolder + "position" + std::to_string(this->numberOfScan) + ".csv");
        myFile1 << this->currentGTPos.x(); //
        myFile1 << "\n";
        myFile1 << this->currentGTPos.y(); //
        myFile1 << "\n";
        myFile1 << this->currentGTPos.z(); //
        myFile1 << "\n";

        Eigen::Matrix3d rotationTMP(currentGTRot);
        //std::cout << "our first rotation" << std::endl;
        //std::cout << rotationTMP << std::endl;
        for(int i = 0 ; i<9;i++){
            myFile1 << rotationTMP.data()[i]; //
            myFile1 << "\n";
        }
        myFile1.close();
    }

public:
    void updateHilbertMap(){
        int numberOfPointsDataset = 5000;
        std::cout << "started Hilbert Shift:" << std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::vector<dataPointStruct> dataSet;
        bool foundDataset =  this->getDatasetFromGraph(numberOfPointsDataset,dataSet);
        //return if dataset cannot be found
        if(!foundDataset){
            return;
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time it takes to get the dataset = "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
                  << "[ms]" << std::endl;

        std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
        this->occupancyMap.trainClassifier(dataSet, (int)(numberOfPointsDataset*1.8));
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
    rosClassEKF rosClassForTests(n_,false);


//    ros::spin();


    ros::Rate loop_rate(0.1);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Duration(15).sleep();

    while (ros::ok()) {
//        ros::spinOnce();

//        rosClassForTests.updateHilbertMap();
        loop_rate.sleep();

        //std::cout << ros::Time::now() << std::endl;
    }


    return (0);
}
