//
// Created by tim-external on 22.06.22.
// This should be a file, which reads the intensity Values by ROS and then calculates the matching.


#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
//#include "ping360_sonar/SonarEcho.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "commonbluerovmsg/staterobotforevaluation.h"
#include "../slamTools/generalHelpfulTools.h"
#include "../slamTools/slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include <hilbertMap.h>
#include <opencv2/imgcodecs.hpp>
#include "std_srvs/SetBool.h"

#define NUMBER_OF_POINTS_DIMENSION 128
#define DIMENSION_OF_VOXEL_DATA 60
#define NUMBER_OF_POINTS_MAP 512.0
#define DIMENSION_OF_MAP 200.0


struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
};

struct groundTruthPositionStruct {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};


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

        subscriberEKF = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(1).sleep();
        subscriberIntensitySonar = n_.subscribe("sonar/intensity", 1000, &rosClassEKF::scanCallback, this);
        subscriberGroundTruth = n_.subscribe("/positionGT", 1000, &rosClassEKF::groundTruthCallback, this);
        pauseRosbag = n_.serviceClient<std_srvs::SetBoolRequest>(nameOfTheServiceCall);

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


    ros::Subscriber subscriberEKF, subscriberIntensitySonar, subscriberGroundTruth;
    ros::Publisher publisherPoseSLAM, publisherOccupancyMap;
    ros::ServiceServer serviceSaveGraph;
    ros::ServiceClient pauseRosbag;
    std::mutex stateEstimationMutex;
    std::mutex intensityMutex;
    std::mutex graphSlamMutex;
    std::mutex GTMutex;
    //GraphSlam things
    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;

    //PCL
    //std::vector<ping360_sonar::SonarEcho> sonarIntensityList;
    //Matrices:
    Eigen::Matrix4d currentTransformation;
    Eigen::Matrix4d initialGuessTransformation;
    Eigen::Matrix4d transformationX180Degree;


    groundTruthPositionStruct currentGTPosition;
    groundTruthPositionStruct lastGTPosition;
    //EKF savings
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
    std::deque<Eigen::Quaterniond> rotationVector;

    int indexLastFullScan;

    double fitnessScore;
    double sigmaScaling;


    graphSlamSaveStructure graphSaved;
    scanRegistrationClass scanRegistrationObject;
    bool firstSonarInput, firstCompleteSonarScan;


    void scanCallback(const commonbluerovmsg::SonarEcho2::ConstPtr &msg) {

        std::lock_guard<std::mutex> lock2(this->GTMutex);
        std::lock_guard<std::mutex> lock1(this->graphSlamMutex);
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
                                 10);


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


            groundTruthPositionStruct currentGTlocalPosition = this->currentGTPosition;
            this->graphSaved.getVertexList()->back().setTypeOfVertex(INTENSITY_SAVED_AND_KEYFRAME);

            if (firstCompleteSonarScan) {
                this->lastGTPosition = currentGTlocalPosition;
                firstCompleteSonarScan = false;
                return;
            }


            std_srvs::SetBool srv;
            srv.request.data = true;
            pauseRosbag.call(srv);


            //angleDiff = angleBetweenLastKeyframeAndNow(false);
            indexOfLastKeyframe = getLastIntensityKeyframe();
            // also get ground trouth transformation

            //match these voxels together
            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->back().getTransformation();
//            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
//                                                  this->initialGuessTransformation(0, 0));

/////////////////////////////////////////////////////// start of Registration ////////////////////////////////////////////////////

            pcl::PointCloud<pcl::PointXYZ>::Ptr scan1(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

            Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(M_PI, 0, 0);
//            *scan1 = createPCLFromGraphOneValue(indexOfLastKeyframe, tmpMatrix);
//            *scan2 = createPCLFromGraphOneValue(this->graphSaved.getVertexList()->size() - 1, tmpMatrix);
            *scan1 = createPCLFromGraphOnlyThreshold(indexOfLastKeyframe, tmpMatrix);
            *scan2 = createPCLFromGraphOnlyThreshold(this->graphSaved.getVertexList()->size() - 1, tmpMatrix);

            this->initialGuessTransformation.block<3, 1>(0, 3) =
                    this->initialGuessTransformation.block<3, 1>(0, 3) + Eigen::Vector3d(0, -0, 0);

//            this->initialGuessTransformation(0, 1) = -this->initialGuessTransformation(0, 1);
//            this->initialGuessTransformation(1, 0) = -this->initialGuessTransformation(1, 0);
//            this->initialGuessTransformation(1, 3) = -this->initialGuessTransformation(1, 3);
            double fitnessScoreX, fitnessScoreY;
//            this->currentTransformation = this->scanRegistrationObject.generalizedIcpRegistration(scan2, scan1, final,
//                                                                                                  fitnessScoreX,
//                                                                                                  initialGuessTransformation);


            Eigen::Matrix4d tmpMatruc = Eigen::Matrix4d::Identity();

//            this->currentTransformation = this->scanRegistrationObject.normalDistributionsTransformRegistration(scan2,
//                                                                                                                scan1,
//                                                                                                                final,
//                                                                                                                fitnessScoreX,
//                                                                                                                initialGuessTransformation,
//                                                                                                                1.0, 0.1,
//                                                                                                                0.01);
//            this->currentTransformation = this->scanRegistrationObject.RANSACRegistration(scan2, scan1,
//                                                                                          initialGuessTransformation,
//                                                                                          false, false);

//                        this->currentTransformation = this->scanRegistrationObject.super4PCSRegistration(scan1, scan2,
//                                                                                             this->initialGuessTransformation,
//                                                                                             true, false);

            this->currentTransformation = this->scanRegistrationObject.ndt_d2d_2d(scan2, scan1,
                                                                                  initialGuessTransformation, true);

            this->currentTransformation = this->scanRegistrationObject.ndt_p2d(scan2, scan1,
                                                                                  initialGuessTransformation, true);
//            this->initialGuessTransformation(0, 1) = -this->initialGuessTransformation(0, 1);
//            this->initialGuessTransformation(1, 0) = -this->initialGuessTransformation(1, 0);
//            this->initialGuessTransformation(1, 3) = -this->initialGuessTransformation(1, 3);
            pcl::io::savePLYFileBinary("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",
                                       *scan1);
            pcl::io::savePLYFileBinary("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",
                                       *scan2);
//            pcl::io::savePLYFileBinary("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/final.ply",
//                                       *final);

            // create a registration of two scans.




            // transform from 1 to 2
//            this->currentTransformation = this->registrationOfTwoVoxelsFMSOld(indexOfLastKeyframe,
//                                                                             this->graphSaved.getVertexList()->size() -
//                                                                             1,
//                                                                             fitnessScoreX,
//                                                                             fitnessScoreY,
//                                                                             this->initialGuessTransformation, true,
//                                                                             true,
//                                                                             true);
//            this->currentTransformation = this->registrationOfTwoVoxelsSOFFT(indexOfLastKeyframe,
//                                                                             this->graphSaved.getVertexList()->size() -
//                                                                             1,
//                                                                             fitnessScoreX,
//                                                                             fitnessScoreY,
//                                                                             this->initialGuessTransformation, true,
//                                                                             true,
//                                                                             true);
/////////////////////////////////////////////////////// end of Registration ////////////////////////////////////////////////////






//            double differenceAngleBeforeAfter = generalHelpfulTools::angleDiff(
//                    std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)),
//                    std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)));
            Eigen::Matrix4d GTTransformation =
                    gtToTranformation(this->lastGTPosition).inverse() * gtToTranformation(currentGTlocalPosition);

            //maybe print ground truth vs registration difference
//            std::cout << "current Position:" << std::endl;
//            std::cout << gtToTranformation(currentGTlocalPosition) << std::endl;
//            std::cout << "last Position:" << std::endl;
//            std::cout << gtToTranformation(this->lastGTPosition) << std::endl;
            std::cout << "everything in NED:" << std::endl;
            std::cout << "GT difference:" << std::endl;
            std::cout << GTTransformation << std::endl;
            std::cout << "Transformation Estimated:" << std::endl;
            std::cout << this->currentTransformation << std::endl;
            std::cout << "Initial Guess:" << std::endl;
            std::cout << this->initialGuessTransformation << std::endl;
//            //std::cout << "error:" << std::endl;
//            std::cout << std::fixed;
//            std::cout << std::setprecision(4);
            std::cout << "Error gt vs Registration: "
                      << (GTTransformation.block<3, 1>(0, 3) - this->currentTransformation.block<3, 1>(0, 3)).norm();
            std::cout << " Error gt vs Feed Forward: " << (GTTransformation.block<3, 1>(0, 3) -
                                                           this->initialGuessTransformation.block<3, 1>(0, 3)).norm()
                      << std::endl;
            std::cout << "Angle difference GT and registration: " << generalHelpfulTools::angleDiff(
                    std::atan2(GTTransformation(1, 0), GTTransformation(0, 0)),
                    std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)));
            std::cout << " Angle difference GT and initial Guess: " << generalHelpfulTools::angleDiff(
                    std::atan2(GTTransformation(1, 0), GTTransformation(0, 0)),
                    std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)))
                      << std::endl;
//            std::cout << "GT Angle: " << std::atan2(GTTransformation(1, 0), GTTransformation(0, 0)) << std::endl;
//            std::cout << "initialGuessTransformation Angle: " << std::atan2(this->initialGuessTransformation(1, 0), this->initialGuessTransformation(0, 0)) << std::endl;
//            std::cout << "reg Angle: " << std::atan2(this->currentTransformation(1, 0), this->currentTransformation(0, 0)) << std::endl;

            this->lastGTPosition = currentGTlocalPosition;
            srv.request.data = false;
            pauseRosbag.call(srv);
//            std::cout << "next: " << std::endl;
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

    Eigen::Matrix4d registrationOfTwoVoxelsSOFFT(int indexVoxel1,
                                                 int indexVoxel2,
                                                 double &fitnessX, double &fitnessY, Eigen::Matrix4d initialGuess,
                                                 bool useInitialAngle, bool useInitialTranslation,
                                                 bool debug = false) {
        double goodGuessAlpha = -100;
        if (useInitialAngle) {
            goodGuessAlpha = std::atan2(initialGuess(1, 0),
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
            cv::imwrite("/home/tim-external/Documents/imreg_fmt/firstImage.jpg", magTMP1);

            cv::Mat magTMP2(NUMBER_OF_POINTS_DIMENSION, NUMBER_OF_POINTS_DIMENSION, CV_64F, voxelData2);
            //add gaussian blur
            cv::GaussianBlur(magTMP2, magTMP2, cv::Size(9, 9), 0);
            cv::imwrite("/home/tim-external/Documents/imreg_fmt/secondImage.jpg", magTMP2);
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
                                                                                                       initialGuessTransformation.block<3, 1>(
                                                                                                               0, 3),
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

    void groundTruthCallback(const commonbluerovmsg::staterobotforevaluation::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->GTMutex);
        this->currentGTPosition.x = msg->xPosition;
        this->currentGTPosition.y = msg->yPosition;
        this->currentGTPosition.z = msg->zPosition;
        this->currentGTPosition.roll = msg->roll;
        this->currentGTPosition.pitch = msg->pitch;
        this->currentGTPosition.yaw = msg->yaw;

    }

    Eigen::Matrix4d gtToTranformation(groundTruthPositionStruct input) {
        Eigen::Matrix4d output;
        output = generalHelpfulTools::getTransformationMatrixFromRPY(input.roll, input.pitch, input.yaw);

        output(0, 3) = input.x;
        output(1, 3) = input.y;
        output(2, 3) = input.z;

        return output;
    }


    pcl::PointCloud<pcl::PointXYZ> createPCLFromGraphOneValue(int indexStart,
                                                              Eigen::Matrix4d transformationInTheEndOfCalculation) {
        pcl::PointCloud<pcl::PointXYZ> scan;
        //create array with all intencities.
        // Calculate maximum of intensities.
        // only use maximum of 10% of max value as points

        double maximumIntensity = 0;
        int i = 0;
        do {
            for (int j = 0;
                 j < this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                    maximumIntensity) {
                    maximumIntensity = this->graphSaved.getVertexList()->at(
                            indexStart - i).getIntensities().intensities[j];
                }
            }
            i++;
        } while (this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);

        double thresholdIntensityScan = maximumIntensity * 0.3;//maximum intensity of 0.9



        i = 0;
        do {
            //find max Position
            int maxPosition = 0;
            for (int j = 5;
                 j < this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                    this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition]) {
                    maxPosition = j;
                }
            }
            if (maxPosition > 5 &&
                this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition] >
                thresholdIntensityScan) {
                Eigen::Matrix4d transformationOfIntensityRay =
                        this->graphSaved.getVertexList()->at(indexStart).getTransformation().inverse() *
                        this->graphSaved.getVertexList()->at(indexStart - i).getTransformation();

                //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
                Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                                 this->graphSaved.getVertexList()->at(
                                                                                                                         indexStart -
                                                                                                                         i).getIntensities().angle);

                double distanceOfIntensity =
                        maxPosition / ((double) this->graphSaved.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().range);
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);

                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //create point for PCL
                pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                       (float) positionOfIntensity[1],
                                       (float) positionOfIntensity[2]);
                scan.push_back(tmpPoint);
            }


            i++;
        } while (this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);
        return scan;
    }

    Eigen::Matrix4d registrationOfTwoVoxelsFMSOld(int indexVoxel1,
                                                  int indexVoxel2,
                                                  double &fitnessX, double &fitnessY, Eigen::Matrix4d initialGuess,
                                                  bool useInitialAngle, bool useInitialTranslation,
                                                  bool debug = false) {
        //we always assume without initial guess for now.

        double goodGuessAlpha = -100;
        if (useInitialAngle) {
            goodGuessAlpha = std::atan2(initialGuess(1, 0),
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


        if (debug) {
            std::ofstream myFile3, myFile6;

            myFile3.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW1.csv");
            myFile6.open("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW2.csv");
            for (int i = 0; i < NUMBER_OF_POINTS_DIMENSION; i++) {
                for (int j = 0; j < NUMBER_OF_POINTS_DIMENSION; j++) {
                    myFile3 << voxelData1[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
                    myFile3 << "\n";
                    myFile6 << voxelData2[j + NUMBER_OF_POINTS_DIMENSION * i]; // imaginary part
                    myFile6 << "\n";
                }
            }

            myFile3.close();
            myFile6.close();
        }

        Eigen::Matrix4d estimatedTransformationScans = this->scanRegistrationObject.FMSRegistrationOld(voxelData1,
                                                                                                       voxelData2,
                                                                                                       (double) DIMENSION_OF_VOXEL_DATA /
                                                                                                       (double) NUMBER_OF_POINTS_DIMENSION,
                                                                                                       debug);


        return estimatedTransformationScans;//should be the transformation matrix from 1 to 2
    }

    pcl::PointCloud<pcl::PointXYZ> createPCLFromGraphOnlyThreshold(int indexStart,
                                                                   Eigen::Matrix4d transformationInTheEndOfCalculation) {
        pcl::PointCloud<pcl::PointXYZ> scan;
        //create array with all intencities.
        // Calculate maximum of intensities.


        double maximumIntensity = 0;
        int i = 0;
        do {
            for (int j = 0;
                 j < this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                    maximumIntensity) {
                    maximumIntensity = this->graphSaved.getVertexList()->at(
                            indexStart - i).getIntensities().intensities[j];
                }
            }
            i++;
        } while (this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);

        double thresholdIntensityScan = maximumIntensity * 0.3;//maximum intensity of 0.9



        i = 0;
        do {
            Eigen::Matrix4d transformationOfIntensityRay =
                    this->graphSaved.getVertexList()->at(indexStart).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->at(indexStart - i).getTransformation();

            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             this->graphSaved.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);
            for (int j = 5;
                 j < this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (this->graphSaved.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                    thresholdIntensityScan) {
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
                    //create point for PCL
                    pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                           (float) positionOfIntensity[1],
                                           (float) positionOfIntensity[2]);
                    scan.push_back(tmpPoint);
                }
            }
            i++;
        } while (this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 this->graphSaved.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);
        return scan;
    }

};


bool getNodes(ros::V_string &nodes) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
        return false;
    }

    ros::S_string node_set;
    for (int i = 0; i < payload.size(); ++i) {
        for (int j = 0; j < payload[i].size(); ++j) {
            XmlRpc::XmlRpcValue val = payload[i][j][1];
            for (int k = 0; k < val.size(); ++k) {
                std::string name = payload[i][j][1][k];
                node_set.insert(name);
            }
        }
    }

    nodes.insert(nodes.end(), node_set.begin(), node_set.end());

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekfwithros");
    ros::NodeHandle n;
//
    ros::V_string nodes;


    getNodes(nodes);


    int i;
    for (i = 0; i < nodes.size(); i++) {

        if (nodes[i].substr(1, 4) == "play") {
//            std::cout << "we found it" << std::endl;
            break;
        }
    }
//    std::cout << nodes[i]+"/pause_playback" << std::endl;
    ros::ServiceServer serviceResetEkf;


    if (ros::service::exists(nodes[i] + "/pause_playback", true)) {

    } else {
        exit(-1);
    }


    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_, nodes[i] + "/pause_playback");


//    ros::spin();


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


