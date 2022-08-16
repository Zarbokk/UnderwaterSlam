//
// Created by tim-external on 27.07.22.
//

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
//#include "ping360_sonar/SonarEcho.h"
#include "commonbluerovmsg/SonarEcho2.h"
#include "commonbluerovmsg/staterobotforevaluation.h"
#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include "commonbluerovmsg/saveGraph.h"
#include <hilbertMap.h>
#include <opencv2/imgcodecs.hpp>
#include "std_srvs/SetBool.h"
#include <filesystem>

//#define numberOfPoints 128
#define DIMENSION_OF_VOXEL_DATA 40
#define NUMBER_OF_POINTS_MAP 1024.0
#define DIMENSION_OF_MAP 200.0
#define FACTOR_OF_THRESHOLD 0.3
#define IGNORE_DISTANCE_TO_ROBOT 1.5

#define SHOULD_USE_ROSBAG true


#define SHIFT_VALUE_ANGLE 10.0
#define SHIFT_VALUE_POSITION 5.0
#define NUMBER_OF_CHANGED_SCANS 10.0
#define ONLY_ANGLE_CHANGED false

#define ADD_NOISE_TO_SCANS true


#define HOME_LOCATION "/home/tim-external/dataFolder/ValentinBunkerData/"
#define WHICH_FOLDER_SHOULD_BE_SAVED "randomTests/"


//#define HOME_LOCATION "/home/tim-external/dataFolder/StPereDataset/"
//#define WHICH_FOLDER_SHOULD_BE_SAVED "randomShifts105/"





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


//occupancyMap(256, numberOfPoints, 70, hilbertMap::HINGED_FEATURES)
class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_, const std::string &nameOfTheServiceCall) : graphSaved(3, INTENSITY_BASED_GRAPH) {

        this->numberOfScan = 0;

        subscriberEKF = n_.subscribe("publisherPoseEkf", 1000, &rosClassEKF::stateEstimationCallback, this);
        ros::Duration(1).sleep();
        subscriberIntensitySonar = n_.subscribe("sonar/intensity", 1000, &rosClassEKF::scanCallback, this);

        subscriberGroundTruth = n_.subscribe("/magnetic_heading", 1000, &rosClassEKF::groundTruthCallbackStPere, this);
//        subscriberGroundTruth = n_.subscribe("/positionGT", 1000, &rosClassEKF::groundTruthCallbackGazebo, this);


        if (SHOULD_USE_ROSBAG) {
            pauseRosbag = n_.serviceClient<std_srvs::SetBoolRequest>(nameOfTheServiceCall);
        }

        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
        transformationX180Degree(3, 3) = 1;

        graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                             Eigen::Vector3d(0, 0, 0), 0, ros::Time::now().toSec(),
                             FIRST_ENTRY);


        this->sigmaScaling = 0.2;


        this->firstSonarInput = true;
        this->firstCompleteSonarScan = true;

        std::filesystem::create_directory(std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED));

    }


private:
    int numberOfScan;


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

    std::ofstream comparisonFile;


    graphSlamSaveStructure graphSaved;
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
//        std::cout << "Time difference 4 = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
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

            if (SHOULD_USE_ROSBAG) {

                std_srvs::SetBool srv;
                srv.request.data = true;
                pauseRosbag.call(srv);
            }

            //angleDiff = angleBetweenLastKeyframeAndNow(false);
            indexOfLastKeyframe = getLastIntensityKeyframe();
            // also get ground trouth transformation

            //match these voxels together
            this->initialGuessTransformation =
                    this->graphSaved.getVertexList()->at(indexOfLastKeyframe).getTransformation().inverse() *
                    this->graphSaved.getVertexList()->back().getTransformation();
//            double initialGuessAngle = std::atan2(this->initialGuessTransformation(1, 0),
//                                                  this->initialGuessTransformation(0, 0));


            std::string directoryOfInterest = std::string(HOME_LOCATION) + std::string(WHICH_FOLDER_SHOULD_BE_SAVED) +
                                              std::string("scanNumber_") + std::to_string(numberOfScan) +
                                              std::string("/");
            std::filesystem::create_directory(directoryOfInterest);


            std::ofstream outVoxelSize(directoryOfInterest + "dimensionVoxelData.csv");
            outVoxelSize << DIMENSION_OF_VOXEL_DATA;
            outVoxelSize << '\n';
            outVoxelSize.close();


            pcl::PointCloud<pcl::PointXYZ> scan1Threshold;
            pcl::PointCloud<pcl::PointXYZ> scan1ThresholdShifted;

            pcl::PointCloud<pcl::PointXYZ> scan1OneValue;
            pcl::PointCloud<pcl::PointXYZ> scan1OneValueShifted;

            Eigen::Matrix4d tmpMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(M_PI, 0, 0);
            scan1OneValue = createPCLFromGraphOneValue(this->graphSaved.getVertexList()->size() - 1, tmpMatrix,this->graphSaved);
            scan1Threshold = createPCLFromGraphOnlyThreshold(this->graphSaved.getVertexList()->size() - 1, tmpMatrix,this->graphSaved);

            this->initialGuessTransformation.block<3, 1>(0, 3) =
                    this->initialGuessTransformation.block<3, 1>(0, 3) + Eigen::Vector3d(0, -0, 0);

            Eigen::Matrix4d GTTransformation =
                    gtToTranformation(this->lastGTPosition).inverse() * gtToTranformation(currentGTlocalPosition);

            std::random_device rd;  // Will be used to obtain a seed for the random number engine
            std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
            std::uniform_real_distribution<> dis(-1.0, 1.0);
            double angleScaling = SHIFT_VALUE_ANGLE / 180.0 * M_PI;

            double xScaling = SHIFT_VALUE_POSITION;
            double yScaling = SHIFT_VALUE_POSITION;

            for (int numberOfTransformation = 0;
                 numberOfTransformation < NUMBER_OF_CHANGED_SCANS; numberOfTransformation++) {
                //save already working binaries
                pcl::io::savePLYFileBinary(
                        directoryOfInterest + std::to_string(numberOfTransformation) + std::string("_OneValue.ply"),
                        scan1OneValue);

                pcl::io::savePLYFileBinary(
                        directoryOfInterest + std::to_string(numberOfTransformation) + std::string("_Threshold.ply"),
                        scan1Threshold);





                //create random tranformation;
                double randomYaw = dis(gen) * angleScaling;
                double randomX = dis(gen) * xScaling;
                double randomY = dis(gen) * yScaling;

                if (ONLY_ANGLE_CHANGED) {
                    randomYaw = SHIFT_VALUE_ANGLE / 180.0 * M_PI;
                }
                Eigen::Matrix4d randomTransformation = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                           randomYaw);
                randomTransformation(0, 3) = randomX;
                randomTransformation(1, 3) = randomY;

                std::ofstream outMatrix(
                        directoryOfInterest + std::to_string(numberOfTransformation) + std::string("_transformation") +
                        std::string(".csv"));

                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k < 4; k++)
                        outMatrix << randomTransformation(j, k) << ',';
                    outMatrix << '\n';
                }
                outMatrix.close();


                graphSlamSaveStructure randomNoiseGraph = graphSaved;


                if(ADD_NOISE_TO_SCANS){
                    //maybe add here NOISE or wrong angles complete wrong other things
                    randomNoiseGraph.addRandomNoiseToGraph(0.1,0.005);
                }

//                std::cout << randomTransformation << std::endl;

                scan1OneValueShifted = createPCLFromGraphOneValue(this->graphSaved.getVertexList()->size() - 1, tmpMatrix,randomNoiseGraph);
                scan1ThresholdShifted = createPCLFromGraphOnlyThreshold(this->graphSaved.getVertexList()->size() - 1, tmpMatrix,randomNoiseGraph);



                pcl::transformPointCloud(scan1OneValueShifted, scan1OneValueShifted, randomTransformation);
                pcl::transformPointCloud(scan1ThresholdShifted, scan1ThresholdShifted, randomTransformation);


                pcl::io::savePLYFileBinary(directoryOfInterest + std::to_string(numberOfTransformation) +
                                           std::string("_ThresholdShifted.ply"),
                                           scan1ThresholdShifted);
                pcl::io::savePLYFileBinary(directoryOfInterest + std::to_string(numberOfTransformation) +
                                           std::string("_OneValueShifted.ply"),
                                           scan1OneValueShifted);




                for (int numberOfPoints = 32;
                     numberOfPoints <= 256; numberOfPoints = numberOfPoints * 2) {//save for different points

                    double *voxelData;
                    voxelData = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
                    double *voxelDataShifted;
                    voxelDataShifted = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
                    //still missing


                    //add noise here

                    std::ofstream out(
                            directoryOfInterest + std::to_string(numberOfTransformation) + std::string("intensity") +
                            std::string(std::to_string(numberOfPoints)) + std::string(".csv"));

                    double maximumVoxel2 = createVoxelOfGraph(voxelData,
                                                              this->graphSaved.getVertexList()->size() - 1,
                                                              Eigen::Matrix4d::Identity(),
                                                              numberOfPoints,this->graphSaved);//get voxel
                    for (int j = 0; j < numberOfPoints; j++) {
                        for (int k = 0; k < numberOfPoints; k++)
                            out << voxelData[j + numberOfPoints * k] << ',';
                        out << '\n';
                    }
                    out.close();

                    std::ofstream outShifted(directoryOfInterest + std::to_string(numberOfTransformation) +
                                             std::string("intensityShifted") +
                                             std::string(std::to_string(numberOfPoints)) + std::string(".csv"));

                    double maximumVoxel1 = createVoxelOfGraph(voxelDataShifted,
                                                              this->graphSaved.getVertexList()->size() - 1,
                                                              randomTransformation,
                                                              numberOfPoints,randomNoiseGraph);//get voxel
                    for (int j = 0; j < numberOfPoints; j++) {
                        for (int k = 0; k < numberOfPoints; k++)
                            outShifted << voxelDataShifted[j + numberOfPoints * k] << ',';
                        outShifted << '\n';
                    }
                    outShifted.close();

                    std::free(voxelData);
                    std::free(voxelDataShifted);
                }
            }

            double *voxelData;
            voxelData = (double *) malloc(sizeof(double) * 256 * 256);

            double maximumVoxel2 = createVoxelOfGraph(voxelData,
                                                      this->graphSaved.getVertexList()->size() - 1,
                                                      Eigen::Matrix4d::Identity(),
                                                      256,this->graphSaved);//get voxel

            cv::Mat magTMP1(256, 256, CV_64F, voxelData);

            cv::imwrite(directoryOfInterest + "00_ForShow.jpg", magTMP1);

            free(voxelData);


            std::cout
                    << "########################################################## NEW SCAN ##########################################################"
                    << std::endl;
            this->lastGTPosition = currentGTlocalPosition;
            this->numberOfScan++;
            if (SHOULD_USE_ROSBAG) {

                std_srvs::SetBool srv;
                srv.request.data = false;
                pauseRosbag.call(srv);
            }


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

    double createVoxelOfGraph(double voxelData[], int indexStart, Eigen::Matrix4d transformationInTheEndOfCalculation,
                              int numberOfPoints, graphSlamSaveStructure &usedGraph) {
        int *voxelDataIndex;
        voxelDataIndex = (int *) malloc(sizeof(int) * numberOfPoints * numberOfPoints);
        //set zero voxel and index
        for (int i = 0; i < numberOfPoints * numberOfPoints; i++) {
            voxelDataIndex[i] = 0;
            voxelData[i] = 0;
        }


        int i = 0;
        do {
            //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.


            //get position of current intensityRay
            Eigen::Matrix4d transformationOfIntensityRay =
                    usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                            usedGraph.getVertexList()->at(indexStart - i).getTransformation();

            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             usedGraph.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);

            int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT /
                                        (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                         ((double) usedGraph.getVertexList()->at(
                                                 indexStart - i).getIntensities().intensities.size())));


            for (int j = ignoreDistance;
                 j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                double distanceOfIntensity =
                        j / ((double) usedGraph.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);

                int incrementOfScan = usedGraph.getVertexList()->at(indexStart - i).getIntensities().increment;
                for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                    Eigen::Vector4d positionOfIntensity(
                            distanceOfIntensity,
                            0,
                            0,
                            1);
                    double rotationOfPoint = l / 400.0;
                    Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                                rotationOfPoint);
                    positionOfIntensity = rotationForBetterView * positionOfIntensity;

                    positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                          rotationOfSonarAngleMatrix * positionOfIntensity;
                    //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                    int indexX =
                            (int) (positionOfIntensity.x() / (DIMENSION_OF_VOXEL_DATA / 2) * numberOfPoints /
                                   2) +
                            numberOfPoints / 2;
                    int indexY =
                            (int) (positionOfIntensity.y() / (DIMENSION_OF_VOXEL_DATA / 2) * numberOfPoints /
                                   2) +
                            numberOfPoints / 2;


                    if (indexX < numberOfPoints && indexY < numberOfPoints && indexY >= 0 &&
                        indexX >= 0) {
                        //                    std::cout << indexX << " " << indexY << std::endl;
                        //if index fits inside of our data, add that data. Else Ignore
                        voxelDataIndex[indexX + numberOfPoints * indexY] =
                                voxelDataIndex[indexX + numberOfPoints * indexY] + 1;
                        //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                        voxelData[indexX + numberOfPoints * indexY] =
                                voxelData[indexX + numberOfPoints * indexY] +
                                        usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j];
                        //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                        //                    std::cout << "random: " << std::endl;
                    }
                }
            }
            i++;
        } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);
        double maximumOfVoxelData = 0;
        for (i = 0; i < numberOfPoints * numberOfPoints; i++) {
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


    void groundTruthCallbackGazebo(const commonbluerovmsg::staterobotforevaluation::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->GTMutex);
        this->currentGTPosition.x = msg->xPosition;
        this->currentGTPosition.y = msg->yPosition;
        this->currentGTPosition.z = msg->zPosition;
        this->currentGTPosition.roll = msg->roll;
        this->currentGTPosition.pitch = msg->pitch;
        this->currentGTPosition.yaw = msg->yaw;

    }

    void groundTruthCallbackStPere(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->GTMutex);
        this->currentGTPosition.x = msg->vector.x;
        this->currentGTPosition.y = msg->vector.y;
        this->currentGTPosition.z = 0;
        this->currentGTPosition.roll = 0;
        this->currentGTPosition.pitch = 0;
        this->currentGTPosition.yaw = msg->vector.z;

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
                                                              Eigen::Matrix4d transformationInTheEndOfCalculation, graphSlamSaveStructure &usedGraph) {
        pcl::PointCloud<pcl::PointXYZ> scan;
        //create array with all intencities.
        // Calculate maximum of intensities.
        // only use maximum of 10% of max value as points
        int i = 0;
        double maximumIntensity = 0;

        int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT /
                                    (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                     ((double) usedGraph.getVertexList()->at(
                                             indexStart - i).getIntensities().intensities.size())));

        do {
            for (int j = ignoreDistance;
                 j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                    maximumIntensity) {
                    maximumIntensity = usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities[j];
                }
            }
            i++;
        } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);

        double thresholdIntensityScan = maximumIntensity * FACTOR_OF_THRESHOLD;//maximum intensity of 0.9



        i = 0;
        do {
            //find max Position
            int maxPosition = ignoreDistance;
            for (int j = ignoreDistance;
                 j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                        usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition]) {
                    maxPosition = j;
                }
            }
            if (maxPosition > ignoreDistance &&
                    usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition] >
                thresholdIntensityScan) {
                Eigen::Matrix4d transformationOfIntensityRay =
                        usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                        usedGraph.getVertexList()->at(indexStart - i).getTransformation();

                //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
                Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                                 usedGraph.getVertexList()->at(
                                                                                                                         indexStart -
                                                                                                                         i).getIntensities().angle);

                double distanceOfIntensity =
                        maxPosition / ((double) usedGraph.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
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
        } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);
        return scan;
    }


    pcl::PointCloud<pcl::PointXYZ> createPCLFromGraphOnlyThreshold(int indexStart,
                                                                   Eigen::Matrix4d transformationInTheEndOfCalculation, graphSlamSaveStructure &usedGraph) {
        pcl::PointCloud<pcl::PointXYZ> scan;
        //create array with all intencities.
        // Calculate maximum of intensities.


        double maximumIntensity = 0;
        int i = 0;

        int ignoreDistance = (int) (IGNORE_DISTANCE_TO_ROBOT /
                                    (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                     ((double) usedGraph.getVertexList()->at(
                                             indexStart - i).getIntensities().intensities.size())));


        do {
            for (int j = ignoreDistance;
                 j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                    maximumIntensity) {
                    maximumIntensity = usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities[j];
                }
            }
            i++;
        } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);

        double thresholdIntensityScan = maximumIntensity * FACTOR_OF_THRESHOLD;//maximum intensity of 0.9



        i = 0;
        do {
            Eigen::Matrix4d transformationOfIntensityRay =
                    usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                    usedGraph.getVertexList()->at(indexStart - i).getTransformation();

            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             usedGraph.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);
            for (int j = ignoreDistance;
                 j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
                if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                    thresholdIntensityScan) {
                    double distanceOfIntensity =
                            j / ((double) usedGraph.getVertexList()->at(
                                    indexStart - i).getIntensities().intensities.size()) *
                            ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
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
        } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
                 usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
                 INTENSITY_SAVED_AND_KEYFRAME);
        return scan;
    }

    std::vector<std::string> get_directories(const std::string &s) {
        std::vector<std::string> r;
        for (auto &p: std::filesystem::directory_iterator(s))
            if (p.is_directory())
                r.push_back(p.path().string());
        return r;
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
    std::cout << HOME_LOCATION << WHICH_FOLDER_SHOULD_BE_SAVED << std::endl;

    ros::init(argc, argv, "ekfwithros");
    ros::NodeHandle n;
//
    ros::V_string nodes;
    int i = 0;
    std::string stringForRosClass;
    if (SHOULD_USE_ROSBAG) {
        getNodes(nodes);


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

        stringForRosClass = nodes[i] + "/pause_playback";
    }

    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_, stringForRosClass);


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


