//
// Created by tim on 26.03.21.
//

#include <slamToolsRos.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include <iostream>
#include "ping360_sonar/SonarEcho.h"

struct intensityMeasurement {
    double timeStamp;
    double angle;//in rad
    double increment;
    int size;
    std::vector<int> intensities;
};


std::deque<measurement> parseCSVFileGT(std::istream &stream,double removeLinesUntil) {
    std::deque<measurement> returnVector;

    std::string firstLine;
    std::getline(stream, firstLine);

    //std::stringstream          lineStream(line);
    std::string cell;


    for (std::string line; std::getline(stream, line);) {
        std::stringstream lineStream(line);
        std::vector<std::string> result;
        while (std::getline(lineStream, cell, ',')) {
            result.push_back(cell);
            //std::cout << cell << std::endl;
        }
        if(removeLinesUntil<std::stod(result[0])){


        measurement tmpMeas{};
        tmpMeas.keyframe = -1;
        tmpMeas.x = std::stod(result[1]);
        tmpMeas.y = std::stod(result[2]);
        tmpMeas.z = 0;
        tmpMeas.timeStamp = std::stod(result[0]);
        returnVector.push_back(tmpMeas);
        }
    }
    return returnVector;
}

std::deque<ImuData> parseCSVFileIMU(std::istream &stream, Eigen::Matrix4d &tranformationMatrixIMU, double removeLinesUntil) {
    std::deque<ImuData> returnVector;

    std::string firstLine;
    std::getline(stream, firstLine);

    //std::stringstream          lineStream(line);
    std::string cell;


    for (std::string line; std::getline(stream, line);) {
        std::stringstream lineStream(line);
        std::vector<std::string> result;
        while (std::getline(lineStream, cell, ',')) {
            result.push_back(cell);
            //std::cout << cell << std::endl;
        }
        if(removeLinesUntil<std::stod(result[0])) {
            ImuData tmpMeas{};

            Eigen::Vector4d tmpPoint(std::stod(result[4]), std::stod(result[5]), std::stod(result[6]), 1);
            Eigen::Vector4d tmpPoint2(std::stod(result[7]), std::stod(result[8]), std::stod(result[9]), 1);
            tmpPoint = tranformationMatrixIMU * tmpPoint;
            tmpMeas.ax = tmpPoint2[0];
            tmpMeas.ay = tmpPoint2[1];
            tmpMeas.az = tmpPoint2[2];
            tmpMeas.wx = tmpPoint[0];
            tmpMeas.wy = tmpPoint[1];
            tmpMeas.wz = tmpPoint[2];
            tmpMeas.roll = std::stod(result[1]);
            tmpMeas.pitch = std::stod(result[2]);
            tmpMeas.timeStamp = std::stod(result[0]);
            returnVector.push_back(tmpMeas);
        }
    }
    return returnVector;
}

std::deque<DvlData> parseCSVFileDVL(std::istream &stream, Eigen::Matrix4d &tranformationMatrixDVL, double removeLinesUntil) {
    std::deque<DvlData> returnVector;

    std::string firstLine;
    std::getline(stream, firstLine);

    //std::stringstream          lineStream(line);
    std::string cell;


    for (std::string line; std::getline(stream, line);) {
        std::stringstream lineStream(line);
        std::vector<std::string> result;
        while (std::getline(lineStream, cell, ',')) {
            result.push_back(cell);
            //std::cout << cell << std::endl;
        }

        if(removeLinesUntil<std::stod(result[0])) {
            DvlData tmpMeas{};

            Eigen::Vector4d tmpPoint(std::stod(result[1]), std::stod(result[2]), std::stod(result[3]), 1);
            tmpPoint = tranformationMatrixDVL * tmpPoint;
            tmpMeas.vx = tmpPoint[0];
            tmpMeas.vy = tmpPoint[1];
            tmpMeas.vz = tmpPoint[2];
            tmpMeas.timeStamp = std::stod(result[0]);
            returnVector.push_back(tmpMeas);
        }
    }
    return returnVector;
}

std::deque<double> parseCSVFileKeyFramesTimeStamps(std::istream &stream, double removeLinesUntil) {
    std::deque<double> returnVector;

    std::string firstLine;
    std::getline(stream, firstLine);

    //std::stringstream          lineStream(line);
    std::string cell;


    for (std::string line; std::getline(stream, line);) {
        std::stringstream lineStream(line);
        std::vector<std::string> result;
        while (std::getline(lineStream, cell, ',')) {
            result.push_back(cell);
            //std::cout << cell << std::endl;
        }
        if(removeLinesUntil<std::stod(result[0])) {
            double tmpTime;
            tmpTime = std::stod(result[1]);
            returnVector.push_back(tmpTime);//@TODO the last line is still missing
        }
    }
    return returnVector;
}

std::deque<intensityMeasurement> parseCSVFileIntensityTimeStamps(std::istream &stream, double removeLinesUntil) {
    std::deque<intensityMeasurement> returnVector;

    //std::string firstLine;
    //std::getline(stream, firstLine);

    //std::stringstream          lineStream(line);
    std::string cell;


    for (std::string line; std::getline(stream, line);) {
        std::stringstream lineStream(line);
        std::vector<std::string> result;
        while (std::getline(lineStream, cell, ',')) {
            result.push_back(cell);
            //std::cout << cell << std::endl;
        }
        if(removeLinesUntil<std::stod(result[0])) {
            intensityMeasurement tmpIntensity{};

            tmpIntensity.timeStamp = std::stod(result[0]);
            tmpIntensity.angle = std::stod(result[1]);
            for (int i = 2; i < result.size(); i++) {
                tmpIntensity.intensities.push_back(std::stod(result[i]));
            }
            returnVector.push_back(tmpIntensity);
        }
    }
    return returnVector;
}

void loadCSVFiles(std::deque<measurement> &groundTruthSorted,
                  std::deque<ImuData> &ImuSorted,
                  std::deque<DvlData> &bodyVelocitySorted,
                  std::deque<intensityMeasurement> &intensitySonarSorted,
                  std::deque<double> &keyFramesTimeStampes, std::string &folderExperiment, std::string const HOME, double removeLinesUntil) {


    std::ifstream fileGroundTruth(HOME + folderExperiment + "/GTData.csv");
    if (fileGroundTruth.fail()) {
        std::cout << "fileGroundTruth file not found" << std::endl;
        exit(-1);
    }
    std::ifstream fileAngularVelocity(HOME + folderExperiment + "/IMUData.csv");
    if (fileAngularVelocity.fail()) {
        std::cout << "fileAngularVelocity file not found" << std::endl;
        exit(-1);
    }
    std::ifstream fileBodyVelocity(HOME + folderExperiment + "/dvlData.csv");
    if (fileBodyVelocity.fail()) {
        std::cout << "fileBodyVelocity file not found" << std::endl;
        exit(-1);
    }
    std::ifstream fileKeyframesDefined(HOME + folderExperiment + "/keyFrameTimeStamps.csv");
    if (fileKeyframesDefined.fail()) {
        std::cout << "fileKeyframesDefined file not found" << std::endl;
        exit(-1);
    }

    std::ifstream fileKeyIntensityDefined(HOME + folderExperiment + "/intensitiesData.csv");
    if (fileKeyIntensityDefined.fail()) {
        std::cout << "fileKeyIntensityDefined file not found" << std::endl;
        exit(-1);
    }


    //keyframes are always -1
    groundTruthSorted = parseCSVFileGT(fileGroundTruth, removeLinesUntil);

    Eigen::Matrix4d tranformationMatrixIMU;
    Eigen::AngleAxisd rotation_vectorz(90 / 180.0 * M_PI, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpMatrix3d = rotation_vectorz.toRotationMatrix();
    tranformationMatrixIMU.block<3, 3>(0, 0) = tmpMatrix3d;
    tranformationMatrixIMU(3, 3) = 1;
    ImuSorted = parseCSVFileIMU(fileAngularVelocity, tranformationMatrixIMU, removeLinesUntil);

    Eigen::Matrix4d tranformationMatrixDVL;
    Eigen::AngleAxisd rotation_vectorzDVL(-60 / 180.0 * M_PI, Eigen::Vector3d(0, 0, 1));
    tmpMatrix3d = rotation_vectorzDVL.toRotationMatrix();
    Eigen::AngleAxisd rotation_vectorxDVL(180 / 180.0 * M_PI, Eigen::Vector3d(1, 0, 0));
    tmpMatrix3d = rotation_vectorxDVL.toRotationMatrix() * tmpMatrix3d;
    tranformationMatrixDVL.block<3, 3>(0, 0) = tmpMatrix3d;
    tranformationMatrixDVL(3, 3) = 1;
    bodyVelocitySorted = parseCSVFileDVL(fileBodyVelocity, tranformationMatrixDVL, removeLinesUntil);
    keyFramesTimeStampes = parseCSVFileKeyFramesTimeStamps(fileKeyframesDefined, removeLinesUntil);
    intensitySonarSorted = parseCSVFileIntensityTimeStamps(fileKeyIntensityDefined, removeLinesUntil);
}


int
main(int argc, char **argv) {

    std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";//home path


    std::string folderExperiment = "/dataFolder/StPereDataset";// folder of experiment
    ros::init(argc, argv, "StPereDatasetPublishing01");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherIntensities, publisherImuData, publisherVelocityBody;
    publisherIntensities = n_.advertise<ping360_sonar::SonarEcho>("sonar/intensity", 10);
    publisherImuData = n_.advertise<sensor_msgs::Imu>("mavros/imu/data_frd", 10);
    publisherVelocityBody = n_.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body_frd", 10);


    std::deque<measurement> groundTruthSorted;
    std::deque<ImuData> IMUSorted;
    std::deque<DvlData> bodyVelocitySorted;
    std::deque<intensityMeasurement> intensitySonarSorted;
    std::deque<double> keyFramesTimeStampesSorted;
    double removeLinesUntil = 1093455407.03;
    loadCSVFiles(groundTruthSorted, IMUSorted, bodyVelocitySorted, intensitySonarSorted, keyFramesTimeStampesSorted,
                 folderExperiment, HOME,removeLinesUntil);

    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);


    //Matrices 180 degree rotation

    Eigen::Matrix4d transformationImu2PCL = Eigen::Matrix4d::Zero();
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));//was 180
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationImu2PCL.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationImu2PCL(3, 3) = 1;

    //pcl::transformPointCloud(*currentScan, *currentScan, transformation90Degree);


    while (ros::ok()) {
        //find lowest Timestamp
        //publish that thing
        //remove it and start anew.
        if (IMUSorted[0].timeStamp < bodyVelocitySorted[0].timeStamp &&
            IMUSorted[0].timeStamp < intensitySonarSorted[0].timeStamp &&
            IMUSorted[0].timeStamp < groundTruthSorted[0].timeStamp) {
            //publish imu data
            std::cout << "publish IMU" << std::endl;
            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time(IMUSorted[0].timeStamp);
            msg.angular_velocity.x = IMUSorted[0].wx;
            msg.angular_velocity.y = IMUSorted[0].wy;
            msg.angular_velocity.z = IMUSorted[0].wz;
            msg.linear_acceleration.x = IMUSorted[0].ax;
            msg.linear_acceleration.y = IMUSorted[0].ay;
            msg.linear_acceleration.z = IMUSorted[0].az;
            Eigen::Quaterniond currentOrientationAUV = generalHelpfulTools::getQuaternionFromRPY(IMUSorted[0].roll,
                                                                                                 IMUSorted[0].pitch, 0);

            msg.orientation.x = currentOrientationAUV.x();
            msg.orientation.y = currentOrientationAUV.y();
            msg.orientation.z = currentOrientationAUV.z();
            msg.orientation.w = currentOrientationAUV.w();

            publisherImuData.publish(msg);
            IMUSorted.pop_front();

        } else {
            if (bodyVelocitySorted[0].timeStamp < intensitySonarSorted[0].timeStamp &&
                bodyVelocitySorted[0].timeStamp < groundTruthSorted[0].timeStamp) {
                //publish body velocity
                std::cout << "publish VEL" << std::endl;
                geometry_msgs::TwistStamped msg;
                msg.header.stamp = ros::Time(bodyVelocitySorted[0].timeStamp);
                msg.twist.linear.x = bodyVelocitySorted[0].vx;
                msg.twist.linear.y = bodyVelocitySorted[0].vy;
                msg.twist.linear.z = bodyVelocitySorted[0].vz;
                publisherVelocityBody.publish(msg);
                bodyVelocitySorted.pop_front();

            } else {
                if (intensitySonarSorted[0].timeStamp < groundTruthSorted[0].timeStamp) {
                    //publish intensity
                    std::cout << "publish Intensity" << std::endl;
//                    std::cout << intensitySonarSorted.size() << std::endl;
                    ping360_sonar::SonarEcho msg;
                    msg.header.stamp = ros::Time(intensitySonarSorted[0].timeStamp);
                    msg.angle = intensitySonarSorted[0].angle;
                    msg.number_of_samples = intensitySonarSorted[0].size;

                    for (int k = 0; k < intensitySonarSorted[0].intensities.size(); k++) {
                        msg.intensities.push_back(intensitySonarSorted[0].intensities[k]);
                    }
                    publisherIntensities.publish(msg);
                    intensitySonarSorted.pop_front();

                } else {
                    //publish ground truth
                    std::cout << "publish GT" << std::endl;
                    groundTruthSorted.pop_front();
                }
            }
        }
        ros::Duration(0.03).sleep();
    }

    return (0);
}
