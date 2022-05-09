//
// Created by tim on 26.03.21.
//

#include "slamToolsRos.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include <iostream>

std::vector<measurement> parseCSVFileGT(std::istream &stream) {
    std::vector<measurement> returnVector;

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
        measurement tmpMeas{};
        tmpMeas.keyframe = -1;
        tmpMeas.x = std::stod(result[1]);
        tmpMeas.y = std::stod(result[2]);
        tmpMeas.z = 0;
        tmpMeas.timeStamp = std::stod(result[0]);
        returnVector.push_back(tmpMeas);
    }
    return returnVector;
}

std::deque<ImuData> parseCSVFileIMU(std::istream &stream, Eigen::Matrix4d &tranformationMatrixIMU) {
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
    return returnVector;
}

std::deque<DvlData> parseCSVFileDVL(std::istream &stream, Eigen::Matrix4d &tranformationMatrixDVL) {
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
        DvlData tmpMeas{};

        Eigen::Vector4d tmpPoint(std::stod(result[1]), std::stod(result[2]), std::stod(result[3]), 1);
        tmpPoint = tranformationMatrixDVL * tmpPoint;
        tmpMeas.vx = tmpPoint[0];
        tmpMeas.vy = tmpPoint[1];
        tmpMeas.vz = tmpPoint[2];
        tmpMeas.timeStamp = std::stod(result[0]);
        returnVector.push_back(tmpMeas);
    }
    return returnVector;
}

std::vector<double> parseCSVFileKeyFramesTimeStamps(std::istream &stream) {
    std::vector<double> returnVector;

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
        double tmpTime;
        tmpTime = std::stod(result[1]);
        returnVector.push_back(tmpTime);//@TODO the last line is still missing
    }
    return returnVector;
}

void loadCSVFiles(std::vector<measurement> &groundTruthSorted,
                  std::deque<ImuData> &ImuSorted,
                  std::deque<DvlData> &bodyVelocitySorted,
                  std::vector<double> &keyFramesTimeStampes, std::string &folderExperiment, std::string const HOME) {


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

    //keyframes are always -1
    groundTruthSorted = parseCSVFileGT(fileGroundTruth);

    Eigen::Matrix4d tranformationMatrixIMU;
    Eigen::AngleAxisd rotation_vectorz(90 / 180.0 * M_PI, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpMatrix3d = rotation_vectorz.toRotationMatrix();
    tranformationMatrixIMU.block<3, 3>(0, 0) = tmpMatrix3d;
    tranformationMatrixIMU(3, 3) = 1;
    ImuSorted = parseCSVFileIMU(fileAngularVelocity, tranformationMatrixIMU);

    Eigen::Matrix4d tranformationMatrixDVL;
    Eigen::AngleAxisd rotation_vectorzDVL(-60 / 180.0 * M_PI, Eigen::Vector3d(0, 0, 1));
    tmpMatrix3d = rotation_vectorzDVL.toRotationMatrix();
    Eigen::AngleAxisd rotation_vectorxDVL(180 / 180.0 * M_PI, Eigen::Vector3d(1, 0, 0));
    tmpMatrix3d = rotation_vectorxDVL.toRotationMatrix() * tmpMatrix3d;
    tranformationMatrixDVL.block<3, 3>(0, 0) = tmpMatrix3d;
    tranformationMatrixDVL(3, 3) = 1;
    bodyVelocitySorted = parseCSVFileDVL(fileBodyVelocity, tranformationMatrixDVL);
    keyFramesTimeStampes = parseCSVFileKeyFramesTimeStamps(fileKeyframesDefined);
}


int
main(int argc, char **argv) {

    std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";//home path


    std::string folderExperiment = "/dataFolder/StPereDataset";// folder of experiment
    ros::init(argc, argv, "StPereDatasetPublishing01");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherScan, publisherImuData, publisherVelocityBody;
    publisherScan = n_.advertise<sensor_msgs::PointCloud2>("sonar/full_scan", 10);
    publisherImuData = n_.advertise<sensor_msgs::Imu>("mavros/imu/data_frd", 10);
    publisherVelocityBody = n_.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body_frd", 10);


    std::vector<measurement> groundTruthSorted;
    std::deque<ImuData> IMUSorted;
    std::deque<DvlData> bodyVelocitySorted;
    std::vector<double> keyFramesTimeStampesSorted;
    loadCSVFiles(groundTruthSorted, IMUSorted, bodyVelocitySorted, keyFramesTimeStampesSorted,
                 folderExperiment, HOME);

    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);


    //Matrices 180 degree rotation

    Eigen::Matrix4d transformationImu2PCL= Eigen::Matrix4d::Zero();
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));//was 180
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationImu2PCL.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationImu2PCL(3, 3) = 1;

    //pcl::transformPointCloud(*currentScan, *currentScan, transformation90Degree);


    int startKeyFrame = 16;//was 16 maybe test start at 150
    //remove everything before startKeyFrame timestamp in IMU and DVL
    while (IMUSorted[0].timeStamp < keyFramesTimeStampesSorted[startKeyFrame] || bodyVelocitySorted[0].timeStamp < keyFramesTimeStampesSorted[startKeyFrame]) {
        //find which is smallest
        if (IMUSorted[0].timeStamp < bodyVelocitySorted[0].timeStamp) {
            IMUSorted.pop_front();
        } else {
            bodyVelocitySorted.pop_front();
        }
    }

    for (int currentKeyFrame = startKeyFrame;
         currentKeyFrame<50; currentKeyFrame++) {//keyFramesTimeStampesSorted.size()-1  || 50
        if (ros::ok()) {
            pcl::io::loadPCDFile(
                    HOME + "/pclKeyFrame" + std::to_string(currentKeyFrame) +
                    ".pcd",
                    *currentScan);
            //pcl::transformPointCloud(*currentScan, *currentScan, transformation90Degree);
            sensor_msgs::PointCloud2 currentScanMsg;
            pcl::toROSMsg(*currentScan,currentScanMsg);
            currentScanMsg.header.stamp = ros::Time(keyFramesTimeStampesSorted[currentKeyFrame]);
            currentScanMsg.header.frame_id = "map_ned";
            publisherScan.publish(currentScanMsg);
            std::cout << "waiting for Enter to keep running" << std::endl;
            ros::Duration(10.0).sleep();
            std::cout << "starting:" << std::endl;
            while (IMUSorted[0].timeStamp < keyFramesTimeStampesSorted[currentKeyFrame + 1] || bodyVelocitySorted[0].timeStamp < keyFramesTimeStampesSorted[currentKeyFrame + 1]) {
                //find which is smallest
                if (IMUSorted[0].timeStamp < bodyVelocitySorted[0].timeStamp) {
                    //smallest: angular velocity sort in
                    sensor_msgs::Imu msg;
                    msg.header.stamp = ros::Time(IMUSorted[0].timeStamp);
                    msg.angular_velocity.x = IMUSorted[0].wx;
                    msg.angular_velocity.y = IMUSorted[0].wy;
                    msg.angular_velocity.z = IMUSorted[0].wz;
                    msg.linear_acceleration.x = IMUSorted[0].ax;
                    msg.linear_acceleration.y = IMUSorted[0].ay;
                    msg.linear_acceleration.z = IMUSorted[0].az;
                    Eigen::Quaterniond currentOrientationAUV = generalHelpfulTools::getQuaternionFromRPY(IMUSorted[0].roll,IMUSorted[0].pitch,0);
                    msg.orientation.x = currentOrientationAUV.x();
                    msg.orientation.y = currentOrientationAUV.y();
                    msg.orientation.z = currentOrientationAUV.z();
                    msg.orientation.w = currentOrientationAUV.w();

                    publisherImuData.publish(msg);
                    IMUSorted.pop_front();
                } else {
                    //smallest: bodyVelocitySorted
                    geometry_msgs::TwistStamped msg;
                    msg.header.stamp = ros::Time(bodyVelocitySorted[0].timeStamp);
                    msg.twist.linear.x = bodyVelocitySorted[0].vx;
                    msg.twist.linear.y = bodyVelocitySorted[0].vy;
                    msg.twist.linear.z = bodyVelocitySorted[0].vz;
                    publisherVelocityBody.publish(msg);
                    bodyVelocitySorted.pop_front();
                }
                ros::Duration(0.001).sleep();
            }
            //some kind of sleep necessary.
            std::cout << "next: " << currentKeyFrame << std::endl;
        } else { exit(-1); }
    }
    return (0);
}
