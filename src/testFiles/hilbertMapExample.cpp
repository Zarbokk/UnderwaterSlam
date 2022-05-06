//
// Created by tim on 26.03.21.
//

#include "../slamTools/slamToolsRos.h"
#include <hilbertMap.h>
#include "json.h"


std::vector<dataPointStruct> readGraphSlamJson(std::string fileName,double shiftX, double shiftY) {
    Json::Value keyFrames;
    std::ifstream keyFramesFile(fileName, std::ifstream::binary);
    keyFramesFile >> keyFrames;
    std::vector<dataPointStruct> dataSet;

    for (int i = 0; i < keyFrames["keyFrames"].size(); i++) {
        double roll = keyFrames["keyFrames"][i]["position"]["roll"].asDouble();
        double pitch = keyFrames["keyFrames"][i]["position"]["pitch"].asDouble();
        double yaw = keyFrames["keyFrames"][i]["position"]["yaw"].asDouble();
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d currentRotationMatrix = q.matrix();
        Eigen::Vector3d currentShift(keyFrames["keyFrames"][i]["position"]["x"].asDouble(),
                                     keyFrames["keyFrames"][i]["position"]["y"].asDouble(),
                                     keyFrames["keyFrames"][i]["position"]["z"].asDouble());

        for (int j = 0; j < keyFrames["keyFrames"][i]["pointCloud"].size(); j++) {
            Eigen::Vector3d pointPos(keyFrames["keyFrames"][i]["pointCloud"][j]["point"]["x"].asDouble(),
                                     keyFrames["keyFrames"][i]["pointCloud"][j]["point"]["y"].asDouble(),
                                     keyFrames["keyFrames"][i]["pointCloud"][j]["point"]["z"].asDouble());
            pointPos = currentRotationMatrix * pointPos + currentShift;
            dataPointStruct tmpDP;
            tmpDP.x = shiftX+pointPos.x();
            tmpDP.y = shiftY+pointPos.y();
            tmpDP.z = pointPos.z();
            tmpDP.occupancy = 1;
            dataSet.push_back(tmpDP);

            //create an additional point where occupancy = -1
            Eigen::Vector3d pointPosTwo(keyFrames["keyFrames"][i]["pointCloud"][j]["point"]["x"].asDouble(),
                                        keyFrames["keyFrames"][i]["pointCloud"][j]["point"]["y"].asDouble(),
                                        keyFrames["keyFrames"][i]["pointCloud"][j]["point"]["z"].asDouble());
            pointPosTwo = currentRotationMatrix * pointPosTwo;

            double randomNumber = std::rand() / double(RAND_MAX);// should be between 0 and 1
            pointPosTwo = currentShift + randomNumber * pointPosTwo;
            tmpDP.x = shiftX+pointPosTwo.x();
            tmpDP.y = shiftY+pointPosTwo.y();
            tmpDP.z = pointPosTwo.z();
            tmpDP.occupancy = -1;
            dataSet.push_back(tmpDP);
        }
    }
    return dataSet;
}


int
main(int argc, char **argv) {
    ros::init(argc, argv, "createHilbertMap");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherMarkerArray;
    publisherMarkerArray = n_.advertise<nav_msgs::OccupancyGrid>("occupancyHilbertMap", 10);

    hilbertMap mapRepresentation(256,128,
                                 60,hilbertMap::HINGED_FEATURES);
    mapRepresentation.createRandomMap();//initialize everything with 0.5

    ros::Rate loop_rate(1);


    std::vector<dataPointStruct> dataSet = readGraphSlamJson(
            "/home/tim-linux/dataFolder/savingGraphs/savedGraphTest.json",-5,0);

    double i=0;
    while (ros::ok()) {
        std::cout << "currently starting training" << std::endl;
        mapRepresentation.trainClassifier(dataSet,30000);
        nav_msgs::OccupancyGrid map = mapRepresentation.createOccupancyMapOfHilbert(10,10,15);
        map.header.stamp = ros::Time::now();
        map.header.frame_id = "map_ned";
        publisherMarkerArray.publish(map);
        ros::spinOnce();
        loop_rate.sleep();
        i=i+1;
        //std::cout << ros::Time::now() << std::endl;
    }

    return (0);
}
