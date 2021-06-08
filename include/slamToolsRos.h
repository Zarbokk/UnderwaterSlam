//
// Created by tim on 26.03.21.
//
#include <ros/ros.h>
#include <graphSlamSaveStructure.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Path.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <scanRegistrationClass.h>
#include <random>

#ifndef SIMULATION_BLUEROV_SLAMTOOLSROS_H
#define SIMULATION_BLUEROV_SLAMTOOLSROS_H

struct measurement {
    int keyframe;
    double x;
    double y;
    double z;
    double timeStamp;
};

class slamToolsRos {

public:
    static void visualizeCurrentGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                      ros::Publisher &publisherCloud, ros::Publisher &publisherMarkerArray,
                                      double sigmaScaling, ros::Publisher &publisherPathGT,
                                      std::vector<std::vector<measurement>> &groundTruthSorted,
                                      ros::Publisher &publisherMarkerArrayLoopClosures);

    static std::vector<measurement>
    parseCSVFile(std::istream &stream);//this is first line description then keyframe,x,y,z,timestamp

    static std::vector<std::vector<measurement>> sortToKeyframe(std::vector<measurement> &input);

    static void correctPointCloudByPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScan, std::vector<edge> &posDiff,
                                            double timeStampBeginning);

    static void
    calculatePositionOverTime(std::vector<measurement> &angularVelocityList, std::vector<measurement> &bodyVelocityList,
                              std::vector<edge> &posOverTimeEdge,
                              double lastTimeStamp, double currentTimeStamp,double stdDev);

    static bool detectLoopClosure(graphSlamSaveStructure &graphSaved,
                                  double sigmaScaling, double cutoffFitnessOnDetect);

    static std::vector<double> linspace(double start_in, double end_in, int num_in);

    static void correctPointCloudAtPos(int positionToCorrect, graphSlamSaveStructure &currentGraph);

    static void correctEveryPointCloud(graphSlamSaveStructure &currentGraph);

    static void recalculatePCLEdges(graphSlamSaveStructure &currentGraph);
};


#endif //SIMULATION_BLUEROV_VISUALIZESLAMINROS_H
