//
// Created by tim on 26.03.21.
//
#include <ros/ros.h>
#include "graphSlamSaveStructure.h"
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
#include <random>
//#include "generalHelpfulTools.h"

#ifndef SIMULATION_BLUEROV_SLAMTOOLSROS_H
#define SIMULATION_BLUEROV_SLAMTOOLSROS_H

struct measurement {
    int keyframe;
    double x;
    double y;
    double z;
    double timeStamp;
};
struct ImuData {
    double ax;//linear acceleration
    double ay;//linear acceleration
    double az;//linear acceleration
    double wx;//angular velocity
    double wy;//angular velocity
    double wz;//angular velocity
    double roll;//from g measured
    double pitch;//from g measured
    double yaw;//mostly useless
    double timeStamp;
};

struct DvlData {
    double vx; // linear body velocity
    double vy; // linear body velocity
    double vz; // linear body velocity
    double height; // above sea
    double timeStamp;
};

struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
};

class slamToolsRos {

public:

    static void visualizeCurrentPoseGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                          ros::Publisher &publisherMarker, double sigmaScaling,
                                          ros::Publisher &publisherPoseSlam, ros::Publisher &publisherLoopClosures);

    static std::vector<measurement>
    parseCSVFile(std::istream &stream);//this is first line description then keyframe,x,y,z,timestamp

    static std::vector<std::vector<measurement>> sortToKeyframe(std::vector<measurement> &input);

    static void
    calculatePositionOverTime(std::deque<ImuData> &angularVelocityList, std::deque<DvlData> &bodyVelocityList,
                              std::vector<edge> &posOverTimeEdge,
                              double lastScanTimeStamp, double currentScanTimeStamp, double noiseAddedStdDiv,
                              int numberOfEdges);


    static std::vector<double> linspace(double start_in, double end_in, int num_in);

    static double createVoxelOfGraph(double voxelData[], int indexStart,
                                     Eigen::Matrix4d transformationInTheEndOfCalculation,
                                     int numberOfPoints, graphSlamSaveStructure &usedGraph,
                                     double ignoreDistanceToRobot, double dimensionOfVoxelData);

    static pcl::PointCloud<pcl::PointXYZ> createPCLFromGraphOneValue(int indexStart,
                                                              Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                              graphSlamSaveStructure &usedGraph,
                                                              double ignoreDistanceToRobo,
                                                              double thresholdFactorPoint);

    static pcl::PointCloud<pcl::PointXYZ> createPCLFromGraphOnlyThreshold(int indexStart,
                                                                   Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                                   graphSlamSaveStructure &usedGraph,
                                                                   double ignoreDistanceToRobo,
                                                                   double thresholdFactorPoint);
    static bool getNodes(ros::V_string &nodes);

    static edge calculatePoseDiffByTimeDepOnEKF(double startTimetoAdd, double endTimeToAdd, std::deque<double> &timeVector,
                                                              std::deque<double> &xPositionVector, std::deque<double> &yPositionVector,
                                                              std::deque<double> &zPositionVector, std::deque<Eigen::Quaterniond> &rotationVector, std::mutex &stateEstimationMutex);

    static double angleBetweenLastKeyframeAndNow(graphSlamSaveStructure &graphSaved);

    static int getLastIntensityKeyframe(graphSlamSaveStructure &graphSaved);

    static double getDatasetFromGraphForMap(std::vector<intensityValues> &dataSet , graphSlamSaveStructure &graphSaved, std::mutex &graphSlamMutex);

    static void clearSavingsOfPoses(double upToTime,std::deque<double> &timeVector,
                                    std::deque<double> &xPositionVector, std::deque<double> &yPositionVector,
                                    std::deque<double> &zPositionVector, std::deque<Eigen::Quaterniond> &rotationVector,std::mutex &stateEstimationMutex);
    };


#endif //SIMULATION_BLUEROV_VISUALIZESLAMINROS_H
