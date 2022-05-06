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
#include "scanRegistrationClass.h"
#include <random>
#include "generalHelpfulTools.h"

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

class slamToolsRos {

public:
//    static void visualizeCurrentGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
//                                      ros::Publisher &publisherCloud, ros::Publisher &publisherMarkerArray,
//                                      double sigmaScaling, ros::Publisher &publisherPathGT,
//                                      std::vector<measurement> *groundTruthSorted,
//                                      ros::Publisher &publisherMarkerArrayLoopClosures,
//                                      double plotGTToTime,int numberOfEdgesBetweenScans);
    static void visualizeCurrentPoseGraph(graphSlamSaveStructure &graphSaved,ros::Publisher &publisherPath);

    static std::vector<measurement>
    parseCSVFile(std::istream &stream);//this is first line description then keyframe,x,y,z,timestamp

    static std::vector<std::vector<measurement>> sortToKeyframe(std::vector<measurement> &input);

//    static void correctPointCloudByPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScan, std::vector<edge> &posDiff,
//                                            double timeStampBeginning, double beginAngle,
//                                            double endAngle, bool reverseScanDirection,
//                                            Eigen::Matrix4d transformationPosData2PclCoord);

    static void
    calculatePositionOverTime(std::deque<ImuData> &angularVelocityList, std::deque<DvlData> &bodyVelocityList,
                              std::vector<edge> &posOverTimeEdge,
                              double lastScanTimeStamp, double currentScanTimeStamp, double noiseAddedStdDiv, int numberOfEdges);

//    static void
//    debugPlotting(pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan, pcl::PointCloud<pcl::PointXYZ>::Ptr afterRegistration,
//                  pcl::PointCloud<pcl::PointXYZ>::Ptr currentScanBeforeCorrection,
//                  pcl::PointCloud<pcl::PointXYZ>::Ptr currentScanAfterCorrection, ros::Publisher &publisherLastPCL,
//                  ros::Publisher &publisherRegistrationPCL, ros::Publisher &publisherBeforeCorrection,
//                  ros::Publisher &publisherAfterCorrection);

//    static bool detectLoopClosureSOFFT(graphSlamSaveStructure &graphSaved,
//                                  double sigmaScaling, double maxTimeOptimization,scanRegistrationClass &scanRegistrationObject);
//
//    static bool detectLoopClosureIPC(graphSlamSaveStructure &graphSaved,
//                                       double sigmaScaling, double cutoffFitnessOnDetect, double maxTimeOptimization,scanRegistrationClass &scanRegistrationObject);

    static std::vector<double> linspace(double start_in, double end_in, int num_in);

//    static void
//    correctPointCloudAtPos(int positionToCorrect, graphSlamSaveStructure &currentGraph,
//                           double beginAngle,
//                           double endAngle, bool reverseScanDirection, Eigen::Matrix4d transformationPosData2PclCoord);

//    static void correctEveryPointCloud(graphSlamSaveStructure &currentGraph,
//                                       double beginAngle,
//                                       double endAngle, bool reverseScanDirection,
//                                       Eigen::Matrix4d transformationPosData2PclCoord);

//    static void recalculatePCLEdges(graphSlamSaveStructure &currentGraph);

//    static void appendEdgesToGraph(graphSlamSaveStructure
//                                   &currentGraph,
//                                   std::deque<edge> &listOfEdges,
//                                   double noiseVelocityIntigration,
//                                   double scalingAngle, double maxTimeOptimization, int maximumNumberOfAddedEdges
//    );
};


#endif //SIMULATION_BLUEROV_VISUALIZESLAMINROS_H
