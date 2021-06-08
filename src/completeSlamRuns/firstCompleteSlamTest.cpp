
#include <scanRegistrationClass.h>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <graphSlamSaveStructure.h>

#include <random>

double scalingAngle = 0.3;
double scalingAllg = 0.25;
double sigmaScaling = 3;

void visualizeCurrentGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                           ros::Publisher &publisherCloud, ros::Publisher &publisherMarkerArray) {

    nav_msgs::Path posOverTime;
    posOverTime.header.frame_id = "map_ned";
    Eigen::Matrix4d currentTransformation, completeTransformation;
    pcl::PointCloud<pcl::PointXYZ> completeCloudWithPos;
    visualization_msgs::MarkerArray markerArray;
    int i = 0;
    std::vector<vertex> vertexList = graphSaved.getVertexList();
    for (const auto &vertexElement : vertexList) {

        pcl::PointCloud<pcl::PointXYZ> currentScanTransformed;
        completeTransformation << 1, 0, 0, vertexElement.getPositionVertex().x(),
                0, 1, 0, vertexElement.getPositionVertex().y(),
                0, 0, 1, vertexElement.getPositionVertex().z(),
                0, 0, 0, 1;//transformation missing currently
        Eigen::Matrix3d m(vertexElement.getRotationVertex().toRotationMatrix());
        completeTransformation.block<3, 3>(0, 0) = m;
        pcl::transformPointCloud(*vertexElement.getPointCloud(), currentScanTransformed, completeTransformation);
        completeCloudWithPos += currentScanTransformed;


        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = vertexElement.getPositionVertex().x();
        pos.pose.position.y = vertexElement.getPositionVertex().y();
        pos.pose.position.z = vertexElement.getPositionVertex().z();
        pos.pose.orientation.x = vertexElement.getRotationVertex().x();
        pos.pose.orientation.y = vertexElement.getRotationVertex().y();
        pos.pose.orientation.z = vertexElement.getRotationVertex().z();
        pos.pose.orientation.w = vertexElement.getRotationVertex().w();

        posOverTime.poses.push_back(pos);

        visualization_msgs::Marker currentMarker;
        currentMarker.pose.position.x = pos.pose.position.x;
        currentMarker.pose.position.y = pos.pose.position.y;
        currentMarker.pose.position.z = pos.pose.position.z;
        currentMarker.pose.orientation.w = 1;
        currentMarker.header.frame_id = "map_ned";
        currentMarker.scale.x = sigmaScaling *
                                2 *
                                vertexElement.getCovariancePosition()[0];//missing covarianz since its in edge and only available after graph optimization
        currentMarker.scale.y = sigmaScaling *
                                2 *
                                vertexElement.getCovariancePosition()[1];//missing covarianz since its in edge and only available after graph optimization
        currentMarker.scale.z = 0;
        currentMarker.color.r = 0;
        currentMarker.color.g = 1;
        currentMarker.color.b = 0;
        currentMarker.color.a = 0.1;
        //currentMarker.lifetime.sec = 10;

        currentMarker.type = 2;
        currentMarker.id = i;
        i++;

        markerArray.markers.push_back(currentMarker);

    }

    publisherPath.publish(posOverTime);

    sensor_msgs::PointCloud2 cloudMsg2;
    pcl::toROSMsg(completeCloudWithPos, cloudMsg2);
    cloudMsg2.header.frame_id = "map_ned";
    publisherCloud.publish(cloudMsg2);

    publisherMarkerArray.publish(markerArray);

}

bool detectLoopClosure(graphSlamSaveStructure &graphSaved, scanRegistrationClass registrationClass) {
    Eigen::Vector3d estimatedPosLastPoint = graphSaved.getVertexList().back().getPositionVertex();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudLast = graphSaved.getVertexList().back().getPointCloud();
    Eigen::ArrayXXd dist;
    dist.resize(graphSaved.getVertexList().size() - 1, 1);
    for (int s = 0; s < graphSaved.getVertexList().size() - 1; s++) {
        //dist.row(s) = (graphSaved.getVertexList()[s].getPositionVertex() - estimatedPosLastPoint).norm();
        dist.row(s) = pow((estimatedPosLastPoint.x() - graphSaved.getVertexList()[s].getPositionVertex().x()), 2) /
                      pow((sigmaScaling * graphSaved.getVertexList()[s].getCovariancePosition().x()), 2) +
                      pow((estimatedPosLastPoint.y() - graphSaved.getVertexList()[s].getPositionVertex().y()), 2) /
                      pow((sigmaScaling * graphSaved.getVertexList()[s].getCovariancePosition().y()), 2);
    }
    const int ignoreLastNLoopClosures = 2;
    std::vector<int> has2beChecked;
    if (dist.size() > ignoreLastNLoopClosures) {
        for (int i = 0; i < dist.size() - ignoreLastNLoopClosures; i++) {
            if (dist(i, 0) < 1) {
                has2beChecked.push_back(i);
            }
        }

        std::cout << "Test loop closure for :" << has2beChecked.size() << std::endl;
        std::shuffle(has2beChecked.begin(), has2beChecked.end(), std::mt19937(std::random_device()()));
//        while(has2beChecked.size()>3){
//            has2beChecked.pop_back();//just check max 3
//        }
        int loopclosureNumber = 0;
        bool foundLoopClosure = false;
        for (const auto &has2beCheckedElemenet : has2beChecked) {
            double fitnessScore;
            Eigen::Matrix4d currentTransformation;
            currentTransformation = registrationClass.generalizedIcpRegistrationSimple(
                    graphSaved.getVertexList()[has2beCheckedElemenet].getPointCloud(),
                    graphSaved.getVertexList().back().getPointCloud(),
                    fitnessScore);
            fitnessScore = scalingAllg * sqrt(fitnessScore);
            if (fitnessScore < scalingAllg*4*0.1) {
                std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
                if (fitnessScore < 0.01) {
                    std::cout << "FitnessScore Very Low: " << fitnessScore << std::endl;
                    fitnessScore = 0.01;
                }
                Eigen::Vector3d currentPosDiff;
                Eigen::Quaterniond currentRotDiff(currentTransformation.inverse().block<3, 3>(0, 0));
                currentPosDiff.x() = currentTransformation.inverse()(0, 3);
                currentPosDiff.y() = currentTransformation.inverse()(1, 3);
                currentPosDiff.z() = 0;
                Eigen::Vector3d positionCovariance(fitnessScore, fitnessScore, 0);
                graphSaved.addEdge(has2beCheckedElemenet, (int) graphSaved.getVertexList().size() - 1, currentPosDiff,
                                   currentRotDiff, positionCovariance, (double) (0.1 * fitnessScore),graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                foundLoopClosure = true;
                loopclosureNumber++;
                if (loopclosureNumber > 1) { break; }// break if multiple loop closures are found
            }
        }
        if (foundLoopClosure) {
            return true;
        }

    }
    return false;
}


int
main(int argc, char **argv) {
    const int dimension = 3;

    ros::init(argc, argv, "exampleRegistration");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherKeyFrameClouds, publisherFirstScan, publisherPathOverTime, publisherMarkerArray;
    publisherFirstScan = n_.advertise<sensor_msgs::PointCloud2>("first_scan", 10);
    publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
    publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
    publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFirstScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);
    int i = 1;//was 16
    pcl::io::loadPCDFile("/home/tim/DataForTests/ScansOfLabyrinth/after_voxel_" + std::to_string(i - 1) + ".pcd",
                         *cloudFirstScan);
    *currentScan = *cloudFirstScan;


    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    scanRegistrationClass registrationClass = scanRegistrationClass();
    Eigen::Matrix4d currentTransformation, completeTransformation;
    completeTransformation << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //initialize first scan(for viewer only)
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloudFirstScan, cloudMsg);
    cloudMsg.header.frame_id = "map_ned";


    graphSlamSaveStructure graphSaved(dimension);

    Eigen::Vector3d firstPosition(0, 0, 0);
    Eigen::AngleAxisd rotationVectorFirst(0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond firstRotation(rotationVectorFirst.toRotationMatrix());
    Eigen::Vector3d covariancePos(0, 0, 0);
    graphSaved.addVertex(0, firstPosition, firstRotation, covariancePos, 0,
                         currentScan,graphSlamSaveStructure::INTEGRATED_POS_USAGE);//the first vertex sets 0 of the coordinate system

    std::deque<double> subgraphs{1, 3};
    graphSaved.initiallizeSubGraphs(subgraphs);

    double fitnessScore;

    for (; i < 8500; i++) {//old i = 70

        *lastScan = *currentScan;
        pcl::io::loadPCDFile("/home/tim/DataForTests/ScansOfLabyrinth/after_voxel_" + std::to_string(i) + ".pcd",
                             *currentScan);
        Eigen::Matrix4d guess;
        guess <<   1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        currentTransformation = registrationClass.generalizedIcpRegistration(lastScan, currentScan, Final,
                                                                             fitnessScore,guess);

        fitnessScore = scalingAllg * sqrt(fitnessScore);//0.05f;//constant fitness score test
        //completeTransformation = completeTransformation * currentTransformation;
        //if (completeTransformation.block<3, 1>(0, 3).norm() > 0.1) {
        if (fitnessScore < 0.01) {
            std::cout << "FitnessScore Very Low: " << fitnessScore << std::endl;
            fitnessScore = 0.01;
        }

        Eigen::Vector3d currentPosDiff;
        Eigen::Quaterniond currentRotDiff(currentTransformation.inverse().block<3, 3>(0, 0));


        currentPosDiff.x() = currentTransformation.inverse()(0, 3);
        currentPosDiff.y() = currentTransformation.inverse()(1, 3);
        currentPosDiff.z() = 0;
        Eigen::Quaterniond absolutRotation =
                currentRotDiff * graphSaved.getVertexByIndex(i - 1)->getRotationVertex();

        //Eigen::Vector3d lastCovariancePosOfVertex = graphSaved.getVertexList().back().getCovariancePosition();
        graphSaved.addVertex(i, graphSaved.getVertexByIndex(i - 1)->getPositionVertex() + currentPosDiff,
                             absolutRotation, Eigen::Vector3d(0, 0, 0), 0, currentScan,graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        Eigen::Vector3d positionCovariance(fitnessScore, fitnessScore, 0);
        graphSaved.addEdge(i - 1, i, currentPosDiff, currentRotDiff, positionCovariance,
                           (double) (scalingAngle * fitnessScore),
                           currentScan,graphSlamSaveStructure::INTEGRATED_POS_USAGE);


        bool detectedLoop = detectLoopClosure(graphSaved, registrationClass);//test loop closure
        if (detectedLoop) {
            std::cout << "loop detected" << std::endl;
        }
        std::cout << "###############################END OF LOOP############################### " << i + 1
                  << " comes next" << std::endl;


//        if (i % 100 == 0) {
//            std::vector<int> holdStill{0};
//            graphSaved.optimizeGraphWithSlam(false,holdStill);
//            graphSaved.initiallizeSubGraphs(subgraphs);
//            visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds, publisherMarkerArray);
//        }


        graphSaved.optimizeGraphWithSlamTopDown(false);
        graphSaved.calculateCovarianceInCloseProximity();//very expensive i guess
        //std::vector<vertex> stateAfterOptimization = hierachicalGraph.getVertexList();
        //Eigen::MatrixXf vectorToAdd = graphSaved.transformStateDiffToAddVector(stateBeforeOptimization,
        //                                                                       stateAfterOptimization, lookUpTable);
        graphSlamSaveStructure subGraph2 = graphSaved.getSubGraph().getSubGraph();
        visualizeCurrentGraph(subGraph2, publisherPathOverTime, publisherKeyFrameClouds, publisherMarkerArray);

        graphSlamSaveStructure subGraph = graphSaved.getSubGraph();
        visualizeCurrentGraph(subGraph, publisherPathOverTime, publisherKeyFrameClouds, publisherMarkerArray);


        //}
        //graphSaved.optimizeGraphWithSlam(false);
        visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds, publisherMarkerArray);
        publisherFirstScan.publish(cloudMsg);//publish first scan always
    }

    graphSaved.printCurrentStateGeneralInformation();
    std::vector<int> holdStill{0};
    graphSaved.optimizeGraphWithSlam(false,holdStill);
    graphSaved.printCurrentStateGeneralInformation();

    return (0);
}