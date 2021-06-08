//
// Created by tim on 26.03.21.
//

#include <slamToolsRos.h>

double scalingAngle = 0.05;
double scalingAllg = 0.25;
double sigmaScaling = 3;
double noiseVelocityIntigration = 0.3;

void loadCSVFiles(std::vector<std::vector<measurement>> &groundTruthSorted,
                  std::vector<std::vector<measurement>> &angularVelocitySorted,
                  std::vector<std::vector<measurement>> &bodyVelocitySorted, std::string &folderExperiment) {


    std::ifstream fileGroundTruth("/home/tim/DataForTests/" + folderExperiment + "/groundTruth.csv");
    std::ifstream fileAngularVelocity("/home/tim/DataForTests/" + folderExperiment + "/angularVelocity.csv");
    std::ifstream fileBodyVelocity("/home/tim/DataForTests/" + folderExperiment + "/bodyVelocity.csv");

    std::vector<measurement> groundTruth = slamToolsRos::parseCSVFile(fileGroundTruth);
    std::vector<measurement> angularVelocity = slamToolsRos::parseCSVFile(fileAngularVelocity);
    std::vector<measurement> bodyVelocity = slamToolsRos::parseCSVFile(fileBodyVelocity);
    groundTruthSorted = slamToolsRos::sortToKeyframe(groundTruth);
    angularVelocitySorted = slamToolsRos::sortToKeyframe(angularVelocity);
    angularVelocitySorted.pop_back();
    bodyVelocitySorted = slamToolsRos::sortToKeyframe(bodyVelocity);
    bodyVelocitySorted.pop_back();

}

void appendEdgesToGraph(graphSlamSaveStructure &currentGraph,std::vector<edge> &listOfEdges){// adds edges to the graph and create vertex, which are represented by edges
    int i = 1;
    for (auto &currentEdge : listOfEdges) {
        vertex lastVertex = currentGraph.getVertexList().back();
        Eigen::Matrix4d tmpTransformation = lastVertex.getTransformation();
        tmpTransformation = tmpTransformation * currentEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);

        currentGraph.addVertex(lastVertex.getVertexNumber() + 1, pos, rot, lastVertex.getCovariancePosition(),
                               lastVertex.getCovarianceQuaternion(),currentEdge.getTimeStamp(), graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        currentGraph.addEdge(lastVertex.getVertexNumber(), lastVertex.getVertexNumber() + 1,
                             currentEdge.getPositionDifference(), currentEdge.getRotationDifference(),
                             Eigen::Vector3d(noiseVelocityIntigration, noiseVelocityIntigration, 0),
                             scalingAngle * noiseVelocityIntigration, graphSlamSaveStructure::INTEGRATED_POS_USAGE);
//        graphSaved.getVertexList().back().setTypeOfVertex(
//                graphSlamSaveStructure::INTEGRATED_POS_USAGE);//1 for vertex defined by dead reckoning
        i++;
    }
}



int
main(int argc, char **argv) {
    std::string folderExperiment = "dataset_04";// folder of experiment
    ros::init(argc, argv, "slamLabyrinth01");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherFirstScan, publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT,publisherBeforeCorrection,publisherAfterCorrection,publisherLastPCL,publisherRegistrationPCL;
    publisherFirstScan = n_.advertise<sensor_msgs::PointCloud2>("first_scan", 10);
    publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
    publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
    publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
    publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
    publisherBeforeCorrection = n_.advertise<sensor_msgs::PointCloud2>("beforeCorrection", 10);
    publisherAfterCorrection= n_.advertise<sensor_msgs::PointCloud2>("afterCorrection", 10);
    publisherLastPCL= n_.advertise<sensor_msgs::PointCloud2>("lastPCL", 10);
    publisherRegistrationPCL= n_.advertise<sensor_msgs::PointCloud2>("registratedPCL", 10);
    //scanRegistrationClass registrationClass = scanRegistrationClass();

    //slamToolsRos toolsObject;
    std::vector<std::vector<measurement>> groundTruthSorted;
    std::vector<std::vector<measurement>> angularVelocitySorted;
    std::vector<std::vector<measurement>> bodyVelocitySorted;
    loadCSVFiles(groundTruthSorted, angularVelocitySorted, bodyVelocitySorted, folderExperiment);

    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(
            new pcl::PointCloud<pcl::PointXYZ>);//@TODOoutput of matching process should be deleted later
    pcl::io::loadPCDFile("/home/tim/DataForTests/" + folderExperiment + "/after_voxel_1.pcd",
                         *currentScan);

    //Matrices
    Eigen::Matrix4d currentTransformation;
    //180 degree rotation
    Eigen::Matrix4d transformation90Degree;
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * M_PI, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformation90Degree.block<3, 3>(0, 0) = tmpMatrix3d;
    transformation90Degree(3,3)=1;

    pcl::transformPointCloud(*currentScan,*currentScan,transformation90Degree);

    std::vector<vertex> posDiffOverTimeVertices;
    std::vector<edge> posDiffOverTimeEdges;

    double lastTimeKeyFrame = groundTruthSorted[0][0].timeStamp;//280
    double timeCurrentGroundTruth = groundTruthSorted[1][0].timeStamp;//290
    double fitnessScore;


    //graphSlamSaveStructure graphSaved = initializeGraph(currentScan);
    sensor_msgs::PointCloud2 firstScanMsg;
    pcl::toROSMsg(*currentScan, firstScanMsg);
    firstScanMsg.header.frame_id = "map_ned";
    //add first vertex
    graphSlamSaveStructure graphSaved(3);
    graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                         Eigen::Vector3d(0, 0, 0),0,lastTimeKeyFrame, graphSlamSaveStructure::FIRST_ENTRY);

    //first step
    slamToolsRos::calculatePositionOverTime(angularVelocitySorted[1], bodyVelocitySorted[1],
                                            posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth);

    //add vertex and so on to graphSaved
    appendEdgesToGraph(graphSaved,posDiffOverTimeEdges);
    graphSaved.getVertexList().back().setPointCloudRaw(currentScan);

//    slamToolsRos::correctPointCloudByPosition(currentScan, posDiffOverTimeEdges, lastTimeKeyFrame);
    slamToolsRos::correctPointCloudAtPos( graphSaved.getVertexList().back().getVertexNumber(), graphSaved);
    slamToolsRos::correctPointCloudByPosition(currentScan, posDiffOverTimeEdges, lastTimeKeyFrame);

//    graphSaved.getVertexList().back().setPointCloudCorrected(currentScan);
//    graphSaved.getVertexList().back().setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);

    //initialize hierachical slam
    std::deque<double> subgraphs{1, 3};
    graphSaved.initiallizeSubGraphs(subgraphs);

    for (int currentKeyFrame = 2; currentKeyFrame < groundTruthSorted.size(); currentKeyFrame++) {
    //for (int currentKeyFrame = 2; currentKeyFrame < 55; currentKeyFrame++) {
        *lastScan = *graphSaved.getVertexList().back().getPointCloudCorrected();
        lastTimeKeyFrame = timeCurrentGroundTruth;
        timeCurrentGroundTruth = groundTruthSorted[currentKeyFrame][0].timeStamp;

        pcl::io::loadPCDFile(
                "/home/tim/DataForTests/" + folderExperiment + "/after_voxel_" + std::to_string(currentKeyFrame) +
                ".pcd",
                *currentScan);
        pcl::transformPointCloud(*currentScan,*currentScan,transformation90Degree);

        //forward calculate pose(relative)(with velocities) add edges+vertexes
        slamToolsRos::calculatePositionOverTime(angularVelocitySorted[currentKeyFrame],
                                                bodyVelocitySorted[currentKeyFrame],
                                                posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth);
        //sort in posDiffOverTime and calculate vertices to be added
        appendEdgesToGraph(graphSaved,posDiffOverTimeEdges);
        graphSaved.getVertexList().back().setPointCloudRaw(currentScan);

        //re-map point cloud
        slamToolsRos::correctPointCloudAtPos( graphSaved.getVertexList().back().getVertexNumber(), graphSaved);


        sensor_msgs::PointCloud2 beforeCorrectionMsg;
        pcl::toROSMsg(*currentScan, beforeCorrectionMsg);
        beforeCorrectionMsg.header.frame_id = "map_ned";
        publisherBeforeCorrection.publish(beforeCorrectionMsg);

        //re-map point cloud
        slamToolsRos::correctPointCloudByPosition(currentScan, posDiffOverTimeEdges,
                                                  lastTimeKeyFrame);//has to be done with edges

        sensor_msgs::PointCloud2 afterCorrectionMsg;
        pcl::toROSMsg(*currentScan, afterCorrectionMsg);
        afterCorrectionMsg.header.frame_id = "map_ned";
        publisherAfterCorrection.publish(afterCorrectionMsg);

//        slamToolsRos::correctPointCloudByPosition(currentScan, posDiffOverTimeEdges,
//                                                  lastTimeKeyFrame);//has to be done with edges
        //Add current Scan to the graph additionally set point cloud usage true
//        graphSaved.getVertexList().back().setPointCloudCorrected(currentScan);
        //graphSaved.getVertexList().back().setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);
//        graphSaved.getVertexByIndex(graphSaved.getVertexList().size() - 1)->setTypeOfVertex(
//                graphSlamSaveStructure::POINT_CLOUD_USAGE);

        //make scan matching with last scan

        Eigen::Matrix4d initialGuessTransformation =
                graphSaved.getVertexList()[graphSaved.getVertexList().size() -
                                           9].getTransformation().inverse() *
                graphSaved.getVertexList().back().getTransformation();//@todo understand if 9 is correct
        sensor_msgs::PointCloud2 lastPCLMsg;
        pcl::toROSMsg(*lastScan, lastPCLMsg);
        lastPCLMsg.header.frame_id = "map_ned";
        publisherLastPCL.publish(lastPCLMsg);

        currentTransformation = scanRegistrationClass::generalizedIcpRegistration(currentScan, lastScan, Final,
                                                                                  fitnessScore, initialGuessTransformation);

        sensor_msgs::PointCloud2 afterRegistrationMsg;
        pcl::toROSMsg(*Final, afterRegistrationMsg);
        afterRegistrationMsg.header.frame_id = "map_ned";
        publisherRegistrationPCL.publish(afterRegistrationMsg);


        //fitnessScore = fitnessScore*fitnessScore;
        std::cout << "current Fitness Score: " << sqrt(fitnessScore) << std::endl;
        //add edge for currentTransformation
        Eigen::Quaterniond qTMP(currentTransformation.block<3, 3>(0, 0));
        graphSaved.addEdge(graphSaved.getVertexList().size() - 10, graphSaved.getVertexList().size() - 1,
                           currentTransformation.block<3, 1>(0, 3), qTMP,
                           Eigen::Vector3d(sqrt(fitnessScore), sqrt(fitnessScore), 0),
                           (double) sqrt(fitnessScore),
                           graphSlamSaveStructure::POINT_CLOUD_USAGE);//@TODO still not sure about size

        //add pointCloud to Vertex and change type
        graphSaved.getVertexList().back().setPointCloudCorrected(currentScan);
        graphSaved.getVertexList().back().setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);

        //watch for loop closure
        slamToolsRos::detectLoopClosure(graphSaved, sigmaScaling, 0.7);
        //optimization of graph
        graphSaved.optimizeGraphWithSlamTopDown(false);
        //std::vector<int> holdStill{0};
        bool calcEverythingAnew = false;//for debugging
        if (calcEverythingAnew) {
            std::vector<int> holdStill{0};
            graphSaved.optimizeGraphWithSlam(false, holdStill);
        }
        //graphSaved.optimizeGraphWithSlam(false,holdStill);
        graphSaved.calculateCovarianceInCloseProximity();
        //visualization of graph in ros

        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted);

        std::cout << "next: " << currentKeyFrame << std::endl;
    }
    std::vector<int> holdStill{0};
    graphSaved.optimizeGraphWithSlam(false, holdStill);
    slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                        publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                        groundTruthSorted);

    for(int i = 0 ; i<1000;i++){
        //correct all point clouds
        slamToolsRos::correctEveryPointCloud(graphSaved);
        //recalculate the edges
        slamToolsRos::recalculatePCLEdges(graphSaved);



        //show
        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted);
        //optimize again
        graphSaved.optimizeGraphWithSlam(false, holdStill);
        //show
        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted);
    }
    return (0);
}
