//
// Created by tim on 26.03.21.
//

#include <slamToolsRos.h>

double scalingAngle = 0.25;
double scalingAllg = 0.25;
double sigmaScaling = 2;
double noiseVelocityIntigration = 0.3;

void loadCSVFiles(std::vector<measurement> &groundTruthSorted,
                  std::vector<std::vector<measurement>> &angularVelocitySorted,
                  std::vector<std::vector<measurement>> &bodyVelocitySorted, std::string &folderExperiment,std::string const HOME) {


    std::ifstream fileGroundTruth(HOME+"/DataForTests/" + folderExperiment + "/groundTruth.csv");
    if (fileGroundTruth.fail()){
        std::cout << "fileGroundTruth file not found"<< std::endl;
        exit(-1);
    }
    std::ifstream fileAngularVelocity(HOME+"/DataForTests/" + folderExperiment + "/angularVelocity.csv");
    if (fileAngularVelocity.fail()){
        std::cout << "fileAngularVelocity file not found"<< std::endl;
        exit(-1);
    }
    std::ifstream fileBodyVelocity(HOME+"/DataForTests/" + folderExperiment + "/bodyVelocity.csv");
    if (fileBodyVelocity.fail()){
        std::cout << "fileBodyVelocity file not found"<< std::endl;
        exit(-1);
    }

    groundTruthSorted = slamToolsRos::parseCSVFile(fileGroundTruth);
    std::vector<measurement> angularVelocity = slamToolsRos::parseCSVFile(fileAngularVelocity);
    std::vector<measurement> bodyVelocity = slamToolsRos::parseCSVFile(fileBodyVelocity);
    //groundTruthSorted = slamToolsRos::sortToKeyframe(groundTruth);
    angularVelocitySorted = slamToolsRos::sortToKeyframe(angularVelocity);
    angularVelocitySorted.pop_back();
    bodyVelocitySorted = slamToolsRos::sortToKeyframe(bodyVelocity);
    bodyVelocitySorted.pop_back();

}

void appendEdgesToGraph(graphSlamSaveStructure &currentGraph,
                        std::vector<edge> &listOfEdges) {// adds edges to the graph and create vertex, which are represented by edges
    int i = 1;
    for (auto &currentEdge : listOfEdges) {
        vertex lastVertex = currentGraph.getVertexList().back();
        Eigen::Matrix4d tmpTransformation = lastVertex.getTransformation();
        tmpTransformation = tmpTransformation * currentEdge.getTransformation();
        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
        Eigen::Quaterniond rot(rotM);

        currentGraph.addVertex(lastVertex.getVertexNumber() + 1, pos, rot, lastVertex.getCovariancePosition(),
                               lastVertex.getCovarianceQuaternion(), currentEdge.getTimeStamp(),
                               graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        currentGraph.addEdge(lastVertex.getVertexNumber(), lastVertex.getVertexNumber() + 1,
                             currentEdge.getPositionDifference(), currentEdge.getRotationDifference(),
                             Eigen::Vector3d(noiseVelocityIntigration, noiseVelocityIntigration, 0),
                             scalingAngle * noiseVelocityIntigration, graphSlamSaveStructure::INTEGRATED_POS_USAGE);
//        graphSaved.getVertexList().back().setTypeOfVertex(
//                graphSlamSaveStructure::INTEGRATED_POS_USAGE);//1 for vertex defined by dead reckoning
        i++;
    }
}

void debugPlotting(pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan, pcl::PointCloud<pcl::PointXYZ>::Ptr afterRegistration,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr currentScanBeforeCorrection,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr currentScanAfterCorrection,
                   ros::Publisher &publisherLastPCL,
                   ros::Publisher &publisherRegistrationPCL,
                   ros::Publisher &publisherBeforeCorrection,
                   ros::Publisher &publisherAfterCorrection) {

    sensor_msgs::PointCloud2 lastPCLMsg;
    pcl::toROSMsg(*lastScan, lastPCLMsg);
    lastPCLMsg.header.frame_id = "map_ned";
    publisherLastPCL.publish(lastPCLMsg);

    sensor_msgs::PointCloud2 afterRegistrationMsg;
    pcl::toROSMsg(*afterRegistration, afterRegistrationMsg);
    afterRegistrationMsg.header.frame_id = "map_ned";
    publisherRegistrationPCL.publish(afterRegistrationMsg);

    sensor_msgs::PointCloud2 beforeCorrectionMsg;
    pcl::toROSMsg(*currentScanBeforeCorrection, beforeCorrectionMsg);
    beforeCorrectionMsg.header.frame_id = "map_ned";
    publisherBeforeCorrection.publish(beforeCorrectionMsg);

    sensor_msgs::PointCloud2 afterCorrectionMsg;
    pcl::toROSMsg(*currentScanAfterCorrection, afterCorrectionMsg);
    afterCorrectionMsg.header.frame_id = "map_ned";
    publisherAfterCorrection.publish(afterCorrectionMsg);

}


int
main(int argc, char **argv) {

    std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";//home path


    std::string folderExperiment = "withoutRotation";// folder of experiment
    ros::init(argc, argv, "slamLabyrinth01");
    ros::start();
    ros::NodeHandle n_;
    ros::Publisher publisherFirstScan, publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;
    publisherFirstScan = n_.advertise<sensor_msgs::PointCloud2>("first_scan", 10);
    publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
    publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
    publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
    publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
    publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
    publisherLastPCL = n_.advertise<sensor_msgs::PointCloud2>("lastPCL", 10);
    publisherRegistrationPCL = n_.advertise<sensor_msgs::PointCloud2>("registratedPCL", 10);
    publisherBeforeCorrection = n_.advertise<sensor_msgs::PointCloud2>("beforeCorrection", 10);
    publisherAfterCorrection = n_.advertise<sensor_msgs::PointCloud2>("afterCorrection", 10);



    std::vector<measurement> groundTruthSorted;
    std::vector<std::vector<measurement>> angularVelocitySorted;
    std::vector<std::vector<measurement>> bodyVelocitySorted;
    loadCSVFiles(groundTruthSorted, angularVelocitySorted, bodyVelocitySorted, folderExperiment,HOME);

    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(
            new pcl::PointCloud<pcl::PointXYZ>);//@TODOoutput of matching process should be deleted later
    pcl::io::loadPCDFile(HOME+"/DataForTests/" + folderExperiment + "/after_voxel_1.pcd",
                         *currentScan);

    //Matrices
    Eigen::Matrix4d currentTransformation;
    //180 degree rotation
    Eigen::Matrix4d transformation90Degree;
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformation90Degree.block<3, 3>(0, 0) = tmpMatrix3d;
    transformation90Degree(3, 3) = 1;

    pcl::transformPointCloud(*currentScan, *currentScan, transformation90Degree);

    std::vector<vertex> posDiffOverTimeVertices;
    std::vector<edge> posDiffOverTimeEdges;

    double lastTimeKeyFrame = groundTruthSorted[0].timeStamp;//280
    double timeCurrentGroundTruth = groundTruthSorted[1].timeStamp;//290
    double fitnessScore;
    bool debug = true;

    sensor_msgs::PointCloud2 firstScanMsg;
    pcl::toROSMsg(*currentScan, firstScanMsg);
    firstScanMsg.header.frame_id = "map_ned";
    //add first vertex
    graphSlamSaveStructure graphSaved(3);
    graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond( 1, 0, 0, 0),
                         Eigen::Vector3d(0, 0, 0), 0, lastTimeKeyFrame, graphSlamSaveStructure::FIRST_ENTRY);

    //first step
    slamToolsRos::calculatePositionOverTime(angularVelocitySorted[1], bodyVelocitySorted[1],
                                            posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth,0.0);

    //add vertex and so on to graphSaved
    appendEdgesToGraph(graphSaved, posDiffOverTimeEdges);
    graphSaved.getVertexList().back().setPointCloudRaw(currentScan);
    //correct last PCL
    slamToolsRos::correctPointCloudAtPos(graphSaved.getVertexList().back().getVertexNumber(), graphSaved);


    //initialize hierachical slam
    std::deque<double> subgraphs{1, 3};
    graphSaved.initiallizeSubGraphs(subgraphs);

    for (int currentKeyFrame = 2; currentKeyFrame < groundTruthSorted.size(); currentKeyFrame++) {
        if(ros::ok()){
        //for (int currentKeyFrame = 2; currentKeyFrame < 100; currentKeyFrame++) {
            *lastScan = *graphSaved.getVertexList().back().getPointCloudCorrected();
            lastTimeKeyFrame = timeCurrentGroundTruth;
            timeCurrentGroundTruth = groundTruthSorted[currentKeyFrame].timeStamp;

            pcl::io::loadPCDFile(
                    HOME+"/DataForTests/" + folderExperiment + "/after_voxel_" + std::to_string(currentKeyFrame) +
                    ".pcd",
                    *currentScan);
            pcl::transformPointCloud(*currentScan, *currentScan, transformation90Degree);

            //forward calculate pose(relative)(with velocities) add edges+vertexes
            slamToolsRos::calculatePositionOverTime(angularVelocitySorted[currentKeyFrame],
                                                    bodyVelocitySorted[currentKeyFrame],
                                                    posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth,0.5);// was 0.8
            //sort in posDiffOverTime and calculate vertices to be added
            appendEdgesToGraph(graphSaved, posDiffOverTimeEdges);
            graphSaved.getVertexList().back().setPointCloudRaw(currentScan);

            //re-map point cloud
            slamToolsRos::correctPointCloudAtPos(graphSaved.getVertexList().back().getVertexNumber(), graphSaved);

            //make scan matching with last scan
            Eigen::Matrix4d initialGuessTransformation =
                    graphSaved.getVertexList()[graphSaved.getVertexList().size() -
                                               9].getTransformation().inverse() *
                    graphSaved.getVertexList().back().getTransformation();//@todo understand if 9 is correct
            currentTransformation = scanRegistrationClass::generalizedIcpRegistration(graphSaved.getVertexList().back().getPointCloudCorrected(), lastScan, Final,
                                                                                      fitnessScore,
                                                                                      initialGuessTransformation);
            if(debug){
                debugPlotting(lastScan, Final, currentScan, graphSaved.getVertexList().back().getPointCloudCorrected(),
                              publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection);
            }
            std::cout << "current Fitness Score: " << sqrt(fitnessScore) << std::endl;
            //add edge for currentTransformation(registration)
            Eigen::Quaterniond qTMP(currentTransformation.block<3, 3>(0, 0));
            graphSaved.addEdge(graphSaved.getVertexList().size() - 10, graphSaved.getVertexList().size() - 1,
                               currentTransformation.block<3, 1>(0, 3), qTMP,
                               Eigen::Vector3d(sqrt(fitnessScore), sqrt(fitnessScore), 0),
                               0.25*sqrt(fitnessScore),
                               graphSlamSaveStructure::POINT_CLOUD_USAGE);//@TODO still not sure about size

            //watch for loop closure
            slamToolsRos::detectLoopClosure(graphSaved, sigmaScaling, 1.0);//was 0.7
            //optimization of graph (hierachical graph)
            graphSaved.optimizeGraphWithSlamTopDown(false, 0.05);
            std::vector<int> holdStill{0};

            graphSaved.calculateCovarianceInCloseProximity();
            //visualization of graph in ros

            slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                                publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                                groundTruthSorted, publisherMarkerArrayLoopClosures,lastTimeKeyFrame);
            std::cout << "next: " << currentKeyFrame << std::endl;
        }else{exit(-1);}
    }
    std::vector<int> holdStill{0};
    graphSaved.optimizeGraphWithSlam(false, holdStill);
    slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                        publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                        groundTruthSorted, publisherMarkerArrayLoopClosures,timeCurrentGroundTruth);

    for (int i = 0; i < 1; i++) {
        //correct all point clouds
        slamToolsRos::correctEveryPointCloud(graphSaved);
        //recalculate the edges
        slamToolsRos::recalculatePCLEdges(graphSaved);



        //show
        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted, publisherMarkerArrayLoopClosures,timeCurrentGroundTruth);
        //optimize again
        graphSaved.optimizeGraphWithSlam(false, holdStill);
        //show
        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted, publisherMarkerArrayLoopClosures,timeCurrentGroundTruth);
    }

    graphSaved.saveGraphJson("testfile.json");

    return (0);
}
