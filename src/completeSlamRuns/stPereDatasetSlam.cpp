//
// Created by tim on 26.03.21.
//

#include <slamToolsRos.h>

double scalingAngle = 0.25;
double scalingAllg = 0.25;
double sigmaScaling = 2;
double noiseVelocityIntigration = 0.5;

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

std::vector<measurement> parseCSVFileIMU(std::istream &stream, Eigen::Matrix4d &tranformationMatrixIMU) {
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
        Eigen::Vector4d tmpPoint(std::stod(result[4]), std::stod(result[5]), std::stod(result[6]), 1);
        tmpPoint = tranformationMatrixIMU * tmpPoint;
        tmpMeas.x = tmpPoint[0];
        tmpMeas.y = tmpPoint[1];
        tmpMeas.z = tmpPoint[2];
        tmpMeas.timeStamp = std::stod(result[0]);
        returnVector.push_back(tmpMeas);
    }
    return returnVector;
}

std::vector<measurement> parseCSVFileDVL(std::istream &stream, Eigen::Matrix4d &tranformationMatrixDVL) {
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
        Eigen::Vector4d tmpPoint(std::stod(result[1]), std::stod(result[2]), std::stod(result[3]), 1);
        tmpPoint = tranformationMatrixDVL * tmpPoint;
        tmpMeas.x = tmpPoint[0];
        tmpMeas.y = tmpPoint[1];
        tmpMeas.z = tmpPoint[2];
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
                  std::vector<measurement> &angularVelocitySorted,
                  std::vector<measurement> &bodyVelocitySorted,
                  std::vector<double> &keyFramesTimeStampes, std::string &folderExperiment, std::string const HOME) {


    std::ifstream fileGroundTruth(HOME + "/DataForTests/" + folderExperiment + "/GTData.csv");
    if (fileGroundTruth.fail()) {
        std::cout << "fileGroundTruth file not found" << std::endl;
        exit(-1);
    }
    std::ifstream fileAngularVelocity(HOME + "/DataForTests/" + folderExperiment + "/IMUData.csv");
    if (fileAngularVelocity.fail()) {
        std::cout << "fileAngularVelocity file not found" << std::endl;
        exit(-1);
    }
    std::ifstream fileBodyVelocity(HOME + "/DataForTests/" + folderExperiment + "/dvlData.csv");
    if (fileBodyVelocity.fail()) {
        std::cout << "fileBodyVelocity file not found" << std::endl;
        exit(-1);
    }
    std::ifstream fileKeyframesDefined(HOME + "/DataForTests/" + folderExperiment + "/keyFrameTimeStamps.csv");
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
    angularVelocitySorted = parseCSVFileIMU(fileAngularVelocity, tranformationMatrixIMU);

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

void appendEdgesToGraph(graphSlamSaveStructure &currentGraph,
                        std::vector<edge> &listodEdges) {// adds edges to the graph and create vertex, which are represented by edges
    int i = 1;
    for (auto &currentEdge : listodEdges) {
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


    std::string folderExperiment = "StPereDataset";// folder of experiment
    ros::init(argc, argv, "StPereDatasetSLAM01");
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
    std::vector<measurement> angularVelocitySorted;
    std::vector<measurement> bodyVelocitySorted;
    std::vector<double> keyFramesTimeStampesSorted;
    loadCSVFiles(groundTruthSorted, angularVelocitySorted, bodyVelocitySorted, keyFramesTimeStampesSorted,
                 folderExperiment, HOME);

    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(
            new pcl::PointCloud<pcl::PointXYZ>);//@TODOoutput of matching process should be deleted later
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloudPlotOnly(
            new pcl::PointCloud<pcl::PointXYZ>);

    //Matrices
    Eigen::Matrix4d currentTransformation = Eigen::Matrix4d::Zero();
    //180 degree rotation

    Eigen::Matrix4d transformationImu2PCL= Eigen::Matrix4d::Zero();
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));//was 180
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationImu2PCL.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationImu2PCL(3, 3) = 1;

    //pcl::transformPointCloud(*currentScan, *currentScan, transformation90Degree);

    std::vector<vertex> posDiffOverTimeVertices;
    std::vector<edge> posDiffOverTimeEdges;

    int startKeyFrame = 16;//was 16 maybe test start at 150
    double lastTimeKeyFrame = keyFramesTimeStampesSorted[startKeyFrame];//280
    double timeCurrentGroundTruth = keyFramesTimeStampesSorted[startKeyFrame + 1];//290
    double fitnessScore;

    bool debug = true;


    pcl::io::loadPCDFile(
            HOME + "/DataForTests/" + folderExperiment + "/pclKeyFrame" + std::to_string(startKeyFrame) + ".pcd",
            *currentScan);
    sensor_msgs::PointCloud2 firstScanMsg;
    pcl::toROSMsg(*currentScan, firstScanMsg);
    firstScanMsg.header.frame_id = "map_ned";
    //add first vertex
    graphSlamSaveStructure graphSaved(3);
    graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                         Eigen::Vector3d(0, 0, 0), 0, lastTimeKeyFrame, graphSlamSaveStructure::FIRST_ENTRY);

    //first step
    slamToolsRos::calculatePositionOverTime(angularVelocitySorted, bodyVelocitySorted,
                                            posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth, 0.0);

    //add vertex and so on to graphSaved
    appendEdgesToGraph(graphSaved, posDiffOverTimeEdges);
    graphSaved.getVertexList().back().setPointCloudRaw(currentScan);
    //correct last PCL
    slamToolsRos::correctPointCloudAtPos(graphSaved.getVertexList().back().getVertexNumber(), graphSaved,
                                         2 * M_PI / 200, 0, 2 * M_PI, true,transformationImu2PCL);


    //initialize hierachical slam
    std::deque<double> subgraphs{1, 3};
    graphSaved.initiallizeSubGraphs(subgraphs);

    for (int currentKeyFrame = startKeyFrame + 2;
         currentKeyFrame<keyFramesTimeStampesSorted.size()-1; currentKeyFrame++) {//keyFramesTimeStampesSorted.size()-1  || 50
        if (ros::ok()) {
//            if(currentKeyFrame == 32)
//            {
//                std::cout << "we are at 32" << std::endl;
//            }
            //for (int currentKeyFrame = 2; currentKeyFrame < 100; currentKeyFrame++) {
            *lastScan = *graphSaved.getVertexList().back().getPointCloudCorrected();
            lastTimeKeyFrame = timeCurrentGroundTruth;
            timeCurrentGroundTruth = keyFramesTimeStampesSorted[currentKeyFrame];

            pcl::io::loadPCDFile(
                    HOME + "/DataForTests/" + folderExperiment + "/pclKeyFrame" + std::to_string(currentKeyFrame) +
                    ".pcd",
                    *currentScan);
            //pcl::transformPointCloud(*currentScan, *currentScan, transformation90Degree);

            //forward calculate pose(relative)(with velocities) add edges+vertexes
            slamToolsRos::calculatePositionOverTime(angularVelocitySorted,
                                                    bodyVelocitySorted,
                                                    posDiffOverTimeEdges, lastTimeKeyFrame, timeCurrentGroundTruth,
                                                    0.0);// was 0.8
            //sort in posDiffOverTime and calculate vertices to be added
            appendEdgesToGraph(graphSaved, posDiffOverTimeEdges);
            graphSaved.getVertexList().back().setPointCloudRaw(currentScan);

            //re-map point cloud
            slamToolsRos::correctPointCloudAtPos(graphSaved.getVertexList().back().getVertexNumber(), graphSaved,
                                                 2 * M_PI / 200, 0, 2 * M_PI, true,transformationImu2PCL);

            //make scan matching with last scan
            Eigen::Matrix4d initialGuessTransformation =
                    graphSaved.getVertexList()[graphSaved.getVertexList().size() -
                                               9].getTransformation().inverse() *
                    graphSaved.getVertexList().back().getTransformation();//@todo understand if 9 is correct
            currentTransformation = scanRegistrationClass::generalizedIcpRegistration(
                    graphSaved.getVertexList().back().getPointCloudCorrected(), lastScan, Final,
                    fitnessScore,
                    initialGuessTransformation);
            if (debug) {
                *tmpCloudPlotOnly = *currentScan;
                pcl::transformPointCloud(*tmpCloudPlotOnly, *tmpCloudPlotOnly, transformationImu2PCL);
                debugPlotting(lastScan, Final, tmpCloudPlotOnly, graphSaved.getVertexList().back().getPointCloudCorrected(),
                              publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection,
                              publisherAfterCorrection);
            }
            std::cout << "current Fitness Score: " << sqrt(fitnessScore) << std::endl;
            //add edge for currentTransformation(registration)
            Eigen::Quaterniond qTMP(currentTransformation.block<3, 3>(0, 0));
            graphSaved.addEdge(graphSaved.getVertexList().size() - 10, graphSaved.getVertexList().size() - 1,
                               currentTransformation.block<3, 1>(0, 3), qTMP,
                               Eigen::Vector3d(sqrt(fitnessScore), sqrt(fitnessScore), 0),
                               0.25 * sqrt(fitnessScore),
                               graphSlamSaveStructure::POINT_CLOUD_USAGE);//@TODO still not sure about size

            //watch for loop closure
            slamToolsRos::detectLoopClosure(graphSaved, sigmaScaling, 3.0);//was 1.0
            slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                                publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                                groundTruthSorted, publisherMarkerArrayLoopClosures,
                                                timeCurrentGroundTruth);
            //optimization of graph (hierachical graph)
            //graphSaved.optimizeGraphWithSlamTopDown(false, 0.05);

            slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                                publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                                groundTruthSorted, publisherMarkerArrayLoopClosures,
                                                timeCurrentGroundTruth);
            std::vector<int> holdStill{0};
            graphSaved.optimizeGraphWithSlam(false, holdStill);

            graphSaved.calculateCovarianceInCloseProximity();
            //visualization of graph in ros

            slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                                publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                                groundTruthSorted, publisherMarkerArrayLoopClosures,
                                                timeCurrentGroundTruth);
            std::cout << "next: " << currentKeyFrame << std::endl;
        } else { exit(-1); }
    }
    std::vector<int> holdStill{0};
    graphSaved.optimizeGraphWithSlam(false, holdStill);
    slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                        publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                        groundTruthSorted, publisherMarkerArrayLoopClosures, timeCurrentGroundTruth);

    for (int i = 0; i < 10; i++) {
        //correct all point clouds
        slamToolsRos::correctEveryPointCloud(graphSaved, 2 * M_PI / 200, 0, 2 * M_PI, true,transformationImu2PCL);
        //recalculate the edges
        slamToolsRos::recalculatePCLEdges(graphSaved);



        //show
        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted, publisherMarkerArrayLoopClosures,
                                            timeCurrentGroundTruth);
        //optimize again
        graphSaved.optimizeGraphWithSlam(false, holdStill);
        //show
        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
                                            groundTruthSorted, publisherMarkerArrayLoopClosures,
                                            timeCurrentGroundTruth);
    }

    graphSaved.saveGraphJson(HOME + "/DataForTests/" + folderExperiment + "outputSlam.json");

    return (0);
}
