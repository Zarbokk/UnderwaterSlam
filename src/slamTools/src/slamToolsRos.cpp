//
// Created by tim on 26.03.21.
//

#include "slamToolsRos.h"
//#include "generalHelpfulTools.h"

void slamToolsRos::visualizeCurrentPoseGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                             ros::Publisher &publisherMarkerArray, double sigmaScaling,
                                             ros::Publisher &publisherPoseSlam, ros::Publisher &publisherLoopClosures) {

    nav_msgs::Path posOverTime;
    posOverTime.header.frame_id = "map_ned";
    Eigen::Matrix4d currentTransformation, completeTransformation;
    //pcl::PointCloud<pcl::PointXYZ> completeCloudWithPos;


    //std::vector<vertex> vertexList =;
    for (int i = 0; i < graphSaved.getVertexList()->size(); i++) {//skip the first pointCloud
        vertex vertexElement = graphSaved.getVertexList()->at(i);
        //pcl::PointCloud<pcl::PointXYZ> currentScanTransformed;
        //vertexElement.getTransformation();
        //completeTransformation = vertexElement.getTransformation();
//        if(vertexElement.getTypeOfVertex()==graphSlamSaveStructure::POINT_CLOUD_SAVED){
//            pcl::io::savePCDFileASCII("/home/jurobotics/DataForTests/savingRandomPCL/firstPCL.pcd",*vertexElement.getPointCloudCorrected());
//            pcl::io::savePCDFileASCII("/home/jurobotics/DataForTests/savingRandomPCL/secondPCL.pcd",currentScanTransformed);
//        }

        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = vertexElement.getPositionVertex().x();
        pos.pose.position.y = vertexElement.getPositionVertex().y();
        pos.pose.position.z = vertexElement.getPositionVertex().z();
        pos.pose.orientation.x = vertexElement.getRotationVertex().x();
        pos.pose.orientation.y = vertexElement.getRotationVertex().y();
        pos.pose.orientation.z = vertexElement.getRotationVertex().z();
        pos.pose.orientation.w = vertexElement.getRotationVertex().w();

        posOverTime.poses.push_back(pos);



    }

    visualization_msgs::MarkerArray markerArray;
    int k = 0;
    for (int i = 0; i < graphSaved.getVertexList()->size(); i = i + 10) {//skip the first pointCloud
        vertex vertexElement = graphSaved.getVertexList()->at(i);
        visualization_msgs::Marker currentMarker;
        currentMarker.pose.position.x = vertexElement.getPositionVertex().x();
        currentMarker.pose.position.y = vertexElement.getPositionVertex().y();
        currentMarker.pose.position.z = vertexElement.getPositionVertex().z();
        currentMarker.pose.orientation.w = 1;
        currentMarker.header.frame_id = "map_ned";
        currentMarker.scale.x = sigmaScaling *
                                vertexElement.getCovariancePosition()[0];
        currentMarker.scale.y = sigmaScaling *
                                vertexElement.getCovariancePosition()[1];
        currentMarker.scale.z = 0;
        currentMarker.color.r = 0;
        currentMarker.color.g = 1;
        currentMarker.color.b = 0;
        currentMarker.color.a = 0.05;
//currentMarker.lifetime.sec = 10;
        currentMarker.type = 2;
        currentMarker.id = k;
        k++;
        markerArray.markers.push_back(currentMarker);

    }
    publisherMarkerArray.publish(markerArray);
    publisherPath.publish(posOverTime);


    geometry_msgs::PoseStamped pos;
    pos.header.stamp = ros::Time(graphSaved.getVertexList()->back().getTimeStamp());
    pos.header.frame_id = "map_ned";
    pos.pose.position.x = graphSaved.getVertexList()->back().getPositionVertex().x();
    pos.pose.position.y = graphSaved.getVertexList()->back().getPositionVertex().y();
    pos.pose.position.z = graphSaved.getVertexList()->back().getPositionVertex().z();
    pos.pose.orientation.x = graphSaved.getVertexList()->back().getRotationVertex().x();
    pos.pose.orientation.y = graphSaved.getVertexList()->back().getRotationVertex().y();
    pos.pose.orientation.z = graphSaved.getVertexList()->back().getRotationVertex().z();
    pos.pose.orientation.w = graphSaved.getVertexList()->back().getRotationVertex().w();

    publisherPoseSlam.publish(pos);




    //create marker for evey loop closure
    visualization_msgs::MarkerArray markerArrowsArray;
    int j = 0;
    for (int i = 0; i < graphSaved.getEdgeList()->size(); i++) {

        if (graphSaved.getEdgeList()->at(i).getTypeOfEdge()==LOOP_CLOSURE) {//if its a loop closure then create arrow from vertex a to vertex b
            visualization_msgs::Marker currentMarker;
            //currentMarker.pose.position.x = pos.pose.position.x;
            //currentMarker.pose.position.y = pos.pose.position.y;
            //currentMarker.pose.position.z = pos.pose.position.z;
            //currentMarker.pose.orientation.w = 1;
            currentMarker.header.frame_id = "map_ned";
            currentMarker.scale.x = 0.1;
            currentMarker.scale.y = 0.3;
            currentMarker.scale.z = 0;
            currentMarker.color.r = 0;
            currentMarker.color.g = 0;
            currentMarker.color.b = 1;
            currentMarker.color.a = 0.8;
            //currentMarker.lifetime.sec = 10;
            geometry_msgs::Point startPoint;
            geometry_msgs::Point endPoint;

            startPoint.x = graphSaved.getVertexList()->at(graphSaved.getEdgeList()->at(i).getFromKey()).getPositionVertex()[0];
            startPoint.y = graphSaved.getVertexList()->at(graphSaved.getEdgeList()->at(i).getFromKey()).getPositionVertex()[1];
            startPoint.z = graphSaved.getVertexList()->at(graphSaved.getEdgeList()->at(i).getFromKey()).getPositionVertex()[2];

            endPoint.x = graphSaved.getVertexList()->at(graphSaved.getEdgeList()->at(i).getToKey()).getPositionVertex()[0];
            endPoint.y = graphSaved.getVertexList()->at(graphSaved.getEdgeList()->at(i).getToKey()).getPositionVertex()[1];
            endPoint.z = graphSaved.getVertexList()->at(graphSaved.getEdgeList()->at(i).getToKey()).getPositionVertex()[2];
            currentMarker.points.push_back(startPoint);
            currentMarker.points.push_back(endPoint);
            currentMarker.type = 0;
            currentMarker.id = j;
            j++;
            markerArrowsArray.markers.push_back(currentMarker);
        }
    }
    publisherLoopClosures.publish(markerArrowsArray);

}

std::vector<measurement> slamToolsRos::parseCSVFile(std::istream &stream) {
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
        tmpMeas.keyframe = std::stoi(result[0]);
        tmpMeas.x = std::stof(result[1]);
        tmpMeas.y = std::stof(result[2]);
        tmpMeas.z = std::stof(result[3]);
        tmpMeas.timeStamp = std::stof(result[4]);
        returnVector.push_back(tmpMeas);
    }
    return returnVector;
}

std::vector<std::vector<measurement>> slamToolsRos::sortToKeyframe(std::vector<measurement> &input) {
    int currentKeyframe = input[0].keyframe;
    std::vector<std::vector<measurement>> output;
    std::vector<measurement> tmp1;
    output.push_back(tmp1);
    for (auto currentMeasurement: input) {
        if (currentMeasurement.keyframe != currentKeyframe) {//new keyframe reached
            std::vector<measurement> tmp;
            currentKeyframe = currentMeasurement.keyframe;
            output.push_back(tmp);
            output[currentKeyframe].push_back(currentMeasurement);
        } else {
            output[currentKeyframe].push_back(currentMeasurement);
        }
    }
    return output;
}


std::vector<double> slamToolsRos::linspace(double start_in, double end_in, int num_in) {
    if (num_in < 0) {
        std::cout << "number of linspace negative" << std::endl;
        exit(-1);
    }
    std::vector<double> linspaced;

    double start = start_in;
    double end = end_in;
    auto num = (double) num_in;

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);//stepSize

    for (int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

void slamToolsRos::calculatePositionOverTime(std::deque<ImuData> &angularVelocityList,
                                             std::deque<DvlData> &bodyVelocityList,
                                             std::vector<edge> &posOverTimeEdge,
                                             double lastScanTimeStamp,
                                             double currentScanTimeStamp,
                                             double noiseAddedStdDiv, int numberOfEdges) {//last then current
    posOverTimeEdge.clear();
    std::vector<double> timeSteps = slamToolsRos::linspace(lastScanTimeStamp, currentScanTimeStamp,
                                                           numberOfEdges);// could be changed this is the number of pos+1 between the scans(10th is the scan itself)
    std::vector<double> angularX;
    std::vector<double> angularY;
    std::vector<double> angularZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate angular between lastScanTimeStamp and currentScanTimeStamp
        std::vector<ImuData> measurementsOfInterest;
        for (int j = 0; j < angularVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= angularVelocityList[j].timeStamp &&
                timeSteps[i] > angularVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(angularVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;
        if (measurementsOfInterest.size() > 1) {
            for (int j = 0; j < measurementsOfInterest.size(); j++) {
                if (j == measurementsOfInterest.size() - 1) {
                    integratorX +=
                            measurementsOfInterest[j].wx * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorY +=
                            measurementsOfInterest[j].wy * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorZ +=
                            measurementsOfInterest[j].wz * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                } else {
                    if (j == 0) {
                        integratorX +=
                                measurementsOfInterest[j].wx * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorY +=
                                measurementsOfInterest[j].wy * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorZ +=
                                measurementsOfInterest[j].wz * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    } else {
                        integratorX += (measurementsOfInterest[j].wx + measurementsOfInterest[j - 1].wx) / 2 *
                                       (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorY += (measurementsOfInterest[j].wy + measurementsOfInterest[j - 1].wy) / 2 *
                                       (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorZ += (measurementsOfInterest[j].wz + measurementsOfInterest[j - 1].wz) / 2 *
                                       (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    }
                }
            }
        } else {//only one or zero velocity measurement exists
            integratorX = measurementsOfInterest[0].wx * (timeSteps[i] - timeSteps[i - 1]);
            integratorY = measurementsOfInterest[0].wy * (timeSteps[i] - timeSteps[i - 1]);
            integratorZ = measurementsOfInterest[0].wz * (timeSteps[i] - timeSteps[i - 1]);
        }
        angularX.push_back(integratorX);
        angularY.push_back(integratorY);
        angularZ.push_back(integratorZ);
    }

    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, noiseAddedStdDiv);
    std::vector<double> linearX;
    std::vector<double> linearY;
    std::vector<double> linearZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastScanTimeStamp and currentScanTimeStamp
        std::vector<DvlData> measurementsOfInterest;
        for (int j = 0; j < bodyVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= bodyVelocityList[j].timeStamp &&
                timeSteps[i] > bodyVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(bodyVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;

        if (measurementsOfInterest.size() > 1) {
            for (int j = 0; j < measurementsOfInterest.size(); j++) {
                if (j == measurementsOfInterest.size() - 1) {
                    integratorX += (dist(generator) + measurementsOfInterest[j].vx) *
                                   (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorY += (dist(generator) + measurementsOfInterest[j].vy) *
                                   (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                    integratorZ += (dist(generator) + measurementsOfInterest[j].vz) *
                                   (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                } else {
                    if (j == 0) {
                        integratorX +=
                                (dist(generator) + measurementsOfInterest[j].vx) *
                                (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorY +=
                                (dist(generator) + measurementsOfInterest[j].vy) *
                                (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                        integratorZ +=
                                (dist(generator) + measurementsOfInterest[j].vz) *
                                (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    } else {
                        integratorX +=
                                (dist(generator) +
                                 (measurementsOfInterest[j].vx + measurementsOfInterest[j - 1].vx) / 2) *
                                (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorY +=
                                (dist(generator) +
                                 (measurementsOfInterest[j].vy + measurementsOfInterest[j - 1].vy) / 2) *
                                (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                        integratorZ +=
                                (dist(generator) +
                                 (measurementsOfInterest[j].vz + measurementsOfInterest[j - 1].vz) / 2) *
                                (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    }
                }
            }
        } else {//only one velocity measurement exists
            integratorX = measurementsOfInterest[0].vx * (timeSteps[i] - timeSteps[i - 1]);
            integratorY = measurementsOfInterest[0].vy * (timeSteps[i] - timeSteps[i - 1]);
            integratorZ = measurementsOfInterest[0].vz * (timeSteps[i] - timeSteps[i - 1]);
        }
        linearX.push_back(integratorX);
        linearY.push_back(integratorY);
        linearZ.push_back(integratorZ);
    }



    //std::vector<vertex> &posOverTimeVertex,
    //std::vector<edge> &posOverTimeEdge,

    for (int i = 0; i < timeSteps.size() - 1; i++) {
        Eigen::Vector3d posDiff(linearX[i], linearY[i], 0);//linear Z missing
        Eigen::Quaterniond rotDiff = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(angularZ[i],
                                                         Eigen::Vector3d::UnitZ());
        Eigen::Vector3d covariancePos(0, 0, 0);
        edge currentEdge(0, 0, posDiff, rotDiff, covariancePos, 0, 3,
                         INTEGRATED_POSE);
        //currentEdge.setTimeStamp(timeSteps[i + 1]);
        posOverTimeEdge.push_back(currentEdge);
    }

}

double slamToolsRos::createVoxelOfGraph(double voxelData[], int indexStart,
                                        Eigen::Matrix4d transformationInTheEndOfCalculation,
                                        int numberOfPoints, graphSlamSaveStructure &usedGraph,
                                        double ignoreDistanceToRobot, double dimensionOfVoxelData) {
    int *voxelDataIndex;
    voxelDataIndex = (int *) malloc(sizeof(int) * numberOfPoints * numberOfPoints);
    //set zero voxel and index
    for (int i = 0; i < numberOfPoints * numberOfPoints; i++) {
        voxelDataIndex[i] = 0;
        voxelData[i] = 0;
    }


    int i = 0;
    do {
        //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.


        //get position of current intensityRay
        Eigen::Matrix4d transformationOfIntensityRay =
                usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                usedGraph.getVertexList()->at(indexStart - i).getTransformation();

        //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
        Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                         usedGraph.getVertexList()->at(
                                                                                                                 indexStart -
                                                                                                                 i).getIntensities().angle);

        int ignoreDistance = (int) (ignoreDistanceToRobot /
                                    (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                     ((double) usedGraph.getVertexList()->at(
                                             indexStart - i).getIntensities().intensities.size())));


        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            double distanceOfIntensity =
                    j / ((double) usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities.size()) *
                    ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);

            int incrementOfScan = usedGraph.getVertexList()->at(indexStart - i).getIntensities().increment;
            for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);
                double rotationOfPoint = l / 400.0;
                Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                            rotationOfPoint);
                positionOfIntensity = rotationForBetterView * positionOfIntensity;

                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                int indexX =
                        (int) (positionOfIntensity.x() / (dimensionOfVoxelData / 2) * numberOfPoints /
                               2) +
                        numberOfPoints / 2;
                int indexY =
                        (int) (positionOfIntensity.y() / (dimensionOfVoxelData / 2) * numberOfPoints /
                               2) +
                        numberOfPoints / 2;


                if (indexX < numberOfPoints && indexY < numberOfPoints && indexY >= 0 &&
                    indexX >= 0) {
                    //                    std::cout << indexX << " " << indexY << std::endl;
                    //if index fits inside of our data, add that data. Else Ignore
                    voxelDataIndex[indexY + numberOfPoints * indexX] =
                            voxelDataIndex[indexY + numberOfPoints * indexX] + 1;
                    //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                    voxelData[indexY + numberOfPoints * indexX] =
                            voxelData[indexY + numberOfPoints * indexX] +
                            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j];
                    //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                    //                    std::cout << "random: " << std::endl;
                }
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    double maximumOfVoxelData = 0;
    for (i = 0; i < numberOfPoints * numberOfPoints; i++) {
        if (voxelDataIndex[i] > 0) {
            voxelData[i] = voxelData[i] / voxelDataIndex[i];
            if (maximumOfVoxelData < voxelData[i]) {
                maximumOfVoxelData = voxelData[i];
            }
            //std::cout << voxelData[i] << std::endl;

        }
    }// @TODO calculate the maximum and normalize "somehow"




    free(voxelDataIndex);
    return maximumOfVoxelData;
}

pcl::PointCloud<pcl::PointXYZ> slamToolsRos::createPCLFromGraphOneValue(int indexStart,
                                                                        Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                                        graphSlamSaveStructure &usedGraph,
                                                                        double ignoreDistanceToRobo,
                                                                        double thresholdFactorPoint) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    //create array with all intencities.
    // Calculate maximum of intensities.
    // only use maximum of 10% of max value as points
    int i = 0;
    double maximumIntensity = 0;

    int ignoreDistance = (int) (ignoreDistanceToRobo /
                                (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                 ((double) usedGraph.getVertexList()->at(
                                         indexStart - i).getIntensities().intensities.size())));

    do {
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                maximumIntensity) {
                maximumIntensity = usedGraph.getVertexList()->at(
                        indexStart - i).getIntensities().intensities[j];
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);

    double thresholdIntensityScan = maximumIntensity * thresholdFactorPoint;//maximum intensity of 0.9



    i = 0;
    do {
        //find max Position
        int maxPosition = ignoreDistance;
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition]) {
                maxPosition = j;
            }
        }
        if (maxPosition > ignoreDistance &&
            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition] >
            thresholdIntensityScan) {
            Eigen::Matrix4d transformationOfIntensityRay =
                    usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                    usedGraph.getVertexList()->at(indexStart - i).getTransformation();

            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             usedGraph.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);

            double distanceOfIntensity =
                    maxPosition / ((double) usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities.size()) *
                    ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
            Eigen::Vector4d positionOfIntensity(
                    distanceOfIntensity,
                    0,
                    0,
                    1);

            positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                  rotationOfSonarAngleMatrix * positionOfIntensity;
            //create point for PCL
            pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                   (float) positionOfIntensity[1],
                                   (float) positionOfIntensity[2]);
            scan.push_back(tmpPoint);
        }


        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    return scan;
}


pcl::PointCloud<pcl::PointXYZ> slamToolsRos::createPCLFromGraphOnlyThreshold(int indexStart,
                                                                             Eigen::Matrix4d transformationInTheEndOfCalculation,
                                                                             graphSlamSaveStructure &usedGraph,
                                                                             double ignoreDistanceToRobo,
                                                                             double thresholdFactorPoint) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    //create array with all intencities.
    // Calculate maximum of intensities.


    double maximumIntensity = 0;
    int i = 0;

    int ignoreDistance = (int) (ignoreDistanceToRobo /
                                (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                 ((double) usedGraph.getVertexList()->at(
                                         indexStart - i).getIntensities().intensities.size())));


    do {
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                maximumIntensity) {
                maximumIntensity = usedGraph.getVertexList()->at(
                        indexStart - i).getIntensities().intensities[j];
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);

    double thresholdIntensityScan = maximumIntensity * thresholdFactorPoint;//maximum intensity of 0.9



    i = 0;
    do {
        Eigen::Matrix4d transformationOfIntensityRay =
                usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                usedGraph.getVertexList()->at(indexStart - i).getTransformation();

        //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
        Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                         usedGraph.getVertexList()->at(
                                                                                                                 indexStart -
                                                                                                                 i).getIntensities().angle);
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                thresholdIntensityScan) {
                double distanceOfIntensity =
                        j / ((double) usedGraph.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);
                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //create point for PCL
                pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                       (float) positionOfIntensity[1],
                                       (float) positionOfIntensity[2]);
                scan.push_back(tmpPoint);
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    return scan;
}


bool slamToolsRos::getNodes(ros::V_string &nodes) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
        return false;
    }

    ros::S_string node_set;
    for (int i = 0; i < payload.size(); ++i) {
        for (int j = 0; j < payload[i].size(); ++j) {
            XmlRpc::XmlRpcValue val = payload[i][j][1];
            for (int k = 0; k < val.size(); ++k) {
                std::string name = payload[i][j][1][k];
                node_set.insert(name);
            }
        }
    }

    nodes.insert(nodes.end(), node_set.begin(), node_set.end());

    return true;
}



//void slamToolsRos::appendEdgesToGraph(graphSlamSaveStructure &currentGraph,
//                                      std::deque<edge> &listOfEdges, double noiseVelocityIntigration,
//                                      double scalingAngle, double maxTimeOptimization,
//                                      int maximumNumberOfAddedEdges) {// adds edges to the graph and create vertex, which are represented by edges
//    std::deque<edge> listOfEdgesForForLoop;
//    //add only a max number of edges
//    if (listOfEdges.size() > maximumNumberOfAddedEdges) {
//        double startTime = currentGraph.getVertexList()->back().getTimeStamp();
//        double endTime = listOfEdges.back().getTimeStamp();
//        //ignore the first element(since its already a timestep) can be changed for not to be the case
//        std::vector<double> timestampsForEdge = slamToolsRos::linspace(startTime, endTime,
//                                                                       maximumNumberOfAddedEdges + 1);
//        int currentEdgeIndex = 0;
//        for (int i = 1;
//             i < timestampsForEdge.size(); i++) {//@TODO add interpolation(not done currently) but roughly correct
//
//            while(listOfEdges[currentEdgeIndex].getTimeStamp()<timestampsForEdge[i-1]){
//                currentEdgeIndex++;
//            }
//            //initialize transformation of 0
//            Eigen::Matrix4d currentTransformation = Eigen::Matrix4d::Identity();
//            while (listOfEdges[currentEdgeIndex].getTimeStamp() < timestampsForEdge[i]) {
//                //add up transformations from zero
//                currentTransformation = currentTransformation * listOfEdges[currentEdgeIndex].getTransformation();
//                currentEdgeIndex++;
//            }
//
//            // create an edge and add it to a list that is then added to the graph.
//
//            Eigen::Quaterniond qTMP(currentTransformation.block<3, 3>(0, 0));
//            Eigen::Vector3d covariancePos(0, 0, 0);
//            edge currentEdge(0, 0, currentTransformation.block<3, 1>(0, 3), qTMP, covariancePos, 0, 3,
//                             graphSlamSaveStructure::INTEGRATED_POSE);
//            currentEdge.setTimeStamp(timestampsForEdge[i]);
//            listOfEdgesForForLoop.push_back(currentEdge);
//
//        }
//
//    } else {
//        listOfEdgesForForLoop = listOfEdges;
//    }
//
//
//    int i = 1;
//    for (auto &currentEdge: listOfEdgesForForLoop) {
//        vertex lastVertex = currentGraph.getVertexList()->back();
//        Eigen::Matrix4d tmpTransformation = lastVertex.getTransformation();
//        tmpTransformation = tmpTransformation * currentEdge.getTransformation();
//        Eigen::Vector3d pos = tmpTransformation.block<3, 1>(0, 3);
//        Eigen::Matrix3d rotM = tmpTransformation.block<3, 3>(0, 0);
//        Eigen::Quaterniond rot(rotM);
//
//        currentGraph.addVertex(lastVertex.getVertexNumber() + 1, pos, rot, lastVertex.getCovariancePosition(),
//                               lastVertex.getCovarianceQuaternion(), currentEdge.getTimeStamp(),
//                               graphSlamSaveStructure::INTEGRATED_POSE);
//        currentGraph.addEdge(lastVertex.getVertexNumber(), lastVertex.getVertexNumber() + 1,
//                             currentEdge.getPositionDifference(), currentEdge.getRotationDifference(),
//                             Eigen::Vector3d(noiseVelocityIntigration, noiseVelocityIntigration, 0),
//                             scalingAngle * noiseVelocityIntigration, graphSlamSaveStructure::INTEGRATED_POSE,
//                             maxTimeOptimization);
////        graphSaved.getVertexList()->back().setTypeOfVertex(
////                graphSlamSaveStructure::INTEGRATED_POSE);//1 for vertex defined by dead reckoning
//        i++;
//    }
//}
//
//
//
//void
//slamToolsRos::correctPointCloudAtPos(int positionToCorrect, graphSlamSaveStructure &currentGraph,
//                                     double beginAngle,
//                                     double endAngle, bool reverseScanDirection,
//                                     Eigen::Matrix4d transformationPosData2PclCoord) {
//    // get index of the last vertex
//    int lastIndex;
//    int j = 1;
//    while (true) {
//        if (currentGraph.getVertexList()->at(positionToCorrect - j).getTypeOfVertex() ==
//                    POINT_CLOUD_SAVED ||
//            currentGraph.getVertexList()->at(positionToCorrect - j).getTypeOfVertex() ==
//            FIRST_ENTRY) {
//            lastIndex = positionToCorrect - j;
//            break;
//        }
//        j++;
//    }
//
//
//    std::vector<edge> posDiff;
//    int i = 0;
//    while (lastIndex + i != positionToCorrect) {
//        Eigen::Matrix4d fromTransformation = currentGraph.getVertexList()->at(lastIndex + i).getTransformation();//from
//        Eigen::Matrix4d toTransformation = currentGraph.getVertexList()->at(lastIndex + i + 1).getTransformation();//to
//        Eigen::Matrix4d transofrmationCurrentEdge = fromTransformation.inverse() * toTransformation;
//
//        Eigen::Quaterniond qTMP(transofrmationCurrentEdge.block<3, 3>(0, 0));
//        edge currentEdge(0, 0, transofrmationCurrentEdge.block<3, 1>(0, 3), qTMP, Eigen::Vector3d(0, 0, 0), 0, 3,
//                         graphSlamSaveStructure::INTEGRATED_POSE);
//        currentEdge.setTimeStamp(currentGraph.getVertexList()->at(lastIndex + i + 1).getTimeStamp());
//        posDiff.push_back(currentEdge);
//        i++;
//    }
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScan = currentGraph.getVertexList()->at(positionToCorrect).getPointCloudRaw();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr correctedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    *correctedPointCloud = *cloudScan;//copy raw in corrected
//    slamToolsRos::correctPointCloudByPosition(correctedPointCloud, posDiff,
//                                              currentGraph.getVertexList()->at(lastIndex).getTimeStamp(), beginAngle,
//                                              endAngle, reverseScanDirection,
//                                              transformationPosData2PclCoord);
//    currentGraph.getVertexList()->at(positionToCorrect).setPointCloudCorrected(correctedPointCloud);
//    currentGraph.getVertexByIndex(positionToCorrect)->setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_SAVED);
//}