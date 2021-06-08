//
// Created by tim on 26.03.21.
//

#include "slamToolsRos.h"

void slamToolsRos::visualizeCurrentGraph(graphSlamSaveStructure &graphSaved, ros::Publisher &publisherPath,
                                         ros::Publisher &publisherCloud, ros::Publisher &publisherMarkerArray,
                                         double sigmaScaling, ros::Publisher &publisherPathGT,
                                         std::vector<std::vector<measurement>> &groundTruthSorted,
                                         ros::Publisher &publisherMarkerArrayLoopClosures) {

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
        pcl::transformPointCloud(*vertexElement.getPointCloudCorrected(), currentScanTransformed,
                                 completeTransformation);
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
        currentMarker.color.a = 0.05;
        //currentMarker.lifetime.sec = 10;

        currentMarker.type = 2;
        currentMarker.id = i;
        i++;

        markerArray.markers.push_back(currentMarker);

    }
    publisherMarkerArray.publish(markerArray);
    publisherPath.publish(posOverTime);

    sensor_msgs::PointCloud2 cloudMsg2;
    pcl::toROSMsg(completeCloudWithPos, cloudMsg2);
    cloudMsg2.header.frame_id = "map_ned";
    publisherCloud.publish(cloudMsg2);


    int numberOfKeyframes = (int) (graphSaved.getVertexList().size() / 9) + 1;
    //calculate path GT
    nav_msgs::Path posOverTimeGT;
    posOverTimeGT.header.frame_id = "map_ned";
    for (int i = 0; i < numberOfKeyframes; i++) {
        for (auto &posList:groundTruthSorted[i]) {
            geometry_msgs::PoseStamped pos;
            pos.pose.position.x = posList.y - groundTruthSorted[0][0].y;
            pos.pose.position.y = posList.x - groundTruthSorted[0][0].x;
            pos.pose.position.z = 0;
            pos.pose.orientation.x = 0;
            pos.pose.orientation.y = 0;
            pos.pose.orientation.z = 0;
            pos.pose.orientation.w = 1;
            posOverTimeGT.poses.push_back(pos);
        }
    }
    publisherPathGT.publish(posOverTimeGT);

    //create marker for evey loop closure
    visualization_msgs::MarkerArray markerArrowsArray;
    int j = 0;
    for (int i = 0; i < graphSaved.getEdgeList()->size(); i++) {
        edge currentEdgeOfInterest = graphSaved.getEdgeList()->data()[i];
        if (abs(currentEdgeOfInterest.getFromVertex() - currentEdgeOfInterest.getToVertex()) >
            15) {//if its a loop closure then create arrow from vertex a to vertex b
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

            startPoint.x = graphSaved.getVertexList()[currentEdgeOfInterest.getFromVertex()].getPositionVertex()[0];
            startPoint.y = graphSaved.getVertexList()[currentEdgeOfInterest.getFromVertex()].getPositionVertex()[1];
            startPoint.z = graphSaved.getVertexList()[currentEdgeOfInterest.getFromVertex()].getPositionVertex()[2];

            endPoint.x = graphSaved.getVertexList()[currentEdgeOfInterest.getToVertex()].getPositionVertex()[0];
            endPoint.y = graphSaved.getVertexList()[currentEdgeOfInterest.getToVertex()].getPositionVertex()[1];
            endPoint.z = graphSaved.getVertexList()[currentEdgeOfInterest.getToVertex()].getPositionVertex()[2];
            currentMarker.points.push_back(startPoint);
            currentMarker.points.push_back(endPoint);
            currentMarker.type = 0;
            currentMarker.id = j;
            j++;
            markerArrowsArray.markers.push_back(currentMarker);
        }
    }
    publisherMarkerArrayLoopClosures.publish(markerArrowsArray);


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
    for (auto currentMeasurement:input) {
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

void
slamToolsRos::correctPointCloudAtPos(int positionToCorrect, graphSlamSaveStructure &currentGraph) {
    // get index of the last vertex
    int lastIndex;
    int j = 1;
    while (true) {
        if (currentGraph.getVertexList()[positionToCorrect - j].getTypeOfVertex() ==
            graphSlamSaveStructure::POINT_CLOUD_USAGE ||
            currentGraph.getVertexList()[positionToCorrect - j].getTypeOfVertex() ==
            graphSlamSaveStructure::FIRST_ENTRY) {
            lastIndex = positionToCorrect - j;
            break;
        }
        j++;
    }


    std::vector<edge> posDiff;
    int i = 0;
    while (lastIndex + i != positionToCorrect) {
        Eigen::Matrix4d fromTransformation = currentGraph.getVertexList()[lastIndex + i].getTransformation();//from
        Eigen::Matrix4d toTransformation = currentGraph.getVertexList()[lastIndex + i + 1].getTransformation();//to
        Eigen::Matrix4d transofrmationCurrentEdge = fromTransformation.inverse() * toTransformation;

        Eigen::Quaterniond qTMP(transofrmationCurrentEdge.block<3, 3>(0, 0));
        edge currentEdge(0, 0, transofrmationCurrentEdge.block<3, 1>(0, 3), qTMP, Eigen::Vector3d(0, 0, 0), 0, 3,
                         graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        currentEdge.setTimeStamp(currentGraph.getVertexList()[lastIndex + i + 1].getTimeStamp());
        posDiff.push_back(currentEdge);
        i++;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScan = currentGraph.getVertexList()[positionToCorrect].getPointCloudRaw();
    pcl::PointCloud<pcl::PointXYZ>::Ptr correctedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *correctedPointCloud = *cloudScan;
    slamToolsRos::correctPointCloudByPosition(correctedPointCloud, posDiff, currentGraph.getVertexList()[lastIndex].getTimeStamp());
    currentGraph.getVertexList()[positionToCorrect].setPointCloudCorrected(correctedPointCloud);
    currentGraph.getVertexByIndex(positionToCorrect)->setTypeOfVertex(graphSlamSaveStructure::POINT_CLOUD_USAGE);
}

void
slamToolsRos::correctEveryPointCloud(graphSlamSaveStructure &currentGraph) {

    for (int i = 1; i < currentGraph.getVertexList().size(); i++) {
        if (currentGraph.getVertexList()[i].getTypeOfVertex() == graphSlamSaveStructure::POINT_CLOUD_USAGE) {
            slamToolsRos::correctPointCloudAtPos(i, currentGraph);
        }
    }
}

void
slamToolsRos::recalculatePCLEdges(graphSlamSaveStructure &currentGraph) {

    for (int i = 1; i < currentGraph.getEdgeList()->size(); i++) {
        if (currentGraph.getEdgeList()->data()[i].getTypeOfEdge() == graphSlamSaveStructure::POINT_CLOUD_USAGE) {
            //recalculate edge
            pcl::PointCloud<pcl::PointXYZ>::Ptr Final(
                    new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan(
                    new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan(
                    new pcl::PointCloud<pcl::PointXYZ>);

            double fitnessScore;
            Eigen::Matrix4d initialGuessTransformation =
                    currentGraph.getVertexList()[currentGraph.getEdgeList()->data()[i].getFromVertex()].getTransformation().inverse() *
                    currentGraph.getVertexList()[currentGraph.getEdgeList()->data()[i].getToVertex()].getTransformation();//@todo understand if 9 is correct
            Eigen::Matrix4d currentTransformation;

            *lastScan = *currentGraph.getVertexList()[currentGraph.getEdgeList()->data()[i].getFromVertex()].getPointCloudCorrected();

            *currentScan = *currentGraph.getVertexList()[currentGraph.getEdgeList()->data()[i].getToVertex()].getPointCloudCorrected();

            currentTransformation = scanRegistrationClass::generalizedIcpRegistration(currentScan, lastScan, Final,
                                                                                      fitnessScore,
                                                                                      initialGuessTransformation);

            Eigen::Quaterniond qTMP(currentTransformation.block<3, 3>(0, 0));

            currentGraph.getEdgeList()->data()[i].setCovariancePosition(
                    Eigen::Vector3d(sqrt(fitnessScore), sqrt(fitnessScore), 0));
            currentGraph.getEdgeList()->data()[i].setCovarianceQuaternion(0.25*sqrt(fitnessScore));
            currentGraph.getEdgeList()->data()[i].setPositionDifference(currentTransformation.block<3, 1>(0, 3));
            currentGraph.getEdgeList()->data()[i].setRotationDifference(qTMP);
        }
    }

}

void
slamToolsRos::correctPointCloudByPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScan, std::vector<edge> &posDiff,
                                          double timeStampBeginning) {
    //this is assuming we are linear in time
    //resulting scan should be at end position means: point 0 is transformed by diff of pos 0 to n

    struct pclAngle {
        pcl::PointXYZ pointXyz;
        double angle{};
    };

    //calculate angle of every point
    std::vector<pclAngle> vectorOfPointsAngle;
    for (int i = 0; i <
                    cloudScan->size(); i++) {//@TODO has to be corrected Last Point is always a positive angle and not under 2 pi
        double currentAngle = fmod(2 * (double) M_PI + atan2(cloudScan->points[i].y, cloudScan->points[i].x), 2 * M_PI);
        pclAngle tmpPCL;
        tmpPCL.pointXyz = cloudScan->points[i];
        tmpPCL.angle = currentAngle;
        vectorOfPointsAngle.push_back(tmpPCL);
    }
    //sort from 0 to 2pi
    std::sort(vectorOfPointsAngle.begin(), vectorOfPointsAngle.end(),
              [](const auto &i, const auto &j) { return i.angle > j.angle; });

    pclAngle tmpSaving = vectorOfPointsAngle[0];
    vectorOfPointsAngle.erase(vectorOfPointsAngle.begin());//@TODO shitty solution i want a different one
    vectorOfPointsAngle.push_back(tmpSaving);
    //calculate number of points transformations defined by edge list
    std::vector<Eigen::Matrix4d> listOfTransormations;
    double startTime = timeStampBeginning;
    double endTime = posDiff.back().getTimeStamp();
    std::vector<double> timeStepsForCorrection = slamToolsRos::linspace(startTime, endTime, vectorOfPointsAngle.size());
    for (int i = 0; i < timeStepsForCorrection.size(); i++) {
        Eigen::Matrix4d currentTransformation;
        currentTransformation << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        int j = (int) posDiff.size() - 1;
        while (timeStepsForCorrection[i] < posDiff[j].getTimeStamp()) {
            j--;
            if (j == 0) {
                break;
            }
        }
        if (j == 0) {
            if (timeStepsForCorrection[i] == startTime) {
                //add everything
                for (int k = j; k < posDiff.size(); k++) {
                    //calculate transformation of edge
                    currentTransformation *= posDiff[k].getTransformation();
                    //add to currentTransformation
                }
            } else {
                //interpolate from timeStepsForCorrection[i] to posDiff[j]

                double fromTimeStep = timeStepsForCorrection[i];
                double toTimeStep = posDiff[j].getTimeStamp();
                //interpolate posDiff[j] from to
                Eigen::Matrix4d firstTransformation = posDiff[j].getTransformation();
                double interpolationFactor =
                        (toTimeStep - fromTimeStep) / (posDiff[j].getTimeStamp() - startTime);
                firstTransformation.block<3, 1>(0, 3) = interpolationFactor * firstTransformation.block<3, 1>(0, 3);
                Eigen::Matrix3d rotationMatrix = firstTransformation.block<3, 3>(0, 0);
                Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
                if(abs(eulerAngles[2])>2){
                    std::cout << "big rotation" << std::endl;
                }
                eulerAngles = interpolationFactor * eulerAngles;

                Eigen::Matrix3d tmp;
                tmp = Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ());
                firstTransformation.block<3, 3>(0, 0) = tmp;
                currentTransformation *= firstTransformation;
                //add rest
                for (int k = j + 1; k < posDiff.size(); k++) {
                    //calculate transformation of edge
                    currentTransformation *= posDiff[k].getTransformation();
                    //add to currentTransformation
                }
            }
        } else {
            //interpolate first step
            double fromTimeStep = timeStepsForCorrection[i];
            double toTimeStep = posDiff[j].getTimeStamp();
            //interpolate posDiff[j] from to
            Eigen::Matrix4d firstTransformation = posDiff[j].getTransformation();
            double interpolationFactor =
                    (toTimeStep - fromTimeStep) / (posDiff[j].getTimeStamp() - posDiff[j - 1].getTimeStamp());
            firstTransformation.block<3, 1>(0, 3) = interpolationFactor * firstTransformation.block<3, 1>(0, 3);
            Eigen::Matrix3d rotationMatrix = firstTransformation.block<3, 3>(0, 0);
            Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);
            if(abs(eulerAngles[2])>2){
                std::cout << "big rotation" << std::endl;
            }
            eulerAngles = interpolationFactor * eulerAngles;

            Eigen::Matrix3d tmp;
            tmp = Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ());
            firstTransformation.block<3, 3>(0, 0) = tmp;
            currentTransformation *= firstTransformation;
            for (int k = j + 1; k < posDiff.size(); k++) {
                currentTransformation *= posDiff[k].getTransformation();
            }

        }
        Eigen::Matrix3d tmp2;
        tmp2 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(
                      +0.0, Eigen::Vector3d::UnitZ());
        currentTransformation.block<3, 3>(0, 0) *= tmp2;//this is an correction factor, because PX4 thinks it drives forward, while it is twisted
        listOfTransormations.push_back(currentTransformation);
    }


    //correct points by transformation
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < vectorOfPointsAngle.size(); i++) {
        pcl::PointXYZ point;
        point = vectorOfPointsAngle[i].pointXyz;
        Eigen::Vector4d currentPoint(point.x, point.y, point.z, 1);
        currentPoint = listOfTransormations[i].inverse() * currentPoint;
        point.x = currentPoint.x();
        point.y = currentPoint.y();
        point.z = currentPoint.z();
        //fill point
        cloud.push_back(point);
    }
    *cloudScan = cloud;
}

std::vector<double> slamToolsRos::linspace(double start_in, double end_in, int num_in) {

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

void slamToolsRos::calculatePositionOverTime(std::vector<measurement> &angularVelocityList,
                                             std::vector<measurement> &bodyVelocityList,
                                             std::vector<edge> &posOverTimeEdge,
                                             double lastTimeStamp,
                                             double currentTimeStamp,
                                             double stdDev) {//last then current
    posOverTimeEdge.clear();
    std::vector<double> timeSteps = slamToolsRos::linspace(lastTimeStamp, currentTimeStamp, 10);
    std::vector<double> angularX;
    std::vector<double> angularY;
    std::vector<double> angularZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastTimeKeyFrame and timeCurrentGroundTruth
        std::vector<measurement> measurementsOfInterest;
        for (int j = 0; j < angularVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= angularVelocityList[j].timeStamp &&
                timeSteps[i] > angularVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(angularVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;
        for (int j = 0; j < measurementsOfInterest.size(); j++) {
            if (j == measurementsOfInterest.size() - 1) {
                integratorX += measurementsOfInterest[j].x * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                integratorY += measurementsOfInterest[j].y * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                integratorZ += measurementsOfInterest[j].z * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
            } else {
                if (j == 0) {
                    integratorX +=
                            measurementsOfInterest[j].x * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    integratorY +=
                            measurementsOfInterest[j].y * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    integratorZ +=
                            measurementsOfInterest[j].z * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                } else {
                    integratorX += (measurementsOfInterest[j].x + measurementsOfInterest[j - 1].x) / 2 *
                                   (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    integratorY += (measurementsOfInterest[j].y + measurementsOfInterest[j - 1].y) / 2 *
                                   (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    integratorZ += (measurementsOfInterest[j].z + measurementsOfInterest[j - 1].z) / 2 *
                                   (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                }
            }
        }
        angularX.push_back(integratorX);
        angularY.push_back(integratorY);
        angularZ.push_back(integratorZ);
    }

    std::default_random_engine generator;
    std::normal_distribution<double> dist(0, stdDev);
    std::vector<double> linearX;
    std::vector<double> linearY;
    std::vector<double> linearZ;
    for (int i = 1;
         i < timeSteps.size(); i++) {//calculate that between lastTimeKeyFrame and timeCurrentGroundTruth
        std::vector<measurement> measurementsOfInterest;
        for (int j = 0; j < bodyVelocityList.size(); j++) {
            if (timeSteps[i - 1] <= bodyVelocityList[j].timeStamp &&
                timeSteps[i] > bodyVelocityList[j].timeStamp) {
                measurementsOfInterest.push_back(bodyVelocityList[j]);
            }
        }

        double integratorX = 0;
        double integratorY = 0;
        double integratorZ = 0;
        for (int j = 0; j < measurementsOfInterest.size(); j++) {
            if (j == measurementsOfInterest.size() - 1) {
                integratorX += (dist(generator)+measurementsOfInterest[j].x) * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                integratorY += (dist(generator)+measurementsOfInterest[j].y) * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
                integratorZ += (dist(generator)+measurementsOfInterest[j].z) * (timeSteps[i] - measurementsOfInterest[j - 1].timeStamp);
            } else {
                if (j == 0) {
                    integratorX +=
                            (dist(generator)+measurementsOfInterest[j].x) * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    integratorY +=
                            (dist(generator)+measurementsOfInterest[j].y) * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                    integratorZ +=
                            (dist(generator)+measurementsOfInterest[j].z) * (measurementsOfInterest[j].timeStamp - timeSteps[i - 1]);
                } else {
                    integratorX += (dist(generator)+(measurementsOfInterest[j].x + measurementsOfInterest[j - 1].x) / 2) *
                                   (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    integratorY += (dist(generator)+(measurementsOfInterest[j].y + measurementsOfInterest[j - 1].y) / 2) *
                                   (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                    integratorZ += (dist(generator)+(measurementsOfInterest[j].z + measurementsOfInterest[j - 1].z) / 2) *
                                   (measurementsOfInterest[j].timeStamp - measurementsOfInterest[j - 1].timeStamp);
                }
            }
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
                         graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        currentEdge.setTimeStamp(timeSteps[i + 1]);
        posOverTimeEdge.push_back(currentEdge);
    }

}

bool slamToolsRos::detectLoopClosure(graphSlamSaveStructure &graphSaved,
                                     double sigmaScaling, double cutoffFitnessOnDetect) {
    Eigen::Vector3d estimatedPosLastPoint = graphSaved.getVertexList().back().getPositionVertex();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudLast = graphSaved.getVertexList().back().getPointCloudCorrected();
    Eigen::ArrayXXf dist;
    dist.resize(graphSaved.getVertexList().size() - 1, 1);
    for (int s = 0; s < graphSaved.getVertexList().size() - 1; s++) {
        //dist.row(s) = (graphSaved.getVertexList()[s].getPositionVertex() - estimatedPosLastPoint).norm();
        dist.row(s) = pow((estimatedPosLastPoint.x() - graphSaved.getVertexList()[s].getPositionVertex().x()), 2) /
                      pow((sigmaScaling * graphSaved.getVertexList()[s].getCovariancePosition().x()), 2) +
                      pow((estimatedPosLastPoint.y() - graphSaved.getVertexList()[s].getPositionVertex().y()), 2) /
                      pow((sigmaScaling * graphSaved.getVertexList()[s].getCovariancePosition().y()), 2);
    }
    const int ignoreLastNLoopClosures = 1;
    std::vector<int> has2beChecked;
    if (dist.size() > ignoreLastNLoopClosures) {
        for (int i = 0; i < dist.size() - ignoreLastNLoopClosures; i++) {
            if (dist(i, 0) < 1 &&
                graphSaved.getVertexList()[i].getTypeOfVertex() == graphSlamSaveStructure::POINT_CLOUD_USAGE) {
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
            Eigen::Matrix4d currentTransformation, guess;
            guess = graphSaved.getVertexList().back().getTransformation().inverse() *
                    graphSaved.getVertexList()[has2beCheckedElemenet].getTransformation();
            currentTransformation = scanRegistrationClass::generalizedIcpRegistrationSimple(
                    graphSaved.getVertexList()[has2beCheckedElemenet].getPointCloudCorrected(),
                    graphSaved.getVertexList().back().getPointCloudCorrected(),
                    fitnessScore, guess);
            fitnessScore = sqrt(fitnessScore);
            if (fitnessScore < cutoffFitnessOnDetect) {//@TODO was 0.1
                std::cout << "Found Loop Closure with fitnessScore: " << fitnessScore << std::endl;
                if (fitnessScore < 0.01) {
                    std::cout << "FitnessScore Very Low: " << fitnessScore << std::endl;
                    fitnessScore = 0.01;
                }

//                pcl::io::savePCDFileASCII(
//                        "/home/tim/DataForTests/justShit/after_voxel_first.pcd",
//                        *graphSaved.getVertexList()[has2beCheckedElemenet].getPointCloudCorrected());
//                pcl::io::savePCDFileASCII(
//                        "/home/tim/DataForTests/justShit/after_voxel_second.pcd",
//                        *graphSaved.getVertexList().back().getPointCloudCorrected());
//                Eigen::Matrix3d rotationCurrent(currentTransformation.block<3, 3>(0, 0));
//                std::cout << "Euler Angles: " << rotationCurrent.eulerAngles(0, 1, 2) << std::endl;

                Eigen::Vector3d currentPosDiff;
                Eigen::Quaterniond currentRotDiff(currentTransformation.block<3, 3>(0, 0));
                currentPosDiff.x() = currentTransformation(0, 3);
                currentPosDiff.y() = currentTransformation(1, 3);
                currentPosDiff.z() = 0;
                Eigen::Vector3d positionCovariance(sqrt(fitnessScore), sqrt(fitnessScore), 0);
                graphSaved.addEdge((int) graphSaved.getVertexList().size() - 1, has2beCheckedElemenet, currentPosDiff,
                                   currentRotDiff, positionCovariance, 0.25 * sqrt(fitnessScore),
                                   graphSlamSaveStructure::POINT_CLOUD_USAGE);
                foundLoopClosure = true;
                loopclosureNumber++;
                if (loopclosureNumber > 2) { break; }// break if multiple loop closures are found
            }
        }
        if (foundLoopClosure) {
            return true;
        }

    }
    return false;
}