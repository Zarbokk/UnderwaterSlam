//
// Created by tim on 23.02.21.
//

#include "graphSlamSaveStructure.h"
#include "random"

void
graphSlamSaveStructure::addEdge(int fromKey, int toKey, Eigen::Vector3d positionDifference,
                                Eigen::Quaterniond rotationDifference, Eigen::Vector3d covariancePosition,
                                double covarianceQuaternion, int typeOfEdge) {
    if (std::isnan(covarianceQuaternion) ||
        std::isnan(covariancePosition[0]) ||
        std::isnan(covariancePosition[1])) {
        std::cout << "IS NAN: " << std::endl;
    }
    edge edgeToAdd(fromKey, toKey, positionDifference, rotationDifference, covariancePosition,
                   covarianceQuaternion,
                   this->degreeOfFreedom, typeOfEdge);

    // add a factor between the keys
    if (abs(toKey - fromKey) > 2) {
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(1, 2, gtsam::Pose2(
                                                                                edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                                                                                generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
                                                                        this->deadReckoningNoiseModel);//@TODO different Noise Model

    } else {
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(1, 2, gtsam::Pose2(2, 0, 0),
                                                                        this->deadReckoningNoiseModel);

    }
    // add edge to edge list
    this->edgeList.push_back(edgeToAdd);

}

void graphSlamSaveStructure::addVertex(int key, const Eigen::Vector3d &positionVertex,
                                       const Eigen::Quaterniond &rotationVertex,
                                       const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,
                                       intensityMeasurement intensityInput, double timeStamp,
                                       int typeOfVertex) {

    vertex vertexToAdd(key, positionVertex, rotationVertex, this->degreeOfFreedom,
                       intensityInput, covariancePosition, covarianceQuaternion, timeStamp, typeOfVertex);
    this->vertexList.push_back(vertexToAdd);
}





double angleDiff(double first, double second) {//first-second
    return atan2(sin(first - second), cos(first - second));
}


void graphSlamSaveStructure::printCurrentState() {
    std::cout << "current State:" << std::endl;
    for (int i = 0; i < this->numberOfVertex; i++) {
        std::cout << "State:" << i << std::endl;
        std::cout << "Pos:" << std::endl;
        std::cout << this->vertexList[i].getPositionVertex() << std::endl;
        std::cout << "Rot:" << std::endl;
        std::cout <<
                  this->getYawAngle(this->vertexList[i].getRotationVertex()) *
                  180 / M_PI << std::endl;
    }
}

void graphSlamSaveStructure::printCurrentStateGeneralInformation() {
    std::cout << "Number of Vertex:" << this->numberOfVertex << std::endl;
    for (int i = 0; i < this->numberOfVertex; i++) {
        std::cout << "Vertex Nr. " << i << " Pos:\n"
                  << this->vertexList[i].getPositionVertex() << std::endl;
    }

//    std::cout << "Number of Edges:" << this->numberOfEdges << std::endl;
//    for (int i = 0; i < this->numberOfEdges; i++) {
//        std::cout << "Edge from " << this->edgeList[i].getFromVertex() << " to "
//                  << this->edgeList[i].getToVertex() << " Position:\n"
//                  << this->edgeList[i].getPositionDifference() << std::endl;
//    }

}


vertex *graphSlamSaveStructure::getVertexByKey(const int i) {
    if (i >= this->numberOfVertex) {
        std::cout << "access to a vertex that doesnt exist" << std::endl;
        std::exit(-1);
    }
    return &this->vertexList[i];
}

std::vector<vertex> *graphSlamSaveStructure::getVertexList() {
    return &this->vertexList;
}

std::vector<edge> *graphSlamSaveStructure::getEdgeList() {
    return &this->edgeList;
}

void graphSlamSaveStructure::optimizeGraph(bool verbose, std::vector<int> &holdStill, double maxTimeOptimization) {


}


void graphSlamSaveStructure::saveGraphJson(std::string nameSavingFile) {

    Json::Value keyFrames;

    int currentKeyFrameNumber = 0;
    for (int i = 0; i < this->numberOfVertex; i++) {
        if (this->vertexList[i].getTypeOfVertex() == POINT_CLOUD_SAVED) {
            // if point cloud is used then save this keyframe
            Json::Value positionOfKeyframe;
            positionOfKeyframe["x"] = this->vertexList[i].getPositionVertex()[0];
            positionOfKeyframe["y"] = this->vertexList[i].getPositionVertex()[1];
            positionOfKeyframe["z"] = this->vertexList[i].getPositionVertex()[2];
            positionOfKeyframe["roll"] = 0;
            positionOfKeyframe["pitch"] = 0;
            positionOfKeyframe["yaw"] = this->getYawAngle(this->vertexList[i].getRotationVertex());
            Json::Value pointCloud;

            for (int j = 0; j < this->vertexList[i].getPointCloudCorrected()->points.size(); j++) {
                Json::Value tmpPoint;
                tmpPoint["x"] = this->vertexList[i].getPointCloudCorrected()->points[j].x;
                tmpPoint["y"] = this->vertexList[i].getPointCloudCorrected()->points[j].y;
                tmpPoint["z"] = this->vertexList[i].getPointCloudCorrected()->points[j].z;
                pointCloud[j]["point"] = tmpPoint;
            }
            keyFrames[currentKeyFrameNumber]["position"] = positionOfKeyframe;
            keyFrames[currentKeyFrameNumber]["pointCloud"] = pointCloud;
            currentKeyFrameNumber++;
        }
    }

    // create the main object
    Json::Value outputText;
    outputText["keyFrames"] = keyFrames;

    std::ofstream ifs;
    ifs.open(nameSavingFile);
    ifs << outputText << '\n';
    ifs.close();
}


void graphSlamSaveStructure::addRandomNoiseToGraph(double stdDiviationGauss, double percentageOfRandomNoise) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::normal_distribution<> disNormal(0.0, stdDiviationGauss);


    double maximumIntensity = 0;
    for (int i = 0; i < this->vertexList.size(); i++) {
        intensityMeasurement tmpIntensity = this->vertexList[i].getIntensities();
        //find max intensity;
        for (int j = 0; j < tmpIntensity.intensities.size(); j++) {
            if (tmpIntensity.intensities[j] > maximumIntensity) {
                maximumIntensity = tmpIntensity.intensities[j];
            }
        }
    }

    for (int i = 0; i < this->vertexList.size(); i++) {
        intensityMeasurement tmpIntensity = this->vertexList[i].getIntensities();
        //find max intensity;
        for (int j = 0; j < tmpIntensity.intensities.size(); j++) {
            if (dis(gen) < percentageOfRandomNoise) {
                tmpIntensity.intensities[j] = dis(gen) * maximumIntensity;
//                std::cout << tmpIntensity.intensities[j] << std::endl;
            }


            tmpIntensity.intensities[j] = tmpIntensity.intensities[j] + disNormal(gen) * maximumIntensity;
            if (tmpIntensity.intensities[j] < 0) {
                tmpIntensity.intensities[j] = 0;
            }

        }
        this->vertexList[i].setIntensities(tmpIntensity);
    }




//    this->vertexList[i].setIntensities(tmpIntensity);

}