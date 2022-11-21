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
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(
                                                                                edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                                                                                generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
                                                                        this->loopClosureNoiseModel);//@TODO different Noise Model

    } else {
        this->graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(fromKey, toKey, gtsam::Pose2(edgeToAdd.getPositionDifference().x(), edgeToAdd.getPositionDifference().y(),
                                                                                                     generalHelpfulTools::getRollPitchYaw(edgeToAdd.getRotationDifference())[2]),
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

    this->currentEstimate.insert(key,gtsam::Pose2(0,0,0));
}


void graphSlamSaveStructure::optimizeGraph(bool verbose) {
    gtsam::GaussNewtonParams parameters;
    parameters.relativeErrorTol = 1e-5;
    parameters.maxIterations = 100;

    gtsam::GaussNewtonOptimizer optimizer(this->graph, this->currentEstimate, parameters);
    // ... and optimize
    this->currentEstimate = optimizer.optimize();
//    this->currentEstimate.print("Final Result:\n");
    gtsam::Marginals marginals(graph, this->currentEstimate);



    for(int i  = 1 ; i<this->vertexList.size() ; i++){
        gtsam::Pose2 iterativePose = this->currentEstimate.at(this->vertexList[i].getKey()).cast<gtsam::Pose2>();
        this->vertexList.at(i).setPositionVertex(Eigen::Vector3d(iterativePose.x(),iterativePose.y(),0));
        this->vertexList.at(i).setRotationVertex(generalHelpfulTools::getQuaternionFromRPY(0,0,iterativePose.theta()));
//        std::cout << "covariance:\n" << marginals.marginalCovariance(this->vertexList.at(i).getKey()) << std::endl;
//        std::cout << "Pose :\n" << iterativePose << std::endl;
        this->vertexList.at(i).setCovariancePosition(Eigen::Vector3d(marginals.marginalCovariance(this->vertexList.at(i).getKey())(0,0),marginals.marginalCovariance(this->vertexList.at(i).getKey())(1,1),0));
        this->vertexList.at(i).setCovarianceQuaternion(marginals.marginalCovariance(this->vertexList.at(i).getKey())(2,2));

    }


}




void graphSlamSaveStructure::printCurrentState() {
    std::cout << "current State:" << std::endl;
    for (int i = 0; i < this->vertexList.size(); i++) {
        std::cout << "State:" << i << std::endl;
        std::cout << "Pose:" << std::endl;
        this->currentEstimate.at(this->vertexList[i].getKey()).print();
    }
}

void graphSlamSaveStructure::printCurrentStateGeneralInformation() {
    std::cout << "Number of Vertex:" << this->vertexList.size() << std::endl;
    for (int i = 0; i < this->vertexList.size(); i++) {
        std::cout << "Vertex Nr. " << i << " Pos:\n"
                  << this->vertexList[i].getPositionVertex() << std::endl;
    }
}

std::vector<vertex> *graphSlamSaveStructure::getVertexList() {
    return &this->vertexList;
}

std::vector<edge> *graphSlamSaveStructure::getEdgeList() {
    return &this->edgeList;
}




void graphSlamSaveStructure::saveGraphJson(std::string nameSavingFile) {

//    Json::Value keyFrames;
//
//    int currentKeyFrameNumber = 0;
//    for (int i = 0; i < this->vertexList.size(); i++) {
//        if (this->vertexList[i].getTypeOfVertex() == POINT_CLOUD_SAVED) {
//            // if point cloud is used then save this keyframe
//            Json::Value positionOfKeyframe;
//            positionOfKeyframe["x"] = this->vertexList[i].getPositionVertex()[0];
//            positionOfKeyframe["y"] = this->vertexList[i].getPositionVertex()[1];
//            positionOfKeyframe["z"] = this->vertexList[i].getPositionVertex()[2];
//            positionOfKeyframe["roll"] = 0;
//            positionOfKeyframe["pitch"] = 0;
//            positionOfKeyframe["yaw"] = this->getYawAngle(this->vertexList[i].getRotationVertex());
//            Json::Value pointCloud;
//
//            for (int j = 0; j < this->vertexList[i].getPointCloudCorrected()->points.size(); j++) {
//                Json::Value tmpPoint;
//                tmpPoint["x"] = this->vertexList[i].getPointCloudCorrected()->points[j].x;
//                tmpPoint["y"] = this->vertexList[i].getPointCloudCorrected()->points[j].y;
//                tmpPoint["z"] = this->vertexList[i].getPointCloudCorrected()->points[j].z;
//                pointCloud[j]["point"] = tmpPoint;
//            }
//            keyFrames[currentKeyFrameNumber]["position"] = positionOfKeyframe;
//            keyFrames[currentKeyFrameNumber]["pointCloud"] = pointCloud;
//            currentKeyFrameNumber++;
//        }
//    }
//
//    // create the main object
//    Json::Value outputText;
//    outputText["keyFrames"] = keyFrames;
//
//    std::ofstream ifs;
//    ifs.open(nameSavingFile);
//    ifs << outputText << '\n';
//    ifs.close();
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