//
// Created by tim on 11.05.21.
//

#include "hilbertMap.h"


void hilbertMap::createRandomMap() {
    //this->currentMap.clear();
    double scaling = this->quadraticOccupancyMapSize / this->numberOfPointsToCalculateOccupancy;
    for (int i = 0; i < this->currentMap.rows(); i++) {
        //std::vector<dataPointStruct> tmpVector;
        //this->currentMap.push_back(tmpVector);
        for (int j = 0; j < this->currentMap.cols(); j++) {
            Eigen::Vector3d currentPointOfInterest(i * scaling - this->quadraticOccupancyMapSize * 0.5,
                                                   j * scaling - this->quadraticOccupancyMapSize * 0.5, 0);
//            dataPointStruct point;
//            point.x = currentPointOfInterest.x();
//            point.y = currentPointOfInterest.y();
//            point.z = currentPointOfInterest.z();
            this->currentMap(i,j) = 0.5;
        }
    }
}

double hilbertMap::getDiscritisationSizeFeatures() const {
    return this->discritisationSizeFeatures;
}

void hilbertMap::setDiscritisationSizeFeatures(double discritisationSize) {
    this->discritisationSizeFeatures = discritisationSize;
}

int hilbertMap::getNumberOfFeaturesForEachDimension() const {
    return numberOfFeaturesForEachDimension;
}

void hilbertMap::setNumberOfFeaturesForEachDimension(int numberOfFeaturesForEachDimension) {
    hilbertMap::numberOfFeaturesForEachDimension = numberOfFeaturesForEachDimension;
}

const Eigen::MatrixXd &hilbertMap::getCurrentMap() const {
    return currentMap;
}

void hilbertMap::setCurrentMap(const Eigen::MatrixXd &currentMap) {
    hilbertMap::currentMap = currentMap;
}


void hilbertMap::trainClassifier(std::vector<dataPointStruct> &dataSet) {
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(
            this->numberOfFeaturesForEachDimension * this->numberOfFeaturesForEachDimension);
    double stepSize = 0.01;
    double movingAverage = 0.99;
    // randomly shuffles the dataset
    std::shuffle(dataSet.begin(), dataSet.end(), std::mt19937(std::random_device()()));

    for (int i = 0; i < 1000; i++) {//dataSet.size(); i++) {
        Eigen::Vector3d testX(dataSet[i].x, dataSet[i].y, 0);
        double testLabel = dataSet[i].occupancy;
        gradient = gradient * movingAverage + (1 - movingAverage) * this->gradientOfSparseFeatures(testX, testLabel);
        this->weightVector = this->weightVector - stepSize * gradient;
    }


    //std::function<double(double)> func = this->calculateOccupancy;
    //this->currentMap = this->currentMap.unaryExpr(func);
    //save new map
    // currentMap.clear();
    double scaling = this->quadraticOccupancyMapSize / this->numberOfPointsToCalculateOccupancy;
    for (int i = 0; i < this->numberOfPointsToCalculateOccupancy; i++) {
        //std::vector<dataPointStruct> tmpArray;
        //currentMap.push_back(tmpArray);
        for (int j = 0; j < this->numberOfPointsToCalculateOccupancy; j++) {
            Eigen::Vector3d currentPointOfInterest(i * scaling - this->quadraticOccupancyMapSize / 2,
                                                   j * scaling - this->quadraticOccupancyMapSize / 2, 0);
            //dataPointStruct pointTMP;
            //pointTMP.x = currentPointOfInterest.x();
            //pointTMP.y = currentPointOfInterest.y();
            //pointTMP.z = currentPointOfInterest.z();
            //pointTMP.occupancy = this->calculateOccupancy(currentPointOfInterest);
            currentMap(i,j) = this->calculateOccupancy(currentPointOfInterest);
        }
    }
}

//visualization_msgs::MarkerArray
//hilbertMap::createMarkerArrayOfHilbertMap( double threshholdOccupancy) {
//    //int pointsDimensionInMap = currentHilbertMap.getDiscritisationSize();
//    visualization_msgs::MarkerArray arrayOfCells;
//    for (int i = 0; i < this->currentMap.rows(); i++) {
//        for (int j = 0; j < this->currentMap.cols(); j++) {
//            visualization_msgs::Marker tmpMarker;
//            tmpMarker.pose.position.x = this->currentMap[i][j].x;
//            tmpMarker.pose.position.y = this->currentMap[i][j].y;
//            tmpMarker.pose.position.z = this->currentMap[i][j].z;
//            tmpMarker.pose.orientation.w = 1;
//            tmpMarker.header.frame_id = "map_ned";
//            tmpMarker.scale.x = (this->quadraticOccupancyMapSize-1)/this->numberOfPointsToCalculateOccupancy;
//            tmpMarker.scale.y = (this->quadraticOccupancyMapSize-1)/this->numberOfPointsToCalculateOccupancy;
//            tmpMarker.scale.z = (this->quadraticOccupancyMapSize-1)/this->numberOfPointsToCalculateOccupancy;
//            //determine color:
//            if (this->currentMap[i][j].occupancy > 0.5 + threshholdOccupancy) {
//                tmpMarker.color.r = 1;
//                tmpMarker.color.g = 0;
//                tmpMarker.color.b = 0;
//                tmpMarker.color.a = 0.8;
//            } else {
//                if (this->currentMap[i][j].occupancy < 0.5 - threshholdOccupancy) {
//                    tmpMarker.color.r = 0;
//                    tmpMarker.color.g = 1;
//                    tmpMarker.color.b = 0;
//                    tmpMarker.color.a = 0.2;
//                } else {
//                    tmpMarker.color.r = 0;
//                    tmpMarker.color.g = 0;
//                    tmpMarker.color.b = 1;
//                    tmpMarker.color.a = 0.4;
//                }
//            }
//
//            //tmpMarker.lifetime.sec = 10;
//
//            tmpMarker.type = 1;//1 for cube
//            tmpMarker.id = i * currentMap.size() + j;
//            arrayOfCells.markers.push_back(tmpMarker);
//
//        }
//    }
//    return arrayOfCells;
//}

Eigen::VectorXd hilbertMap::mappingBySparseFeatures(Eigen::Vector3d pointOfInterest) {
    Eigen::VectorXd sparseFeatures(this->numberOfFeaturesForEachDimension * this->numberOfFeaturesForEachDimension);

    for (int i = 0; i < this->numberOfFeaturesForEachDimension * this->numberOfFeaturesForEachDimension; i++) {
        double r = sqrt((pointOfInterest.transpose() - this->inducedPoints.block<1, 3>(i, 0)) *
                        0.1 * (pointOfInterest - this->inducedPoints.block<1, 3>(i, 0).transpose()));
        if (r > 1) {
            sparseFeatures(i) = 0;
        } else {
            sparseFeatures(i) = (2 + cos(2 * M_PI * r) / 3) * (1 - r) + 1 / (2 * M_PI) * sin(2 * M_PI * r);
        }
    }
    return sparseFeatures;
}

Eigen::VectorXd hilbertMap::gradientOfSparseFeatures(Eigen::Vector3d pointOfInterest, double occupancy) {
    double lambda1 = 0.0001;
    double lambda2 = 0.001;
    Eigen::VectorXd gradient(this->numberOfFeaturesForEachDimension * this->numberOfFeaturesForEachDimension);

    gradient = -occupancy * this->mappingBySparseFeatures(pointOfInterest) /
               (1 + exp(occupancy * this->weightVector.transpose() * this->mappingBySparseFeatures(pointOfInterest)));

    for (int i = 0; i < this->numberOfFeaturesForEachDimension * this->numberOfFeaturesForEachDimension; i++) {
        double Rdt = lambda1 * this->weightVector(i) * this->weightVector(i) +
                     lambda2 * this->huberLoss(this->weightVector(i), 0.5);
        gradient[i] = gradient[i] + Rdt;
    }
    return gradient;
}

template<typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double hilbertMap::huberLoss(double input, double smoothingPoint) {
    if (abs(input) > smoothingPoint) {
        return sgn(input);
    } else {
        return (input / smoothingPoint);
    }
}

double hilbertMap::calculateOccupancy(Eigen::Vector3d pointOfInterest) {
    return 1 - 1 / (1 + exp(this->weightVector.transpose() * this->mappingBySparseFeatures(pointOfInterest)));
}

int hilbertMap::getNumberOfPointsToCalculateOccupancy() const {
    return numberOfPointsToCalculateOccupancy;
}

void hilbertMap::setNumberOfPointsToCalculateOccupancy(int numberOfPointsToCalculateOccupancy) {
    hilbertMap::numberOfPointsToCalculateOccupancy = numberOfPointsToCalculateOccupancy;
}

double hilbertMap::getQuadraticOccupancyMapSize() const {
    return quadraticOccupancyMapSize;
}

void hilbertMap::setQuadraticOccupancyMapSize(double quadraticOccupancyMapSize) {
    hilbertMap::quadraticOccupancyMapSize = quadraticOccupancyMapSize;
}


nav_msgs::OccupancyGrid
hilbertMap::createOccupancyMapOfHilbert(double threshholdOccupancy) {
    //int pointsDimensionInMap = currentHilbertMap.getDiscritisationSize();
    nav_msgs::OccupancyGrid map;

    map.header.frame_id = "map_ned";
    map.info.height = this->numberOfPointsToCalculateOccupancy;
    map.info.width = this->numberOfPointsToCalculateOccupancy;
    map.info.resolution = this->quadraticOccupancyMapSize / this->numberOfPointsToCalculateOccupancy;
    map.info.origin.position.x = -this->quadraticOccupancyMapSize / 2;
    map.info.origin.position.y = -this->quadraticOccupancyMapSize / 2;
    map.info.origin.position.z = 0;
    for (int i = 0; i < this->currentMap.rows(); i++) {
        for (int j = 0; j < this->currentMap.cols(); j++) {
            //determine color:
            map.data.push_back((int) (this->currentMap(i,j) * 100));
//            if (this->currentMap[i][j].occupancy > 0.5 + threshholdOccupancy) {
//                map.data.push_back(100);
//            } else {
//                if (this->currentMap[i][j].occupancy < 0.5 - threshholdOccupancy) {
//                    map.data.push_back(10);
//                } else {
//                    map.data.push_back(50);
//                }
//            }
        }
    }
    return map;
}
