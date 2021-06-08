//
// Created by tim on 11.05.21.
//

#ifndef SIMULATION_BLUEROV_HILBERTMAP_H
#define SIMULATION_BLUEROV_HILBERTMAP_H

#include "vector"
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>
#include <nav_msgs/OccupancyGrid.h>

struct dataPointStruct {
    double x;
    double y;
    double z;
    double occupancy;
};

class hilbertMap {
public:
    hilbertMap(int numberOfFeaturesForEachDimension, double discritisationSizeFeatures, int numberOfPointsToCalculateOccupancy, double quadraticOccupancyMapSize) {
        this->discritisationSizeFeatures = discritisationSizeFeatures;
        this->numberOfFeaturesForEachDimension = numberOfFeaturesForEachDimension;
        Eigen::VectorXd weightVectorTMP(numberOfFeaturesForEachDimension * numberOfFeaturesForEachDimension);
        this->weightVector = weightVectorTMP;
        Eigen::MatrixXd inducedPointsTMP(numberOfFeaturesForEachDimension * numberOfFeaturesForEachDimension, 3);
        this->inducedPoints = inducedPointsTMP;
        this->numberOfPointsToCalculateOccupancy = numberOfPointsToCalculateOccupancy;
        this->quadraticOccupancyMapSize = quadraticOccupancyMapSize;
        this->currentMap = Eigen::MatrixXd(numberOfPointsToCalculateOccupancy,numberOfPointsToCalculateOccupancy);
        //fill weightVector
        for (int i = 0; i < (numberOfFeaturesForEachDimension * numberOfFeaturesForEachDimension); i++) {
            this->weightVector[i] = 0;
        }
        double shiftX = -0.5 * (numberOfFeaturesForEachDimension - 1) * discritisationSizeFeatures;
        double shiftY = -0.5 * (numberOfFeaturesForEachDimension - 1) * discritisationSizeFeatures;
        //fill induced points
        for (int i = 0; i < numberOfFeaturesForEachDimension; i++) {
            for (int j = 0; j < numberOfFeaturesForEachDimension; j++) {
                this->inducedPoints(i * numberOfFeaturesForEachDimension + j, 0) = i * discritisationSizeFeatures + shiftX;
                this->inducedPoints(i * numberOfFeaturesForEachDimension + j, 1) = j * discritisationSizeFeatures + shiftY;
            }
        }
    }

    void createRandomMap();

    double getDiscritisationSizeFeatures() const;

    void setDiscritisationSizeFeatures(double discritisationSizeFeatures);

    int getNumberOfFeaturesForEachDimension() const;

    void setNumberOfFeaturesForEachDimension(int numberOfPointsForEachDimension);

    const Eigen::MatrixXd &getCurrentMap() const;

    void setCurrentMap(const Eigen::MatrixXd &currentMap);

    void trainClassifier(std::vector<dataPointStruct> &dataSet);

    double calculateOccupancy(Eigen::Vector3d pointOfInterest);

//    visualization_msgs::MarkerArray createMarkerArrayOfHilbertMap( double threshholdOccupancy = 0.15);

    nav_msgs::OccupancyGrid createOccupancyMapOfHilbert( double threshholdOccupancy = 0.15);

private:

    Eigen::VectorXd mappingBySparseFeatures(Eigen::Vector3d pointOfInterest);
    Eigen::VectorXd gradientOfSparseFeatures(Eigen::Vector3d pointOfInterest,double occupancy);
    double huberLoss(double input,double smoothingPoint = 0.1);
    int numberOfFeaturesForEachDimension;
    int numberOfPointsToCalculateOccupancy;
public:
    int getNumberOfPointsToCalculateOccupancy() const;

    void setNumberOfPointsToCalculateOccupancy(int numberOfPointsToCalculateOccupancy);

    double getQuadraticOccupancyMapSize() const;

    void setQuadraticOccupancyMapSize(double quadraticOccupancyMapSize);

private:
    double quadraticOccupancyMapSize;//this is at the square one side length
    //std::vector<std::vector<dataPointStruct>> currentMap;
    Eigen::MatrixXd currentMap;
    double discritisationSizeFeatures;
    Eigen::VectorXd weightVector;
    Eigen::MatrixXd inducedPoints;
};


#endif //SIMULATION_BLUEROV_HILBERTMAP_H
