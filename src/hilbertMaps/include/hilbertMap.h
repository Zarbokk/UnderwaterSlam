//
// Created by tim on 11.05.21.
//

#ifndef SIMULATION_BLUEROV_HILBERTMAP_H
#define SIMULATION_BLUEROV_HILBERTMAP_H

#include "vector"
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>
#include <nav_msgs/OccupancyGrid.h>
//map definition
//x to top
//y to right
struct dataPointStruct {
    double x;
    double y;
    double z;
    double occupancy;
};

class hilbertMap {
public:
    hilbertMap(int numberOfFeaturesForEachDimension, int numberOfPointsToCalculateOccupancy,
               double quadraticOccupancyMapSize, int featureToUse) {
        //this->discritisationSizeFeatures = discritisationSizeFeatures;
        this->numberOfFeaturesForEachDimension = numberOfFeaturesForEachDimension;
        //Eigen::VectorXd weightVectorTMP(numberOfFeaturesForEachDimension * numberOfFeaturesForEachDimension);
        this->weightVector = Eigen::VectorXd(numberOfFeaturesForEachDimension * numberOfFeaturesForEachDimension);
        Eigen::MatrixXd inducedPointsTMP(numberOfFeaturesForEachDimension * numberOfFeaturesForEachDimension, 3);
        this->inducedPoints = inducedPointsTMP;
        this->numberOfPointsToCalculateOccupancy = numberOfPointsToCalculateOccupancy;
        this->quadraticOccupancyMapSize = quadraticOccupancyMapSize;
        this->currentMap = Eigen::MatrixXd(numberOfPointsToCalculateOccupancy, numberOfPointsToCalculateOccupancy);
        this->featureThatIsUsed = featureToUse;
        //fill weightVector
        for (int i = 0; i < (numberOfFeaturesForEachDimension * numberOfFeaturesForEachDimension); i++) {
            this->weightVector[i] = 0;
        }
        double discritisationSizeFeatures = 1 / (numberOfFeaturesForEachDimension / quadraticOccupancyMapSize);
        double shiftX = -0.5 * (numberOfFeaturesForEachDimension - 1) * discritisationSizeFeatures;
        double shiftY = -0.5 * (numberOfFeaturesForEachDimension - 1) * discritisationSizeFeatures;
        //fill induced points
        for (int i = 0; i < numberOfFeaturesForEachDimension; i++) {
            for (int j = 0; j < numberOfFeaturesForEachDimension; j++) {
                this->inducedPoints(i * numberOfFeaturesForEachDimension + j, 0) =
                        i * discritisationSizeFeatures + shiftX;
                this->inducedPoints(i * numberOfFeaturesForEachDimension + j, 1) =
                        j * discritisationSizeFeatures + shiftY;
            }
        }

        this->gradient = Eigen::VectorXd::Zero(
                this->numberOfFeaturesForEachDimension * this->numberOfFeaturesForEachDimension);
    }

    void createRandomMap();

//    double getDiscritisationSizeFeatures() const;
//
//    void setDiscritisationSizeFeatures(double discritisationSizeFeatures);

    int getNumberOfFeaturesForEachDimension() const;

    void setNumberOfFeaturesForEachDimension(int numberOfPointsForEachDimension);

    const Eigen::MatrixXd &getCurrentMap() const;

    void setCurrentMap(const Eigen::MatrixXd &currentMap);

    void trainClassifier(std::vector<dataPointStruct> &dataSet, int numberOfUsedDatapoints);

    double calculateOccupancy(Eigen::Vector3d pointOfInterest);

//    visualization_msgs::MarkerArray createMarkerArrayOfHilbertMap( double threshholdOccupancy = 0.15);
    //x and y pos from bottom left point of square
    nav_msgs::OccupancyGrid
    createOccupancyMapOfHilbert( double xPosSquare, double yPosSquare,
                                double sizeSquareOneDimension,bool exchangeXYAxis=false, double threshholdOccupancy = 0.15);

private:

    Eigen::VectorXd mappingBySparseFeatures(Eigen::Vector3d pointOfInterest);

    Eigen::VectorXd mappingByHingedFeatures(Eigen::Vector3d pointOfInterest);

    Eigen::VectorXd gradientOfSparseFeatures(Eigen::Vector3d pointOfInterest, double occupancy);

    Eigen::VectorXd gradientOfHingedFeatures(Eigen::Vector3d pointOfInterest, double occupancy);

    Eigen::VectorXd getGradient(Eigen::Vector3d pointOfInterest, double occupancy);

    double huberLoss(double input, double smoothingPoint = 0.1);

    int numberOfFeaturesForEachDimension;
    int numberOfPointsToCalculateOccupancy;
    int featureThatIsUsed;
    Eigen::VectorXd gradient ;
public:
    int getNumberOfPointsToCalculateOccupancy() const;

    void setNumberOfPointsToCalculateOccupancy(int numberOfPointsToCalculateOccupancy);

    double getQuadraticOccupancyMapSize() const;

    void setQuadraticOccupancyMapSize(double quadraticOccupancyMapSize);

    static const int SPARSE_RANDOM_FEATURES = 0;
    static const int HINGED_FEATURES = 1;

private:
    double quadraticOccupancyMapSize;//this is at the square one side length
    //std::vector<std::vector<dataPointStruct>> currentMap;
    Eigen::MatrixXd currentMap;
    //double discritisationSizeFeatures;
    Eigen::VectorXd weightVector;
    Eigen::MatrixXd inducedPoints;
};


#endif //SIMULATION_BLUEROV_HILBERTMAP_H
