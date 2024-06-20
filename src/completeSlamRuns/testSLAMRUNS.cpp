//
// Created by jurobotics on 13.09.21.
//
#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <fstream>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "generalHelpfulTools.h"
#include "json.h"
#include <cmath>

struct intensityMeasurement {
    double time;
    double angle;//in rad
    double increment;//step size of angle;
    double range;
    std::vector<double> intensities;
};

struct intensityValues {
    Eigen::Matrix4d transformation;
    intensityMeasurement intensity;
    int type;
};

#define NUMBER_OF_POINTS_MAP 128
#define DIMENSION_OF_MAP 40
#define DISTANCE_TO_ROBOT 1.0


void compute3DEnvironment(std::vector<intensityValues> fullDataSetWithMicron) {
    int *voxelDataIndex;
    voxelDataIndex = (int *) malloc(
            sizeof(int) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
    double *mapData;
    mapData = (double *) malloc(
            sizeof(double) * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP);
    //set zero voxel and index
    for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
        voxelDataIndex[i] = 0;
        mapData[i] = 0;
    }

    for (int currentPosition = 0;
         currentPosition < fullDataSetWithMicron.size(); currentPosition++) {
        //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.
        //was 90 yaw and 180 roll
        if (currentPosition % 1000 == 0) {
            std::cout << currentPosition << std::endl;
        }
        if (fullDataSetWithMicron[currentPosition].type == 5) {
//        if (false) {

            double currentAngle = fullDataSetWithMicron[currentPosition].intensity.angle + M_PI;
//            std::cout << "before: " << currentAngle << std::endl;
            currentAngle = currentAngle + 2.0 * M_PI;

            currentAngle = std::fmod(currentAngle, 2.0 * M_PI);
            if (currentAngle > M_PI) {
                currentAngle = currentAngle - 2 * M_PI;
            }

//            std::cout << "after: " << currentAngle << std::endl;

//            if (abs(abs(currentAngle) - M_PI) < M_PI / 2.0 - M_PI / 20.0) {
            if (abs(abs(currentAngle) - M_PI) < M_PI / 4 ) {

                //this is Micron Sonar
                Eigen::Matrix4d transformationOfSonarPosition = fullDataSetWithMicron[currentPosition].transformation;
                //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
                Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(
                        fullDataSetWithMicron[currentPosition].intensity.angle + M_PI, 0,
                        0);
//                std::cout << (fullDataSetWithMicron[currentPosition].intensity.angle + M_PI)/M_PI*180.0 << std::endl;
                int ignoreDistance = (int) (DISTANCE_TO_ROBOT /
                                            (fullDataSetWithMicron[currentPosition].intensity.range /
                                             ((double) fullDataSetWithMicron[currentPosition].intensity.intensities.size())));


                for (int j = ignoreDistance;
                     j < fullDataSetWithMicron[currentPosition].intensity.intensities.size(); j++) {

                    double distanceOfIntensity =
                            j / ((double) fullDataSetWithMicron[currentPosition].intensity.intensities.size()) *
                            ((double) fullDataSetWithMicron[currentPosition].intensity.range);

//                    int incrementOfScan = fullDataSetWithMicron[currentPosition].intensity.increment;
                    if (distanceOfIntensity < 10.0) {
                        for (int l = -5; l <= +5; l++) {
//                            for (int openingAngle = -15; openingAngle <= +15; openingAngle++) {
                                for (int openingAngle = -1; openingAngle <= +1; openingAngle++) {
                                Eigen::Vector4d positionOfIntensity(
                                        0,
                                        0,
                                        distanceOfIntensity,
                                        1);
                                double incrementAngle = l / 400.0 * 2.0 * M_PI;
                                double openingAngleFor3D = openingAngle / 180.0 * M_PI;
                                Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(
                                        incrementAngle,
                                        0,
                                        0);
                                Eigen::Matrix4d openingAngleRotation = generalHelpfulTools::getTransformationMatrixFromRPY(
                                        0,
                                        openingAngleFor3D,
                                        0);
                                positionOfIntensity =
                                        transformationOfSonarPosition * rotationOfSonarAngleMatrix *
                                        rotationForBetterView *
                                        openingAngleRotation *
                                        positionOfIntensity;

//                            positionOfIntensity =
//                                    transformationOfIntensityRay * rotationOfSonarAngleMatrix * positionOfIntensity;
                                //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                                int indexX =
                                        (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                               2) +
                                        NUMBER_OF_POINTS_MAP / 2;
                                int indexY =
                                        (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                               2) +
                                        NUMBER_OF_POINTS_MAP / 2;
                                int indexZ =
                                        (int) (positionOfIntensity.z() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                               2) +
                                        NUMBER_OF_POINTS_MAP / 2;


                                if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP &&
                                    indexZ < NUMBER_OF_POINTS_MAP && indexX >= 0 &&
                                    indexY >= 0 && indexZ >= 0) {
                                    //                    std::cout << indexX << " " << indexY << std::endl;
                                    //if index fits inside of our data, add that data. Else Ignore
                                    voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                                   NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] =
                                            voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                                           NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] + 1;
                                    //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                                    mapData[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                            NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] =
                                            mapData[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                                    NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] +
                                            fullDataSetWithMicron[currentPosition].intensity.intensities[j];// -
                                            //j / fullDataSetWithMicron[currentPosition].intensity.intensities.size();
                                    //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                                    //                    std::cout << "random: " << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        } else {
//            if (fullDataSetWithMicron[currentPosition].type == 2 || fullDataSetWithMicron[currentPosition].type == 3) {
            if (false) {

                Eigen::Matrix4d transformationOfIntensityRay = fullDataSetWithMicron[currentPosition].transformation;
                //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
                Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                                 fullDataSetWithMicron[currentPosition].intensity.angle);
//            std::cout << fullDataSetWithMicron[currentPosition].intensity.angle << std::endl;
                int ignoreDistance = (int) (DISTANCE_TO_ROBOT /
                                            (fullDataSetWithMicron[currentPosition].intensity.range /
                                             ((double) fullDataSetWithMicron[currentPosition].intensity.intensities.size())));


                for (int j = ignoreDistance;
                     j < fullDataSetWithMicron[currentPosition].intensity.intensities.size(); j++) {
                    double distanceOfIntensity =
                            j / ((double) fullDataSetWithMicron[currentPosition].intensity.intensities.size()) *
                            ((double) fullDataSetWithMicron[currentPosition].intensity.range);

                    int incrementOfScan = fullDataSetWithMicron[currentPosition].intensity.increment;
                    for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                        for (int openingAngle = -10; openingAngle <= +10; openingAngle++) {
                            Eigen::Vector4d positionOfIntensity(
                                    distanceOfIntensity,
                                    0,
                                    0,
                                    1);
                            double openingAngleFor3D = openingAngle / 180.0 * M_PI;
                            Eigen::Matrix4d openingAngleRotation = generalHelpfulTools::getTransformationMatrixFromRPY(
                                    0,
                                    openingAngleFor3D,
                                    0);


                            double rotationOfPoint = l / 400.0;
                            Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(
                                    0,
                                    0,
                                    rotationOfPoint);
                            positionOfIntensity = rotationForBetterView * openingAngleRotation * positionOfIntensity;

                            positionOfIntensity =
                                    transformationOfIntensityRay * rotationOfSonarAngleMatrix *
                                    positionOfIntensity;
                            //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                            int indexX =
                                    (int) (positionOfIntensity.x() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                           2) +
                                    NUMBER_OF_POINTS_MAP / 2;
                            int indexY =
                                    (int) (positionOfIntensity.y() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                           2) +
                                    NUMBER_OF_POINTS_MAP / 2;
                            int indexZ =
                                    (int) (positionOfIntensity.z() / (DIMENSION_OF_MAP / 2) * NUMBER_OF_POINTS_MAP /
                                           2) +
                                    NUMBER_OF_POINTS_MAP / 2;

                            if (indexX < NUMBER_OF_POINTS_MAP && indexY < NUMBER_OF_POINTS_MAP && indexY >= 0 &&
                                indexZ < NUMBER_OF_POINTS_MAP &&
                                indexX >= 0 && indexZ >= 0) {
                                //                    std::cout << indexX << " " << indexY << std::endl;
                                //if index fits inside of our data, add that data. Else Ignore
                                voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                               NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] =
                                        voxelDataIndex[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                                       NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] + 1;
                                //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                                mapData[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                        NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] =
                                        mapData[indexX + NUMBER_OF_POINTS_MAP * indexY +
                                                NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * indexZ] +
                                        fullDataSetWithMicron[currentPosition].intensity.intensities[j] * 3;
                                //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                                //                    std::cout << "random: " << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }


    for (int i = 0; i < NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP; i++) {
        if (voxelDataIndex[i] > 0) {
            mapData[i] = mapData[i] / voxelDataIndex[i];
        }
    }

    std::ofstream myFile1;
    myFile1.open(
            "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/3D/csvFiles/current3DMapTest.csv");
    for (int k = 0; k < NUMBER_OF_POINTS_MAP; k++) {
        for (int j = 0; j < NUMBER_OF_POINTS_MAP; j++) {
            for (int i = 0; i < NUMBER_OF_POINTS_MAP; i++) {

                myFile1 << mapData[k + NUMBER_OF_POINTS_MAP * j +
                                   NUMBER_OF_POINTS_MAP * NUMBER_OF_POINTS_MAP * i]
                        << std::endl;//number of possible rotations
            }
        }
    }


    myFile1.close();

}


int main(int argc, char **argv) {

    std::ifstream testFile(
            "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/3D/csvFiles/fullGraphDataset.csv",
            std::ifstream::binary);
    Json::Value root;
    testFile >> root;

    std::vector<intensityValues> fullDataSetWithMicron;


    for (int i = 0; i < root["keyFrames"].size(); i++) {
        intensityValues intensityTMP;
        Eigen::Matrix4d transformationTMP;
        intensityMeasurement intensityMeasurementTMP;

        intensityMeasurementTMP.time = root["keyFrames"][i]["intensityValues"]["time"].asDouble();
        intensityMeasurementTMP.angle = root["keyFrames"][i]["intensityValues"]["angle"].asDouble();
        intensityMeasurementTMP.increment = root["keyFrames"][i]["intensityValues"]["increment"].asDouble();
        intensityMeasurementTMP.range = root["keyFrames"][i]["intensityValues"]["range"].asDouble();
        for (int j = 0; j < root["keyFrames"][i]["intensityValues"]["intensity"].size(); j++) {
            intensityMeasurementTMP.intensities.push_back(
                    root["keyFrames"][i]["intensityValues"]["intensity"][j].asDouble());
        }

        transformationTMP = generalHelpfulTools::getTransformationMatrixFromRPY(
                root["keyFrames"][i]["position"]["roll"].asDouble(),
                root["keyFrames"][i]["position"]["pitch"].asDouble(),
                root["keyFrames"][i]["position"]["yaw"].asDouble());
        Eigen::Vector3d translation;
        translation.x() = root["keyFrames"][i]["position"]["x"].asDouble();
        translation.y() = root["keyFrames"][i]["position"]["y"].asDouble();
        translation.z() = root["keyFrames"][i]["position"]["z"].asDouble();
        transformationTMP.block<3, 1>(0, 3) = translation;

        intensityTMP.intensity = intensityMeasurementTMP;
        intensityTMP.type = root["keyFrames"][i]["intensityValues"]["type"].asInt();
        intensityTMP.transformation = transformationTMP;


        fullDataSetWithMicron.push_back(intensityTMP);
    }










//    std::cout<<root; //This will print the entire json object.





    compute3DEnvironment(fullDataSetWithMicron);


    return (0);
}
