//
// Created by tim on 22.02.21.
//
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include "graphSlamSaveStructure.h"


void createExampleVertex(graphSlamSaveStructure &graphSaved) {
    Eigen::Vector3f positionVertex0(0, 0, 0);
    Eigen::AngleAxisf rotation_vector0(0.0f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex0(rotation_vector0);
    float covariance0 = 0.0;
    Eigen::Vector3f covarianceVector(1, 1, 0);
    graphSaved.addVertex(0, positionVertex0, rotationVertex0, covarianceVector * covariance0, covariance0);

    Eigen::Vector3f positionVertex1(1, 1, 0);
    Eigen::AngleAxisf rotation_vector1(-0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex1(rotation_vector1.toRotationMatrix());
    graphSaved.addVertex(1, positionVertex1, rotationVertex1, covarianceVector * covariance0, covariance0);

    Eigen::Vector3f positionVertex2(1, 1, 0);
    Eigen::AngleAxisf rotation_vector2(-0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex2(rotation_vector2.toRotationMatrix());
    graphSaved.addVertex(2, positionVertex2, rotationVertex2, covarianceVector * covariance0, covariance0);

    Eigen::Vector3f positionVertex3(1, 1, 0);
    Eigen::AngleAxisf rotation_vector3(0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationVertex3(rotation_vector3.toRotationMatrix());
    graphSaved.addVertex(3, positionVertex3, rotationVertex3, covarianceVector * covariance0, covariance0);

    Eigen::Vector3f positionVertex4(2, 2, 0);
    graphSaved.addVertex(4, positionVertex4, rotationVertex0, covarianceVector * covariance0, covariance0);

}

void createExampleEdge(graphSlamSaveStructure &graphSaved) {

    Eigen::Vector3f eyeVector(1, 1, 0);
    Eigen::Vector3f positionDifference0(8, 10, 0);
    Eigen::AngleAxisf rotation_vector0(-0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference0(rotation_vector0.toRotationMatrix());
    float covariancePosition0 = 0.1;
    float covarianceQuaternion0 = 0.1;
    graphSaved.addEdge(0, 1, positionDifference0, rotationDifference0, eyeVector * covariancePosition0,
                       covarianceQuaternion0);

    Eigen::Vector3f positionDifference1(4, 9, 0);
    Eigen::AngleAxisf rotation_vector1(30.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference1(rotation_vector1.toRotationMatrix());
    float covariancePosition1 = 3;
    float covarianceQuaternion1 = 3;
    graphSaved.addEdge(1, 2, positionDifference1, rotationDifference1, eyeVector * covariancePosition1,
                       covarianceQuaternion1);

    Eigen::Vector3f positionDifference2(0, 12.4, 0);
    Eigen::AngleAxisf rotation_vector2(0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference2(rotation_vector2.toRotationMatrix());
    float covariancePosition2 = 0.1;
    float covarianceQuaternion2 = 0.1;
    graphSaved.addEdge(2, 3, positionDifference2, rotationDifference2, eyeVector * covariancePosition2,
                       covarianceQuaternion2);

    Eigen::Vector3f positionDifference3(2, -9, 0);
    Eigen::AngleAxisf rotation_vector3(0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference3(rotation_vector3.toRotationMatrix());
    float covariancePosition3 = 0.1;
    float covarianceQuaternion3 = 0.1;
    graphSaved.addEdge(3, 4, positionDifference3, rotationDifference3, eyeVector * covariancePosition3,
                       covarianceQuaternion3);


    Eigen::Vector3f positionDifference4(2, 12.4 - 9, 0);
    Eigen::AngleAxisf rotation_vector4(0.0f / 180.0f * 3.14159f, Eigen::Vector3f(0, 0, 1));
    Eigen::Quaternionf rotationDifference4(rotation_vector4.toRotationMatrix());
    float covariancePosition4 = 0.1;
    float covarianceQuaternion4 = 0.1;
    graphSaved.addEdge(2, 4, positionDifference4, rotationDifference4, eyeVector * covariancePosition4,
                       covarianceQuaternion4);

}


int
main(int argc, char **argv) {
    const int dimension = 3;
    graphSlamSaveStructure graphSaved(dimension);


    createExampleVertex(graphSaved);

    createExampleEdge(graphSaved);

    //graphSaved.optimizeGraphWithSlam();
    graphSaved.printCurrentStateGeneralInformation();
    //graphSaved.printCurrentState();
    //graphSaved.getEdgeBetweenNodes(2,4);
    //std::cout << "now hierachical graph design" << std::endl;
    graphSaved.createHierachicalGraph(1);//this is 1 m
    graphSaved.optimizeGraphWithSlamTopDown(false);
    graphSlamSaveStructure differentGraph = graphSaved;

    differentGraph.optimizeGraphWithSlam(false);

    graphSaved.printCurrentStateGeneralInformation();
    //hierachicalGraph.printCurrentStateGeneralInformation();
    graphSaved.printCurrentStateGeneralInformation();

    return (0);
}

