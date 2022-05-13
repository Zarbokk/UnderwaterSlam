//
// Created by tim on 22.02.21.
//

#include <Eigen/Geometry>
#include <iostream>
#include "graphSlamSaveStructure.h"


void createExampleVertex(graphSlamSaveStructure &graphSaved) {
    Eigen::Vector3d positionVertex0(0, 0, 0);
    Eigen::AngleAxisd rotation_vector0(0.0f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationVertex0(rotation_vector0);
    float covariance0 = 0.0;
    Eigen::Vector3d covarianceVector(1, 1, 0);
    graphSaved.addVertex(0, positionVertex0, rotationVertex0, covarianceVector * covariance0, covariance0, 0, 0);


    Eigen::Vector3d positionVertex1(1, 1, 0);
    Eigen::AngleAxisd rotation_vector1(-0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationVertex1(rotation_vector1.toRotationMatrix());
    graphSaved.addVertex(1, positionVertex1, rotationVertex1, covarianceVector * covariance0, covariance0, 0, 0);

    Eigen::Vector3d positionVertex2(2, 2, 0);
    Eigen::AngleAxisd rotation_vector2(-0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationVertex2(rotation_vector2.toRotationMatrix());
    graphSaved.addVertex(2, positionVertex2, rotationVertex2, covarianceVector * covariance0, covariance0, 0, 0);

    Eigen::Vector3d positionVertex3(3, 3, 0);
    Eigen::AngleAxisd rotation_vector3(0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationVertex3(rotation_vector3.toRotationMatrix());
    graphSaved.addVertex(3, positionVertex3, rotationVertex3, covarianceVector * covariance0, covariance0, 0, 0);

    Eigen::Vector3d positionVertex4(4, 4, 0);
    graphSaved.addVertex(4, positionVertex4, rotationVertex0, covarianceVector * covariance0, covariance0, 0, 0);

}

void createExampleEdge(graphSlamSaveStructure &graphSaved) {

    Eigen::Vector3d eyeVector(1, 1, 0);
    Eigen::Vector3d positionDifference0(1, 1, 0);
    Eigen::AngleAxisd rotation_vector0(-0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationDifference0(rotation_vector0.toRotationMatrix());
    float covariancePosition0 = 1;
    float covarianceQuaternion0 = 1;
    graphSaved.addEdge(0, 1, positionDifference0, rotationDifference0, eyeVector * covariancePosition0,
                       covarianceQuaternion0,0,10);

    Eigen::Vector3d positionDifference1(1, 1, 0);
    Eigen::AngleAxisd rotation_vector1(00.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationDifference1(rotation_vector1.toRotationMatrix());
    float covariancePosition1 = 1;
    float covarianceQuaternion1 = 1;
    graphSaved.addEdge(1, 2, positionDifference1, rotationDifference1, eyeVector * covariancePosition1,
                       covarianceQuaternion1,0,10);

    Eigen::Vector3d positionDifference2(1, 1, 0);
    Eigen::AngleAxisd rotation_vector2(0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationDifference2(rotation_vector2.toRotationMatrix());
    float covariancePosition2 = 1;
    float covarianceQuaternion2 = 1;
    graphSaved.addEdge(2, 3, positionDifference2, rotationDifference2, eyeVector * covariancePosition2,
                       covarianceQuaternion2,0,10);

    Eigen::Vector3d positionDifference3(1, 1, 0);
    Eigen::AngleAxisd rotation_vector3(0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationDifference3(rotation_vector3.toRotationMatrix());
    float covariancePosition3 = 1;
    float covarianceQuaternion3 = 1;
    graphSaved.addEdge(3, 4, positionDifference3, rotationDifference3, eyeVector * covariancePosition3,
                       covarianceQuaternion3,0,10);


    Eigen::Vector3d positionDifference4(3, 3, 0);
    Eigen::AngleAxisd rotation_vector4(0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationDifference4(rotation_vector4.toRotationMatrix());
    float covariancePosition4 = 1;
    float covarianceQuaternion4 = 1;
    graphSaved.addEdge(0, 4, positionDifference4, rotationDifference4, eyeVector * covariancePosition4,
                       covarianceQuaternion4,0,10);



}

void createLargeNumberOfVertex(graphSlamSaveStructure &graphSaved, int howManyVertexes){
    for(int i =0;i<howManyVertexes;i++){
        Eigen::Vector3d positionVertex0(i, i, 0);
        Eigen::AngleAxisd rotation_vector0(0.0f, Eigen::Vector3d(0, 0, 1));
        Eigen::Quaterniond rotationVertex0(rotation_vector0);
        float covariance0 = 0.0;
        Eigen::Vector3d covarianceVector(1, 1, 0);
        graphSaved.addVertex(i, positionVertex0, rotationVertex0, covarianceVector * covariance0, covariance0, 0, 0);

    }
}

void createLargeNumberOfEdges(graphSlamSaveStructure &graphSaved, int howManyEdges){
    for(int i =0;i<howManyEdges;i++){
        Eigen::Vector3d eyeVector(1, 1, 0);
        Eigen::Vector3d positionDifference0(1, 1, 0);
        Eigen::AngleAxisd rotation_vector0(-0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
        Eigen::Quaterniond rotationDifference0(rotation_vector0.toRotationMatrix());
        float covariancePosition0 = 1;
        float covarianceQuaternion0 = 1;
        graphSaved.addEdge(i, i+1, positionDifference0, rotationDifference0, eyeVector * covariancePosition0,
                           covarianceQuaternion0,0,10);
    }
}

int
main(int argc, char **argv) {
    const int dimension = 3;
    graphSlamSaveStructure graphSaved(dimension, ONLY_SIMPLE_GRAPH);

    createLargeNumberOfVertex(graphSaved,10);
    createLargeNumberOfEdges(graphSaved,9);
//    createExampleVertex(graphSaved);
//
//    createExampleEdge(graphSaved);
    Eigen::Vector3d eyeVector(1, 1, 0);
    Eigen::Vector3d positionDifference0(10, 10, 0);
    Eigen::AngleAxisd rotation_vector0(-0.0f / 180.0f * 3.14159f, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond rotationDifference0(rotation_vector0.toRotationMatrix());
    float covariancePosition0 = 1;
    float covarianceQuaternion0 = 1;
    graphSaved.addEdge(1, 8, positionDifference0, rotationDifference0, eyeVector * covariancePosition0,
                       covarianceQuaternion0,0,10);



    std::deque<double> subgraphs{3,8};
    graphSaved.initiallizeSubGraphs(subgraphs, 10);


    //graphSaved.optimizeGraphWithSlam();
    graphSaved.printCurrentStateGeneralInformation();
    //graphSaved.printCurrentState();
    //graphSaved.getEdgeBetweenNodes(2,4);
    //std::cout << "now hierachical graph design" << std::endl;


    graphSlamSaveStructure differentGraph = graphSaved;

    graphSaved.optimizeGraphWithSlamTopDown(false, 0.1, 10.0);

    std::vector<int> holdStill{0};
    differentGraph.optimizeGraphWithSlam(false, holdStill, 10.0);
    std::cout << "Hierachical Optimization:" << std::endl;
    graphSaved.printCurrentStateGeneralInformation();
    //hierachicalGraph.printCurrentStateGeneralInformation();
    std::cout << "Ordenary Optimization:" << std::endl;
    differentGraph.printCurrentStateGeneralInformation();

    return (0);
}

