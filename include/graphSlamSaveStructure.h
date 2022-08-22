//
// Created by tim on 23.02.21.
//
#include "edge.h"
#include "vertex.h"
#include <fstream>
#include<Eigen/SparseCholesky>
#include "json.h"
#include <chrono>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

#ifndef SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
#define SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H


class graphSlamSaveStructure {
public:
    graphSlamSaveStructure(int degreeOfFreedom,int typeOfGraphSlam) {
        if (degreeOfFreedom == 3) {
            this->degreeOfFreedom = degreeOfFreedom;
            this->numberOfEdges = 0;
            this->numberOfVertex = 0;
            this->hasHierachicalGraph = false;
            this->typeOfGraphSlam =typeOfGraphSlam;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }

    void addEdge(int fromVertex, int toVertex, Eigen::Vector3d positionDifference,
                 Eigen::Quaterniond rotationDifference,  Eigen::Vector3d covariancePosition,
                  double covarianceQuaternion,int typeOfEdge, double maxTimeOptimization);

    void addEdge( int fromVertex,  int toVertex,  Eigen::Vector3d positionDifference,
                  Eigen::Quaterniond rotationDifference, Eigen::Vector3d covariancePosition,
                  double covarianceQuaternion, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud,int typeOfEdge, double maxTimeOptimization);

    void addVertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
                   const Eigen::Vector3d &covariancePosition, double covarianceQuaternion,double timeStamp,int typeOfVertex);

    void addVertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
                   const Eigen::Vector3d &covariancePosition, double covarianceQuaternion,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,double timeStamp,int typeOfVertex);

    void addVertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
                   const Eigen::Vector3d &covariancePosition, double covarianceQuaternion,
                   intensityMeasurement intensityInput,double timeStamp,int typeOfVertex);

    Eigen::SparseMatrix<double> getInformationMatrix();

    Eigen::SparseMatrix<double> getJacobianMatrix();

    Eigen::MatrixXd getErrorMatrix();

    void printCurrentState();

    void printCurrentStateGeneralInformation();

    void addToEveryState(std::vector<Eigen::Vector3d> &positionDifferenceVector,
                         std::vector<Eigen::Quaterniond> &rotationDifferenceVector);

    void addToState(Eigen::MatrixXd &vectorToAdd);//from optimization

    vertex* getVertexByIndex(int i);

    std::vector<vertex> *getVertexList();

    std::vector<edge> *getEdgeList();

    void optimizeGraphWithSlam(bool verbose, std::vector<int> &holdStill, double maxTimeOptimization);

    void optimizeGraphWithSlamTopDown(bool verbose, double cellSize, double maxTimeOptimization);

    void initiallizeSubGraphs(std::deque<double> cellSizes, double maxTimeOptimization);//cell sizes in order (first 1m cecond 4 m and so on)

    void createHierachicalGraph(double cellSizeDes, double maxTimeOptimization);

    edge getEdgeBetweenNodes(int fromVertex, int toVertex, std::vector<int> &holdStill, double maxTimeOptimization);

    Eigen::MatrixXd transformStateDiffToAddVector(std::vector<vertex> &stateBeforeOptimization,
                                                  std::vector<vertex> &stateAfterOptimization) const;

    graphSlamSaveStructure getSubGraph();

    bool createSubGraphBetweenCell(int vertexIndexFrom, int vertexIndexTo, graphSlamSaveStructure &currentSubGraph, double maxTimeOptimization);

    void calculateCovarianceInCloseProximity(double maxTimeOptimization);// calculate the subgraph for the last hierachical graph, and connected cells. then calculate covariances.

    void removeLastEdge();

    void saveGraphJson(std::string nameSavingFile);

    void resetHierachicalGraph();

    void addRandomNoiseToGraph(double stdDiviationGauss, double percentageOfRandomNoise);
//    static const int POINT_CLOUD_SAVED = 0;
//    static const int INTEGRATED_POSE =1;
//    static const int FIRST_ENTRY =2;
//    static const int INTENSITY_SAVED =3;
//    static const int INTENSITY_BASED_GRAPH =4;
//    static const int POINT_CLOUD_BASED_GRAPH =5;
//    static const int ONLY_SIMPLE_GRAPH =6;
//    static const int INTENSITY_SAVED_AND_KEYFRAME =7;
//    static const int LOOP_CLOSURE =8;

private:

    void removeRowColumn(Eigen::SparseMatrix<double> &matrix, int rowToRemove) const;

    void lookupTableCreation(double minDistanceForNewCell);

    void
    getListofConnectedVertexAndEdges(std::vector<int> &vertexIndicesofICell, std::vector<edge> &listOfContainingEdges,
                                     std::vector<int> &listOfConnectedVertexes);

    int getCellOfVertexIndex(int vertexIndex);

    static bool checkIfElementsOfVectorAreEqual(std::vector<int> &i0, std::vector<int> &i1);

    static std::vector<int> joinTwoLists(std::vector<int> &i0, std::vector<int> &i1);

    bool checkIfDirectConnectionExists(int vertexIndex0,int vertexIndex1);

    static double getYawAngle(Eigen::Quaterniond quaternionYaw);

    int degreeOfFreedom;//3 for [x y alpha] or 6 for [x y z alpha beta gamma]
    int numberOfEdges;
    int numberOfVertex;
    int typeOfGraphSlam;
    std::vector<edge> edgeList;
    std::vector<vertex> vertexList;
    bool hasHierachicalGraph;
    double cellSize;
    std::vector<std::vector<int>> lookUpTableCell;// [i][j] i = cell j=vertex (cell of sub graph and vertex of graph)
    graphSlamSaveStructure *hierachicalGraph;

};


#endif //SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
