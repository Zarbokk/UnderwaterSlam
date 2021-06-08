//
// Created by tim on 23.02.21.
//
#include "edge.h"
#include "vertex.h"
#include <fstream>
#include<Eigen/SparseCholesky>
#include "json.h"
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

#ifndef SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
#define SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H


class graphSlamSaveStructure {
public:
    graphSlamSaveStructure(int degreeOfFreedom) {
        if (degreeOfFreedom == 3) {
            graphSlamSaveStructure::degreeOfFreedom = degreeOfFreedom;
            graphSlamSaveStructure::numberOfEdges = 0;
            graphSlamSaveStructure::numberOfVertex = 0;
            hasHierachicalGraph = false;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }

    void addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3d &positionDifference,
                 const Eigen::Quaterniond &rotationDifference, const Eigen::Vector3d covariancePosition,
                 const double covarianceQuaternion,int typeOfEdge);

    void addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3d &positionDifference,
                 const Eigen::Quaterniond &rotationDifference, const Eigen::Vector3d covariancePosition,
                 const double covarianceQuaternion, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,int typeOfEdge);

    void addVertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
                   const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,double timeStamp,int typeOfVertex);

    void addVertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
                   const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,double timeStamp,int typeOfVertex);

    Eigen::SparseMatrix<double> getInformationMatrix();

    Eigen::SparseMatrix<double> getJacobianMatrix();

    Eigen::MatrixXd getErrorMatrix();

    void printCurrentState();

    void printCurrentStateGeneralInformation();

    void addToEveryState(std::vector<Eigen::Vector3d> &positionDifferenceVector,
                         std::vector<Eigen::Quaterniond> &rotationDifferenceVector);

    void addToState(Eigen::MatrixXd &vectorToAdd);//from optimization

    vertex* getVertexByIndex(int i);

    std::vector<vertex> getVertexList();

    std::vector<edge> *getEdgeList();

    void optimizeGraphWithSlam(bool verbose, std::vector<int> &holdStill);

    void optimizeGraphWithSlamTopDown(bool verbose,double cellSize);

    void initiallizeSubGraphs(std::deque<double> cellSizes);//cell sizes in order (first 1m cecond 4 m and so on)

    void createHierachicalGraph(double cellSizeDes);

    edge getEdgeBetweenNodes(int fromVertex, int toVertex, std::vector<int> &holdStill);

    Eigen::MatrixXd transformStateDiffToAddVector(std::vector<vertex> &stateBeforeOptimization,
                                                  std::vector<vertex> &stateAfterOptimization) const;

    graphSlamSaveStructure getSubGraph();

    bool createSubGraphBetweenCell(int vertexIndexFrom, int vertexIndexTo, graphSlamSaveStructure &currentSubGraph);

    void calculateCovarianceInCloseProximity();// calculate the subgraph for the last hierachical graph, and connected cells. then calculate covariances.

    void removeLastEdge();

    void saveGraphJson(std::string nameSavingFile);

    static const int POINT_CLOUD_USAGE = 0;
    static const int INTEGRATED_POS_USAGE = 1;
    static const int FIRST_ENTRY = 2;
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
    std::vector<edge> edgeList;
    std::vector<vertex> vertexList;
    bool hasHierachicalGraph;
    double cellSize;
    std::vector<std::vector<int>> lookUpTableCell;// [i][j] i = cell j=vertex (cell of sub graph and vertex of graph)
    graphSlamSaveStructure *hierachicalGraph;
};


#endif //SIMULATION_BLUEROV_GRAPHSLAMSAVESTRUCTURE_H
