//
// Created by tim on 23.02.21.
//

#include "graphSlamSaveStructure.h"


void
graphSlamSaveStructure::addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3d &positionDifference,
                                const Eigen::Quaterniond &rotationDifference, const Eigen::Vector3d covariancePosition,
                                const double covarianceQuaternion, int typeOfEdge) {
    if (std::isnan(covarianceQuaternion) ||
        std::isnan(covariancePosition[0]) ||
        std::isnan(covariancePosition[1])) {
        std::cout << "IS NAN: " << std::endl;
    }
    edge edgeToAdd(fromVertex, toVertex, positionDifference, rotationDifference, covariancePosition,
                   covarianceQuaternion,
                   this->degreeOfFreedom, typeOfEdge);
    this->numberOfEdges += 1;
    this->edgeList.push_back(edgeToAdd);

    // look from where to where the edge is going then calculate subgraph for lower number of edge new

    if (this->hasHierachicalGraph) {
        //get cell from
        int indexCellFrom = this->getCellOfVertexIndex(this->edgeList.back().getFromVertex());
        int indexCellTo = this->getCellOfVertexIndex(this->edgeList.back().getToVertex());

        if (indexCellFrom == indexCellTo) {//calculate every connection to cell new(subgraph)
            std::vector<edge> listOfContainingEdges;
            std::vector<int> listOfConnectedVertexes;//should contain every vertex connected to the current cell
            getListofConnectedVertexAndEdges(this->lookUpTableCell[indexCellFrom], listOfContainingEdges,
                                             listOfConnectedVertexes);
            for (int i = 0; i < this->lookUpTableCell.size(); i++) {
                if (i != indexCellFrom) {

                    bool containingConnection = checkIfElementsOfVectorAreEqual(this->lookUpTableCell[i],
                                                                                listOfConnectedVertexes);//check if overlap exists
                    if (containingConnection) {//create graph from indexCellFrom to i
                        graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
                        for (int k = 0;
                             k < this->lookUpTableCell[indexCellFrom].size(); k++) {//add vertices of cell indexCellFrom
                            vertex tmpVertex = this->vertexList[this->lookUpTableCell[indexCellFrom][k]];
                            currentSubGraph.addVertex(k, tmpVertex.getPositionVertex(), tmpVertex.getRotationVertex(),
                                                      tmpVertex.getCovariancePosition(),
                                                      tmpVertex.getCovarianceQuaternion(), 0,
                                                      graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                        }

                        for (int k = 0; k < this->lookUpTableCell[i].size(); k++) {//add vertices of cell i
                            vertex tmpVertex = this->vertexList[this->lookUpTableCell[i][k]];
                            currentSubGraph.addVertex(k + (int) this->lookUpTableCell[indexCellFrom].size(),
                                                      tmpVertex.getPositionVertex(),
                                                      tmpVertex.getRotationVertex(),
                                                      tmpVertex.getCovariancePosition(),
                                                      tmpVertex.getCovarianceQuaternion(), 0,
                                                      graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                        }

                        std::vector<int> vertexIndexOfJoinedGraph = joinTwoLists(this->lookUpTableCell[indexCellFrom],
                                                                                 this->lookUpTableCell[i]);//list of vertex index of cell indexCellFrom and cell i
                        for (auto &currentEdge : this->edgeList) {//go threw complete edge lists

                            auto itFromVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                                          vertexIndexOfJoinedGraph.end(),
                                                          currentEdge.getFromVertex());
                            auto itToVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                                        vertexIndexOfJoinedGraph.end(),
                                                        currentEdge.getToVertex());

                            if (itFromVertex != vertexIndexOfJoinedGraph.end() &&
                                itToVertex !=
                                vertexIndexOfJoinedGraph.end()) {// check if edge is contained in joined graph
                                int indexToVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itToVertex);
                                int indexFromVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itFromVertex);
                                currentSubGraph.addEdge(indexFromVertex, indexToVertex,
                                                        currentEdge.getPositionDifference(),
                                                        currentEdge.getRotationDifference(),
                                                        currentEdge.getCovariancePosition(),
                                                        currentEdge.getCovarianceQuaternion(),
                                                        graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                            }
                        }
                        std::vector<int> holdStill{0};
                        edge resultingEdge = currentSubGraph.getEdgeBetweenNodes(0,
                                                                                 this->lookUpTableCell[indexCellFrom].size(),
                                                                                 holdStill);
                        resultingEdge.setFromVertex(indexCellFrom);
                        resultingEdge.setToVertex(i);
                        for (int j = 0; j < hierachicalGraph->edgeList.size(); j++) {
                            if (hierachicalGraph->edgeList[j].getFromVertex() == indexCellFrom &&
                                hierachicalGraph->edgeList[j].getToVertex() == i ||
                                hierachicalGraph->edgeList[j].getFromVertex() == i &&
                                hierachicalGraph->edgeList[j].getToVertex() == indexCellFrom) {
                                hierachicalGraph->edgeList[j].setEdge(hierachicalGraph->edgeList[j]);
                                break;
                            }
                        }
                    }
                }

            }
        } else {//calculate the connection between cell from and to new

            std::vector<edge> listOfContainingEdges;
            std::vector<int> listOfConnectedVertexes;
            getListofConnectedVertexAndEdges(this->lookUpTableCell[indexCellFrom], listOfContainingEdges,
                                             listOfConnectedVertexes);

            graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
            for (int k = 0;
                 k < this->lookUpTableCell[indexCellFrom].size(); k++) {//add vertices of cell indexCellFrom
                vertex tmpVertex = this->vertexList[this->lookUpTableCell[indexCellFrom][k]];
                currentSubGraph.addVertex(k, tmpVertex.getPositionVertex(), tmpVertex.getRotationVertex(),
                                          tmpVertex.getCovariancePosition(),
                                          tmpVertex.getCovarianceQuaternion(), 0,
                                          graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            }

            for (int k = 0; k < this->lookUpTableCell[indexCellTo].size(); k++) {//add vertices of cell indexCellTo
                vertex tmpVertex = this->vertexList[this->lookUpTableCell[indexCellTo][k]];
                currentSubGraph.addVertex(k + (int) this->lookUpTableCell[indexCellFrom].size(),
                                          tmpVertex.getPositionVertex(),
                                          tmpVertex.getRotationVertex(),
                                          tmpVertex.getCovariancePosition(),
                                          tmpVertex.getCovarianceQuaternion(), 0,
                                          graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            }

            std::vector<int> vertexIndexOfJoinedGraph = joinTwoLists(this->lookUpTableCell[indexCellFrom],
                                                                     this->lookUpTableCell[indexCellTo]);//list of vertex index of cell indexCellFrom and cell indexCellTo
            for (auto &currentEdge : this->edgeList) {//go threw complete edge lists

                auto itFromVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                              vertexIndexOfJoinedGraph.end(),
                                              currentEdge.getFromVertex());
                auto itToVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                            vertexIndexOfJoinedGraph.end(),
                                            currentEdge.getToVertex());

                if (itFromVertex != vertexIndexOfJoinedGraph.end() &&
                    itToVertex !=
                    vertexIndexOfJoinedGraph.end()) {// check if edge is contained in joined graph
                    int indexToVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itToVertex);
                    int indexFromVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itFromVertex);
                    currentSubGraph.addEdge(indexFromVertex, indexToVertex,
                                            currentEdge.getPositionDifference(),
                                            currentEdge.getRotationDifference(),
                                            currentEdge.getCovariancePosition(),
                                            currentEdge.getCovarianceQuaternion(),
                                            graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                }
            }
            std::vector<int> holdStill{0};
            edge resultingEdge = currentSubGraph.getEdgeBetweenNodes(0,
                                                                     this->lookUpTableCell[indexCellFrom].size(),
                                                                     holdStill);
            resultingEdge.setFromVertex(indexCellFrom);
            resultingEdge.setToVertex(indexCellTo);
            bool edgeAlreadyThere = false;
            for (int j = 0; j < hierachicalGraph->edgeList.size(); j++) {
                if (hierachicalGraph->edgeList[j].getFromVertex() == indexCellFrom &&
                    hierachicalGraph->edgeList[j].getToVertex() == indexCellTo ||
                    hierachicalGraph->edgeList[j].getFromVertex() == indexCellTo &&
                    hierachicalGraph->edgeList[j].getToVertex() == indexCellFrom) {
                    hierachicalGraph->edgeList[j].setEdge(hierachicalGraph->edgeList[j]);
                    edgeAlreadyThere = true;
                    break;
                }
            }
            if (!edgeAlreadyThere) {
                if (std::isnan(resultingEdge.getCovarianceQuaternion()) ||
                    std::isnan(resultingEdge.getCovariancePosition()[0]) ||
                    std::isnan(resultingEdge.getCovariancePosition()[1])) {
                    std::cout << "IS NAN: " << std::endl;
                }
                hierachicalGraph->addEdge(indexCellFrom, indexCellTo, resultingEdge.getPositionDifference(),
                                          resultingEdge.getRotationDifference(),
                                          resultingEdge.getCovariancePosition(),
                                          resultingEdge.getCovarianceQuaternion(),
                                          graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            }
        }
    }
}

void
graphSlamSaveStructure::addEdge(const int fromVertex, const int toVertex, const Eigen::Vector3d &positionDifference,
                                const Eigen::Quaterniond &rotationDifference, const Eigen::Vector3d covariancePosition,
                                const double covarianceQuaternion, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
                                int typeOfEdge) {

    if (std::isnan(covarianceQuaternion) ||
        std::isnan(covariancePosition[0]) ||
        std::isnan(covariancePosition[1])) {
        std::cout << "IS NAN: " << std::endl;
    }


    edge edgeToAdd(fromVertex, toVertex, positionDifference, rotationDifference, covariancePosition,
                   covarianceQuaternion,
                   this->degreeOfFreedom, pointCloud, typeOfEdge);
    this->numberOfEdges += 1;
    this->edgeList.push_back(edgeToAdd);
    if (this->hasHierachicalGraph) {
        //get cell from
        int indexCellFrom = this->getCellOfVertexIndex(this->edgeList.back().getFromVertex());
        int indexCellTo = this->getCellOfVertexIndex(this->edgeList.back().getToVertex());

        if (indexCellFrom == indexCellTo) {//calculate every connection new(subgraph)
            std::vector<edge> listOfContainingEdges;
            std::vector<int> listOfConnectedVertexes;
            getListofConnectedVertexAndEdges(this->lookUpTableCell[indexCellFrom], listOfContainingEdges,
                                             listOfConnectedVertexes);
            for (int i = 0; i < this->lookUpTableCell.size(); i++) {
                if (i != indexCellFrom) {

                    bool containingConnection = checkIfElementsOfVectorAreEqual(this->lookUpTableCell[i],
                                                                                listOfConnectedVertexes);//check if overlap exists
                    if (containingConnection) {//create graph from indexCellFrom to i
                        graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
                        for (int k = 0;
                             k < this->lookUpTableCell[indexCellFrom].size(); k++) {//add vertices of cell indexCellFrom
                            vertex tmpVertex = this->vertexList[this->lookUpTableCell[indexCellFrom][k]];
                            currentSubGraph.addVertex(k, tmpVertex.getPositionVertex(), tmpVertex.getRotationVertex(),
                                                      tmpVertex.getCovariancePosition(),
                                                      tmpVertex.getCovarianceQuaternion(), 0,
                                                      graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                        }

                        for (int k = 0; k < this->lookUpTableCell[i].size(); k++) {//add vertices of cell i
                            vertex tmpVertex = this->vertexList[this->lookUpTableCell[i][k]];
                            currentSubGraph.addVertex(k + (int) this->lookUpTableCell[indexCellFrom].size(),
                                                      tmpVertex.getPositionVertex(),
                                                      tmpVertex.getRotationVertex(),
                                                      tmpVertex.getCovariancePosition(),
                                                      tmpVertex.getCovarianceQuaternion(), 0,
                                                      graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                        }

                        std::vector<int> vertexIndexOfJoinedGraph = joinTwoLists(this->lookUpTableCell[indexCellFrom],
                                                                                 this->lookUpTableCell[i]);//list of vertex index of cell indexCellFrom and cell i
                        for (auto &currentEdge : this->edgeList) {//go threw complete edge lists

                            auto itFromVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                                          vertexIndexOfJoinedGraph.end(),
                                                          currentEdge.getFromVertex());
                            auto itToVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                                        vertexIndexOfJoinedGraph.end(),
                                                        currentEdge.getToVertex());

                            if (itFromVertex != vertexIndexOfJoinedGraph.end() &&
                                itToVertex !=
                                vertexIndexOfJoinedGraph.end()) {// check if edge is contained in joined graph
                                int indexToVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itToVertex);
                                int indexFromVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itFromVertex);
                                currentSubGraph.addEdge(indexFromVertex, indexToVertex,
                                                        currentEdge.getPositionDifference(),
                                                        currentEdge.getRotationDifference(),
                                                        currentEdge.getCovariancePosition(),
                                                        currentEdge.getCovarianceQuaternion(),
                                                        graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                            }
                        }
                        std::vector<int> holdStill{0};
                        edge resultingEdge = currentSubGraph.getEdgeBetweenNodes(0,
                                                                                 this->lookUpTableCell[indexCellFrom].size(),
                                                                                 holdStill);
                        resultingEdge.setFromVertex(indexCellFrom);
                        resultingEdge.setToVertex(i);
                        for (int j = 0; j < hierachicalGraph->edgeList.size(); j++) {
                            if (hierachicalGraph->edgeList[j].getFromVertex() == indexCellFrom &&
                                hierachicalGraph->edgeList[j].getToVertex() == i ||
                                hierachicalGraph->edgeList[j].getFromVertex() == i &&
                                hierachicalGraph->edgeList[j].getToVertex() == indexCellFrom) {
                                hierachicalGraph->edgeList[j].setEdge(hierachicalGraph->edgeList[j]);
                                break;
                            }
                        }
                    }
                }

            }
        } else {//calculate the connection between cell from and to new

            std::vector<edge> listOfContainingEdges;
            std::vector<int> listOfConnectedVertexes;
            getListofConnectedVertexAndEdges(this->lookUpTableCell[indexCellFrom], listOfContainingEdges,
                                             listOfConnectedVertexes);

            graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
            for (int k = 0;
                 k < this->lookUpTableCell[indexCellFrom].size(); k++) {//add vertices of cell indexCellFrom
                vertex tmpVertex = this->vertexList[this->lookUpTableCell[indexCellFrom][k]];
                currentSubGraph.addVertex(k, tmpVertex.getPositionVertex(), tmpVertex.getRotationVertex(),
                                          tmpVertex.getCovariancePosition(),
                                          tmpVertex.getCovarianceQuaternion(), 0,
                                          graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            }

            for (int k = 0; k < this->lookUpTableCell[indexCellTo].size(); k++) {//add vertices of cell indexCellTo
                vertex tmpVertex = this->vertexList[this->lookUpTableCell[indexCellTo][k]];
                currentSubGraph.addVertex(k + (int) this->lookUpTableCell[indexCellFrom].size(),
                                          tmpVertex.getPositionVertex(),
                                          tmpVertex.getRotationVertex(),
                                          tmpVertex.getCovariancePosition(),
                                          tmpVertex.getCovarianceQuaternion(), 0,
                                          graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            }

            std::vector<int> vertexIndexOfJoinedGraph = joinTwoLists(this->lookUpTableCell[indexCellFrom],
                                                                     this->lookUpTableCell[indexCellTo]);//list of vertex index of cell indexCellFrom and cell indexCellTo
            for (auto &currentEdge : this->edgeList) {//go threw complete edge lists

                auto itFromVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                              vertexIndexOfJoinedGraph.end(),
                                              currentEdge.getFromVertex());
                auto itToVertex = std::find(vertexIndexOfJoinedGraph.begin(),
                                            vertexIndexOfJoinedGraph.end(),
                                            currentEdge.getToVertex());

                if (itFromVertex != vertexIndexOfJoinedGraph.end() &&
                    itToVertex !=
                    vertexIndexOfJoinedGraph.end()) {// check if edge is contained in joined graph
                    int indexToVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itToVertex);
                    int indexFromVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itFromVertex);
                    currentSubGraph.addEdge(indexFromVertex, indexToVertex,
                                            currentEdge.getPositionDifference(),
                                            currentEdge.getRotationDifference(),
                                            currentEdge.getCovariancePosition(),
                                            currentEdge.getCovarianceQuaternion(),
                                            graphSlamSaveStructure::INTEGRATED_POS_USAGE);
                }
            }
            std::vector<int> holdStill{0};
            edge resultingEdge = currentSubGraph.getEdgeBetweenNodes(0,
                                                                     this->lookUpTableCell[indexCellFrom].size(),
                                                                     holdStill);//resultingEdge.setFromVertex(indexCellFrom); resultingEdge.setToVertex(indexCellTo);
            resultingEdge.setFromVertex(indexCellFrom);
            resultingEdge.setToVertex(indexCellTo);
            bool edgeAlreadyThere = false;
            for (int j = 0; j < hierachicalGraph->edgeList.size(); j++) {
                if (hierachicalGraph->edgeList[j].getFromVertex() == indexCellFrom &&
                    hierachicalGraph->edgeList[j].getToVertex() == indexCellTo ||
                    hierachicalGraph->edgeList[j].getFromVertex() == indexCellTo &&
                    hierachicalGraph->edgeList[j].getToVertex() == indexCellFrom) {
                    hierachicalGraph->edgeList[j].setEdge(hierachicalGraph->edgeList[j]);
                    edgeAlreadyThere = true;
                    break;
                }
            }
            if (!edgeAlreadyThere) {
                if (std::isnan(resultingEdge.getCovarianceQuaternion()) ||
                    std::isnan(resultingEdge.getCovariancePosition()[0]) ||
                    std::isnan(resultingEdge.getCovariancePosition()[1])) {
                    std::cout << "IS NAN: " << std::endl;
                }
                hierachicalGraph->addEdge(indexCellFrom, indexCellTo, resultingEdge.getPositionDifference(),
                                          resultingEdge.getRotationDifference(),
                                          resultingEdge.getCovariancePosition(),
                                          resultingEdge.getCovarianceQuaternion(),
                                          graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            }


        }
    }
}

void graphSlamSaveStructure::addVertex(int vertexNumber, const Eigen::Vector3d &positionVertex,
                                       const Eigen::Quaterniond &rotationVertex,
                                       const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,
                                       double timeStamp,
                                       int typeOfVertex) {

    vertex vertexToAdd(vertexNumber, positionVertex, rotationVertex, this->degreeOfFreedom,
                       covariancePosition, covarianceQuaternion, timeStamp, typeOfVertex);
    this->numberOfVertex += 1;
    this->vertexList.push_back(vertexToAdd);
// look at difference between new vertex and vertex of cell if bigger then cell size then new cell
    if (this->hasHierachicalGraph) {
        //calculate diff between new vertex and old vertex
        Eigen::Vector3d positionDiff =
                this->vertexList[this->lookUpTableCell.back()[0]].getRotationVertex().inverse() *
                this->vertexList.back().getPositionVertex() -
                this->vertexList[this->lookUpTableCell.back()[0]].getRotationVertex().inverse() *
                this->vertexList[this->lookUpTableCell.back()[0]].getPositionVertex();
        if (positionDiff.norm() > this->cellSize) {
            //create new cell
            std::vector<int> tmp;
            this->lookUpTableCell.push_back(tmp);
            this->lookUpTableCell.back().push_back(this->vertexList.back().getVertexNumber());
            this->hierachicalGraph->addVertex((int) (this->lookUpTableCell.size() - 1), positionVertex, rotationVertex,
                                              covariancePosition, covarianceQuaternion, timeStamp,
                                              graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        } else {
            //add to current cell
            this->lookUpTableCell.back().push_back(this->vertexList.back().getVertexNumber());

        }
        // optimize subgraphs, should not be done only if edge is connected
    }
}

void graphSlamSaveStructure::addVertex(int vertexNumber, const Eigen::Vector3d &positionVertex,
                                       const Eigen::Quaterniond &rotationVertex,
                                       const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, double timeStamp,
                                       int typeOfVertex) {

    vertex vertexToAdd(vertexNumber, positionVertex, rotationVertex, this->degreeOfFreedom,
                       pointCloud, covariancePosition, covarianceQuaternion, timeStamp, typeOfVertex);
    this->numberOfVertex += 1;
    this->vertexList.push_back(vertexToAdd);
// look at difference between new vertex and vertex of cell if bigger then cell size then new cell
    if (this->hasHierachicalGraph) {
        //calculate diff between new vertex and old vertex
        Eigen::Vector3d positionDiff =
                this->vertexList[this->lookUpTableCell.back()[0]].getRotationVertex().inverse() *
                this->vertexList.back().getPositionVertex() -
                this->vertexList[this->lookUpTableCell.back()[0]].getRotationVertex().inverse() *
                this->vertexList[this->lookUpTableCell.back()[0]].getPositionVertex();
        if (positionDiff.norm() > this->cellSize) {
            //create new cell
            std::vector<int> tmp;
            this->lookUpTableCell.push_back(tmp);
            this->lookUpTableCell.back().push_back(this->vertexList.back().getVertexNumber());
            this->hierachicalGraph->addVertex((int) (this->lookUpTableCell.size() - 1), positionVertex, rotationVertex,
                                              covariancePosition, covarianceQuaternion, timeStamp,
                                              graphSlamSaveStructure::INTEGRATED_POS_USAGE);

        } else {
            //add to current cell
            this->lookUpTableCell.back().push_back(this->vertexList.back().getVertexNumber());
        }
        // optimize subgraphs, should not be done only if edge is connected
    }
}

Eigen::SparseMatrix<double> graphSlamSaveStructure::getInformationMatrix() {
    Eigen::SparseMatrix<double> informationMatrix(
            this->numberOfEdges * this->degreeOfFreedom,
            this->numberOfEdges * this->degreeOfFreedom);

    for (int i = 0; i < this->numberOfEdges; i++) {
        informationMatrix.insert(i * this->degreeOfFreedom + 0,
                                 i * this->degreeOfFreedom + 0) =
                1 / std::pow(this->edgeList[i].getCovariancePosition()[0], 2);//from std to var

        informationMatrix.insert(i * this->degreeOfFreedom + 1,
                                 i * this->degreeOfFreedom + 1) =
                1 / std::pow(this->edgeList[i].getCovariancePosition()[1], 2);//from std to var

        informationMatrix.insert(i * this->degreeOfFreedom + 2,
                                 i * this->degreeOfFreedom + 2) =
                1 / std::pow(this->edgeList[i].getCovarianceQuaternion(), 2);//from std to var

        if (std::isnan(this->edgeList[i].getCovarianceQuaternion()) ||
            std::isnan(this->edgeList[i].getCovariancePosition()[0]) ||
            std::isnan(this->edgeList[i].getCovariancePosition()[1])) {
            std::cout << "IS NAN: " << std::endl;
        }
    }
    return informationMatrix;
}


Eigen::SparseMatrix<double> graphSlamSaveStructure::getJacobianMatrix() {
//    Eigen::MatrixXd jacobianMatrix = Eigen::MatrixXd::Zero(
//            this->numberOfEdges * this->degreeOfFreedom,
//            this->numberOfVertex * this->degreeOfFreedom);
    Eigen::SparseMatrix<double> jacobianMatrix(
            this->numberOfEdges * this->degreeOfFreedom,
            this->numberOfVertex * this->degreeOfFreedom);

    for (int i = 0; i < this->numberOfEdges; i++) {

        int fromVertex = this->edgeList[i].getFromVertex();
        int toVertex = this->edgeList[i].getToVertex();

        double alpha1 = this->getYawAngle(this->vertexList[fromVertex].getRotationVertex());
        double x1 = this->vertexList[fromVertex].getPositionVertex().x();
        double y1 = this->vertexList[fromVertex].getPositionVertex().y();

        double x2 = this->vertexList[toVertex].getPositionVertex().x();
        double y2 = this->vertexList[toVertex].getPositionVertex().y();


//        //first row
//
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
//                              fromVertex * this->degreeOfFreedom + 0) = -cos(
//                alpha1);//first collumn of fromVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
//                              fromVertex * this->degreeOfFreedom + 1) = -sin(
//                alpha1);//second collumn of fromVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
//                              fromVertex * this->degreeOfFreedom + 2) =
//                -sin(alpha1) * (x2 - x1) + cos(alpha1) * (y2 - y1);//third collumn of fromVertex
//
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
//                              toVertex * this->degreeOfFreedom + 0) = cos(
//                alpha1);//first collumn of toVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
//                              toVertex * this->degreeOfFreedom + 1) = sin(
//                alpha1);//second collumn of toVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
//                              toVertex * this->degreeOfFreedom + 2) = 0;//third collumn of toVertex
//
//
//        //second row
//
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
//                              fromVertex * this->degreeOfFreedom + 0) = sin(
//                alpha1);//first collumn of fromVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
//                              fromVertex * this->degreeOfFreedom + 1) = -cos(
//                alpha1);//second collumn of fromVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
//                              fromVertex * this->degreeOfFreedom + 2) =
//                -cos(alpha1) * (x2 - x1) - sin(alpha1) * (y2 - y1);//third collumn of fromVertex
//
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
//                              toVertex * this->degreeOfFreedom + 0) = -sin(
//                alpha1);//first collumn of toVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
//                              toVertex * this->degreeOfFreedom + 1) = cos(
//                alpha1);//second collumn of toVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
//                              toVertex * this->degreeOfFreedom + 2) = 0;//third collumn of toVertex
//
//
//        //third row
//
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
//                              fromVertex * this->degreeOfFreedom +
//                              0) = 0;//first collumn of fromVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
//                              fromVertex * this->degreeOfFreedom +
//                              1) = 0;//second collumn of fromVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
//                              fromVertex * this->degreeOfFreedom +
//                              2) = -1;//third collumn of fromVertex
//
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
//                              toVertex * this->degreeOfFreedom + 0) = 0;//first collumn of toVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
//                              toVertex * this->degreeOfFreedom + 1) = 0;//second collumn of toVertex
//        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
//                              toVertex * this->degreeOfFreedom + 2) = 1;//third collumn of toVertex

        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
                              fromVertex * this->degreeOfFreedom + 0) = cos(
                alpha1);//first collumn of fromVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
                              fromVertex * this->degreeOfFreedom + 1) = sin(
                alpha1);//second collumn of fromVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
                              fromVertex * this->degreeOfFreedom + 2) =
                sin(alpha1) * (x2 - x1) + cos(alpha1) * (y1 - y2);//third collumn of fromVertex

        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
                              toVertex * this->degreeOfFreedom + 0) = -cos(
                alpha1);//first collumn of toVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
                              toVertex * this->degreeOfFreedom + 1) = -sin(
                alpha1);//second collumn of toVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 0,
                              toVertex * this->degreeOfFreedom + 2) = 0;//third collumn of toVertex


        //second row

        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
                              fromVertex * this->degreeOfFreedom + 0) = -sin(
                alpha1);//first collumn of fromVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
                              fromVertex * this->degreeOfFreedom + 1) = cos(
                alpha1);//second collumn of fromVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
                              fromVertex * this->degreeOfFreedom + 2) =
                cos(alpha1) * (x2 - x1) + sin(alpha1) * (y2 - y1);//third collumn of fromVertex

        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
                              toVertex * this->degreeOfFreedom + 0) = sin(
                alpha1);//first collumn of toVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
                              toVertex * this->degreeOfFreedom + 1) = -cos(
                alpha1);//second collumn of toVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 1,
                              toVertex * this->degreeOfFreedom + 2) = 0;//third collumn of toVertex


        //third row

        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
                              fromVertex * this->degreeOfFreedom +
                              0) = 0;//first collumn of fromVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
                              fromVertex * this->degreeOfFreedom +
                              1) = 0;//second collumn of fromVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
                              fromVertex * this->degreeOfFreedom +
                              2) = 1;//third collumn of fromVertex

        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
                              toVertex * this->degreeOfFreedom + 0) = 0;//first collumn of toVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
                              toVertex * this->degreeOfFreedom + 1) = 0;//second collumn of toVertex
        jacobianMatrix.insert(i * this->degreeOfFreedom + 2,
                              toVertex * this->degreeOfFreedom + 2) = -1;//third collumn of toVertex

    }

    return jacobianMatrix;
}

double angleDiff(double first, double second) {//first-second
    return atan2(sin(first - second), cos(first - second));
}

Eigen::MatrixXd graphSlamSaveStructure::getErrorMatrix() {

    //this is for DOF 3
    Eigen::MatrixXd error = Eigen::MatrixXd::Zero(
            this->numberOfEdges * this->degreeOfFreedom,
            1);//add an additional size for x0
    for (int i = 0; i < this->numberOfEdges; i++) {

        int fromVertex = this->edgeList[i].getFromVertex();
        int toVertex = this->edgeList[i].getToVertex();

        double yawAngle = this->getYawAngle(this->vertexList[fromVertex].getRotationVertex());


        Eigen::AngleAxisd rotation_vector1(-yawAngle, Eigen::Vector3d(0, 0, 1));//@TODO should this not be minus?!
        Eigen::Matrix2d currentRotation = rotation_vector1.matrix().block<2, 2>(0, 0);


        Eigen::Vector2d differenceBeforeRotation =
                this->vertexList[toVertex].getPositionVertex().block<2, 1>(0, 0) -
                this->vertexList[fromVertex].getPositionVertex().block<2, 1>(0, 0);


        error.block<2, 1>(i * this->degreeOfFreedom, 0) =
                this->edgeList[i].getPositionDifference().block<2, 1>(0, 0) -
                currentRotation * differenceBeforeRotation;
        //calculate angle difference

        double angleDiffVertex = angleDiff(this->getYawAngle(this->vertexList[toVertex].getRotationVertex()),yawAngle);

        double angleErrorDiff = angleDiff( this->getYawAngle(this->edgeList[i].getRotationDifference()),angleDiffVertex);
        //tf2::getYaw(this->edgeList[i].getRotationDifference());
        //Eigen::Quaterniond tmpQuat();

        //tmpQuat = ;
        error(i * this->degreeOfFreedom + 2, 0) = angleErrorDiff;
    }
    return error;
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

    std::cout << "Number of Edges:" << this->numberOfEdges << std::endl;
    for (int i = 0; i < this->numberOfEdges; i++) {
        std::cout << "Edge from " << this->edgeList[i].getFromVertex() << " to "
                  << this->edgeList[i].getToVertex() << " covariance Position "
                  << this->edgeList[i].getCovariancePosition() << std::endl;
    }

}

void graphSlamSaveStructure::addToEveryState(std::vector<Eigen::Vector3d> &positionDifferenceVector,
                                             std::vector<Eigen::Quaterniond> &rotationDifferenceVector) {


    for (int i = 0; i < this->numberOfVertex; i++) {
        Eigen::Vector3d currentStatePos = this->vertexList[i].getPositionVertex();
        Eigen::Quaterniond currentStateRotation = this->vertexList[i].getRotationVertex();

        this->vertexList[i].setPositionVertex(currentStatePos + positionDifferenceVector[i]);

        this->vertexList[i].setRotationVertex(currentStateRotation * rotationDifferenceVector[i]);
    }
}

void graphSlamSaveStructure::addToState(Eigen::MatrixXd &vectorToAdd) {


    for (int i = 0; i < this->numberOfVertex; i++) {
        double x = vectorToAdd(i * this->degreeOfFreedom + 0);
        double y = vectorToAdd(i * this->degreeOfFreedom + 1);
        double alpha = vectorToAdd(i * this->degreeOfFreedom + 2);

        Eigen::Vector3d currentStatePos = this->vertexList[i].getPositionVertex();
        Eigen::Quaterniond currentStateRotation = this->vertexList[i].getRotationVertex();


        Eigen::Vector3d addedStatePos(x, y, 0);
        Eigen::AngleAxisd rotation_vector1(alpha, Eigen::Vector3d(0, 0, 1));
        Eigen::Quaterniond addedRotation(rotation_vector1);


        this->vertexList[i].setPositionVertex(currentStatePos + addedStatePos);

        this->vertexList[i].setRotationVertex(currentStateRotation * addedRotation);
    }
}

vertex *graphSlamSaveStructure::getVertexByIndex(const int i) {
    if (i >= this->numberOfVertex) {
        std::cout << "access to a vertex that doesnt exist" << std::endl;
        std::exit(-1);
    }
    return &this->vertexList[i];
}

std::vector<vertex> graphSlamSaveStructure::getVertexList() {
    return this->vertexList;
}

std::vector<edge> *graphSlamSaveStructure::getEdgeList() {
    return &this->edgeList;
}

void graphSlamSaveStructure::optimizeGraphWithSlamTopDown(bool verbose, double cellSize) {
    if (this->hasHierachicalGraph) {
        this->hierachicalGraph->optimizeGraphWithSlamTopDown(verbose, this->cellSize);
        //std::vector<vertex> stateAfterOptimization = this->hierachicalGraph->getVertexList();
        //test if difference between x(k) und x(k-1) is "big" if yes: change graph k-1 else do nothing
        std::vector<int> cellListToUpdate;
        for (auto &currentHierachicalSupGraphCell : this->hierachicalGraph->vertexList) {// get the vertex number
            Eigen::Vector3d diffPosition = currentHierachicalSupGraphCell.getPositionVertex() -
                                           this->vertexList[this->lookUpTableCell[currentHierachicalSupGraphCell.getVertexNumber()][0]].getPositionVertex();
            Eigen::Quaterniond diffRotation = currentHierachicalSupGraphCell.getRotationVertex() *
                                              this->vertexList[this->lookUpTableCell[currentHierachicalSupGraphCell.getVertexNumber()][0]].getRotationVertex().inverse();
            if (diffPosition.norm() > cellSize * 0.01) {//@TODO change this to dependent on the cell size
                //propagate down
                for (int indexVertex : this->lookUpTableCell[currentHierachicalSupGraphCell.getVertexNumber()]) {// also is done for every cell member
                    this->vertexList[indexVertex].setPositionVertex(
                            this->vertexList[indexVertex].getPositionVertex() + diffPosition);
                    this->vertexList[indexVertex].setRotationVertex(
                            diffRotation * this->vertexList[indexVertex].getRotationVertex());
                }
                cellListToUpdate.push_back(currentHierachicalSupGraphCell.getVertexNumber());
            }
        }
        for (auto cellToUpdate:cellListToUpdate) {
            // should optimize sub graph anew keep both keyframes between them still
            for (int i = 0; i < this->hierachicalGraph->edgeList.size(); i++) {
                if (this->hierachicalGraph->edgeList[i].getFromVertex() == cellToUpdate) {
                    graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
                    bool connectionExists = createSubGraphBetweenCell(cellToUpdate,
                                                                      this->hierachicalGraph->edgeList[i].getToVertex(),
                                                                      currentSubGraph);
                    if (connectionExists) {
                        //optimize graph and calculate the edge between the two representing nodes
                        //representing nodes are: first of currentSubGraph and cellVertexLookUpList[i].size()
                        std::vector<int> holdStill{0, (int) this->lookUpTableCell[cellToUpdate].size()};
                        currentSubGraph.optimizeGraphWithSlam(false, holdStill);
                        int j = 0;
                        for (auto vertexIndex:this->lookUpTableCell[this->hierachicalGraph->edgeList[i].getFromVertex()]) {
                            this->vertexList[vertexIndex].setPositionVertex(
                                    currentSubGraph.getVertexByIndex(j)->getPositionVertex());
                            this->vertexList[vertexIndex].setRotationVertex(
                                    currentSubGraph.getVertexByIndex(j)->getRotationVertex());
                            j++;
                        }
                        //j=0;
                        for (auto vertexIndex:this->lookUpTableCell[this->hierachicalGraph->edgeList[i].getToVertex()]) {
                            this->vertexList[vertexIndex].setPositionVertex(
                                    currentSubGraph.getVertexByIndex(j)->getPositionVertex());
                            this->vertexList[vertexIndex].setRotationVertex(
                                    currentSubGraph.getVertexByIndex(j)->getRotationVertex());
                            j++;
                        }


                    } else {
                        std::cout << "this should never happen" << std::endl;
                        exit(-1);
                    }

                } else {
                    if (this->hierachicalGraph->edgeList[i].getToVertex() == cellToUpdate) {
                        graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
                        bool connectionExists = createSubGraphBetweenCell(
                                this->hierachicalGraph->edgeList[i].getFromVertex(),
                                cellToUpdate, currentSubGraph);
                        if (connectionExists) {
                            //optimize graph and calculate the edge between the two representing nodes
                            //representing nodes are: first of currentSubGraph and cellVertexLookUpList[i].size()
                            std::vector<int> holdStill{0,
                                                       (int) this->lookUpTableCell[this->hierachicalGraph->edgeList[i].getFromVertex()].size()};
                            //edge resultingEdge = currentSubGraph.getEdgeBetweenNodes(0, holdStill[1], holdStill);

                            currentSubGraph.optimizeGraphWithSlam(false, holdStill);

                            int j = 0;
                            for (auto vertexIndex:this->lookUpTableCell[this->hierachicalGraph->edgeList[i].getFromVertex()]) {
                                this->vertexList[vertexIndex].setPositionVertex(
                                        currentSubGraph.getVertexByIndex(j)->getPositionVertex());
                                this->vertexList[vertexIndex].setRotationVertex(
                                        currentSubGraph.getVertexByIndex(j)->getRotationVertex());
                                j++;
                            }
                            //j=0;
                            for (auto vertexIndex:this->lookUpTableCell[this->hierachicalGraph->edgeList[i].getToVertex()]) {
                                this->vertexList[vertexIndex].setPositionVertex(
                                        currentSubGraph.getVertexByIndex(j)->getPositionVertex());
                                this->vertexList[vertexIndex].setRotationVertex(
                                        currentSubGraph.getVertexByIndex(j)->getRotationVertex());
                                j++;
                            }


                        } else {
                            std::cout << "this should never happen" << std::endl;
                            exit(-1);
                        }
                    }
                }
            }
        }


        // next the covariance should be added at lower levels
        for (auto &currentSubGraphVertex : this->hierachicalGraph->getVertexList()) {// get the vertex list
            for (int indexVertex : this->lookUpTableCell[currentSubGraphVertex.getVertexNumber()]) {// change the covariance for each cellmember dependend on the covariance of vertex
                this->vertexList[indexVertex].setCovariancePosition(currentSubGraphVertex.getCovariancePosition());
                this->vertexList[indexVertex].setCovarianceQuaternion(
                        currentSubGraphVertex.getCovarianceQuaternion());
            }
        }


    } else {
        std::vector<int> holdStill{0};
        this->optimizeGraphWithSlam(verbose, holdStill);
    }

}

void printInformationAboutMatrix(Eigen::SparseMatrix<double> matrix, Eigen::MatrixXd bMatrix) {

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
    //std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    std::cout << "The Rank is:" << std::endl << svd.rank() << std::endl;
    std::cout << "The rows is:" << std::endl << svd.rows() << std::endl;
    std::cout << "The cols is:" << std::endl << svd.cols() << std::endl;


    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > solveForxTest(matrix);
    std::cout << "The Determinant is:" << std::endl << solveForxTest.determinant() << std::endl;
    Eigen::MatrixXd vectorToAddTest;
    vectorToAddTest = solveForxTest.solve(-bMatrix);
    std::cout << "The Solution to hx=-b is norm:" << std::endl << vectorToAddTest.norm() << std::endl;
//

}

void graphSlamSaveStructure::removeLastEdge() {
    this->edgeList.pop_back();
    this->numberOfEdges = this->numberOfEdges - 1;
}


void graphSlamSaveStructure::optimizeGraphWithSlam(bool verbose, std::vector<int> &holdStill) {
    // if hold still is bigger than 1 then add an edge to the graph, which fixes the graph
    if (holdStill.size() > 1) {
        for (int i = 1; i < holdStill.size(); i++) {
            Eigen::Matrix4d currentTMPTransformation = this->vertexList[0].getTransformation().inverse() *
                                                       this->vertexList[holdStill[i]].getTransformation();
            this->addEdge(0, holdStill[i], Eigen::Vector3d(currentTMPTransformation.block<3, 1>(0, 3)),
                          Eigen::Quaterniond(currentTMPTransformation.block<3, 3>(0, 0)),
                          Eigen::Vector3d(0.0001, 0.0001, 0), 0.0001, graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        }
    }

    Eigen::MatrixXd errorMatrix;
    Eigen::SparseMatrix<double> informationMatrix;
    Eigen::SparseMatrix<double> jacobianMatrix;
    Eigen::MatrixXd bMatrix;
    Eigen::SparseMatrix<double> hMatrix;
    Eigen::SparseMatrix<double> hMatrixInverse;//without last pose
    Eigen::MatrixXd vectorToAdd;
    Eigen::VectorXd covarianzeSquared;
    const double gainVector = 0.9;
    double errorLast = 100000;
    double multiplicator;

    for (int i = 0; i < 20; i++) {

        informationMatrix = this->getInformationMatrix();
        errorMatrix = this->getErrorMatrix();
        jacobianMatrix = this->getJacobianMatrix();
        bMatrix = (errorMatrix.transpose() * informationMatrix * jacobianMatrix).transpose();
        hMatrix = jacobianMatrix.transpose() * informationMatrix * jacobianMatrix;
        //hMatrix.block<3, 3>(0, 0) = hMatrix.block<3, 3>(0, 0) + Eigen::MatrixXd::Identity(3, 3);

        //test scaling
        multiplicator = 1;//1 / ((double) hMatrix.rows());
        hMatrix = multiplicator * hMatrix;
        bMatrix = multiplicator * bMatrix;

        hMatrix.coeffRef(holdStill[0] * this->degreeOfFreedom + 0,
                         holdStill[0] * this->degreeOfFreedom + 0) =
                hMatrix.coeff(holdStill[0] * this->degreeOfFreedom + 0,
                              holdStill[0] * this->degreeOfFreedom + 0) +
                1;
        hMatrix.coeffRef(holdStill[0] * this->degreeOfFreedom + 1,
                         holdStill[0] * this->degreeOfFreedom + 1) =
                hMatrix.coeff(holdStill[0] * this->degreeOfFreedom + 1,
                              holdStill[0] * this->degreeOfFreedom + 1) + 1;
        hMatrix.coeffRef(holdStill[0] * this->degreeOfFreedom + 2,
                         holdStill[0] * this->degreeOfFreedom + 2) =
                hMatrix.coeff(holdStill[0] * this->degreeOfFreedom + 2,
                              holdStill[0] * this->degreeOfFreedom + 2) + 1;
        //}
//        Eigen::SparseMatrix<double> identityAddingTest(hMatrix.rows(),
//                                                      hMatrix.cols());
//        for (int k = 0; k < hMatrix.rows(); k++) {
//            identityAddingTest.insert(k, k) = hMatrix.rows();
//        }
        //hMatrix = hMatrix+identityAddingTest;

//        std::cout << "bMatrix:" << std::endl;
//        std::cout << bMatrix << std::endl;
//        std::cout << "hMatrix:" << std::endl;
//        std::cout << hMatrix.toDense() << std::endl;
        if (std::isnan(errorMatrix.norm())) {
            std::cout << "IS NAN: " << std::endl;
            std::cout << "Minimized at error: " << errorMatrix.norm() << std::endl;
        }
        //vectorToAdd = gainVector * hMatrix.colPivHouseholderQr().solve(-bMatrix);

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > solveForx(hMatrix);

        if (solveForx.info() != Eigen::Success) {
            std::cout << solveForx.info() << std::endl;
        }
        vectorToAdd = gainVector * solveForx.solve(-bMatrix);

//        hMatrix.block<3, 3>(0, 0) = hMatrix.block<3, 3>(0, 0) - Eigen::MatrixXd::Identity(3, 3);
        if (vectorToAdd.norm() > 1000 || std::isnan(errorMatrix.norm())) {
            printInformationAboutMatrix(hMatrix, bMatrix);
            Eigen::SparseMatrix<double> hMatrixTest1, hMatrixTest2, hMatrixTest3, hMatrixTest4;
            hMatrixTest1 = jacobianMatrix.transpose() * jacobianMatrix;
            hMatrixTest2 = jacobianMatrix.transpose() * informationMatrix * jacobianMatrix;
            multiplicator = 1 / ((double) hMatrixTest2.rows());
            hMatrixTest3 = multiplicator * hMatrixTest2;
            //test just hMatrixTest
            printInformationAboutMatrix(hMatrixTest1, bMatrix);
            printInformationAboutMatrix(hMatrixTest2, bMatrix);
            printInformationAboutMatrix(hMatrixTest3, multiplicator * bMatrix);

            hMatrixTest1.coeffRef(0, 0) = hMatrixTest1.coeff(0, 0) + 1;//add identity matrix to solve equation
            hMatrixTest1.coeffRef(1, 1) = hMatrixTest1.coeff(1, 1) + 1;
            hMatrixTest1.coeffRef(2, 2) = hMatrixTest1.coeff(2, 2) + 1;

            hMatrixTest2.coeffRef(0, 0) = hMatrixTest2.coeff(0, 0) + 1;//add identity matrix to solve equation
            hMatrixTest2.coeffRef(1, 1) = hMatrixTest2.coeff(1, 1) + 1;
            hMatrixTest2.coeffRef(2, 2) = hMatrixTest2.coeff(2, 2) + 1;

            hMatrixTest3.coeffRef(0, 0) = hMatrixTest3.coeff(0, 0) + 1;//add identity matrix to solve equation
            hMatrixTest3.coeffRef(1, 1) = hMatrixTest3.coeff(1, 1) + 1;
            hMatrixTest3.coeffRef(2, 2) = hMatrixTest3.coeff(2, 2) + 1;

            printInformationAboutMatrix(hMatrixTest1, bMatrix);
            printInformationAboutMatrix(hMatrixTest2, bMatrix);
            printInformationAboutMatrix(hMatrixTest3, multiplicator * bMatrix);
        }
        if (verbose) {
            std::cout << "hMatrix determinant" << std::endl;
            std::cout << hMatrix.toDense().determinant() << std::endl;
            std::cout << "hMatrix:" << std::endl;
            std::cout << hMatrix.toDense() << std::endl;
            if (std::isinf(hMatrix.toDense().determinant())) {
                std::cout << "hMatrix determinant" << std::endl;
                std::cout << hMatrix.toDense().determinant() << std::endl;
                std::cout << "hMatrix:" << std::endl;
                std::cout << hMatrix.toDense() << std::endl;
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(hMatrix.toDense());
                std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
            }
        }

        //for (auto &holdStill[0] : holdStill) {//remove identity matrix to solve equation
        hMatrix.coeffRef(holdStill[0] * this->degreeOfFreedom + 0,
                         holdStill[0] * this->degreeOfFreedom + 0) =
                hMatrix.coeff(holdStill[0] * this->degreeOfFreedom + 0,
                              holdStill[0] * this->degreeOfFreedom + 0) -
                1;//add identity matrix to solve equation
        hMatrix.coeffRef(holdStill[0] * this->degreeOfFreedom + 1,
                         holdStill[0] * this->degreeOfFreedom + 1) =
                hMatrix.coeff(holdStill[0] * this->degreeOfFreedom + 1,
                              holdStill[0] * this->degreeOfFreedom + 1) - 1;
        hMatrix.coeffRef(holdStill[0] * this->degreeOfFreedom + 2,
                         holdStill[0] * this->degreeOfFreedom + 2) =
                hMatrix.coeff(holdStill[0] * this->degreeOfFreedom + 2,
                              holdStill[0] * this->degreeOfFreedom + 2) - 1;
        //hMatrix = hMatrix-identityAddingTest;
        if (this->degreeOfFreedom == 3) {
            vectorToAdd(holdStill[0] * this->degreeOfFreedom + 0) = 0;
            vectorToAdd(holdStill[0] * this->degreeOfFreedom + 1) = 0;
            vectorToAdd(holdStill[0] * this->degreeOfFreedom + 2) = 0;
        }
        //}
        this->addToState(vectorToAdd);

        if (verbose) {
            std::cout << "VectorXd size:" << std::endl;
            std::cout << covarianzeSquared.size() << std::endl;
            std::cout << "covarianzeSquared:" << std::endl;
            std::cout << covarianzeSquared << std::endl;
            std::cout << "informationMatrix:" << std::endl;
            std::cout << informationMatrix << std::endl;
            std::cout << "jacobianMatrix:" << std::endl;
            std::cout << jacobianMatrix << std::endl;
            std::cout << "bMatrix:" << std::endl;
            std::cout << bMatrix << std::endl;
            std::cout << "hMatrix:" << std::endl;
            std::cout << hMatrix.toDense() << std::endl;
            std::cout << "hMatrix determinant" << std::endl;
            std::cout << hMatrix.toDense().determinant() << std::endl;
            std::cout << "hMatrix Inversed diagonal:" << std::endl;
            std::cout << covarianzeSquared << std::endl;
            std::cout << "solve Hx=-b:" << std::endl;
            std::cout << vectorToAdd << std::endl;
            std::cout << "complete error: " << errorMatrix.norm() << std::endl;
            std::cout << "i = " << i << std::endl;
        }

        if (0.000001 > std::abs(errorLast - errorMatrix.norm()) && std::abs(errorMatrix.norm()) < 0.0001 * i * i) {
            if (verbose) { std::cout << "out at i = " << i << std::endl; }
            //std::cout << "out at i = " << i << std::endl;
            break;
        }
        errorLast = errorMatrix.norm();
    }

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > findHInv(
            hMatrix.block(0, 0, ((this->numberOfVertex - 1) *
                                 this->degreeOfFreedom),
                          ((this->numberOfVertex - 1) *
                           this->degreeOfFreedom)));

    Eigen::SparseMatrix<double> identityMatrix((this->numberOfVertex - 1) *
                                               this->degreeOfFreedom,
                                               ((this->numberOfVertex - 1) *
                                                this->degreeOfFreedom));

    for (int k = 0;
         k < (this->numberOfVertex - 1) * this->degreeOfFreedom; k++) {
        identityMatrix.insert(k, k) = 1;
    }

    hMatrixInverse = findHInv.solve(multiplicator * identityMatrix);
    covarianzeSquared = hMatrixInverse.diagonal();
    //std::cout << "covariance Squared" << covarianzeSquared << std::endl;

    for (int i = 0; i < this->numberOfVertex - 1; i++) {//apply the covariance to the current graph
        this->vertexList[i].setCovarianceQuaternion(
                sqrt(covarianzeSquared[i * this->degreeOfFreedom + 2]));
        Eigen::Vector3d covariancePos(sqrt(covarianzeSquared[i * this->degreeOfFreedom + 0]),
                                      sqrt(covarianzeSquared[i * this->degreeOfFreedom + 1]), 0);
        this->vertexList[i].setCovariancePosition(covariancePos);
    }
    if (verbose) { std::cout << "Minimized at error: " << errorMatrix.norm() << std::endl; }

    //remove the added edge in the end
    if (holdStill.size() > 1) {
        for (int i = 1; i < holdStill.size(); i++) {
            this->removeLastEdge();
        }
    }
}


void graphSlamSaveStructure::lookupTableCreation(double minDistanceForNewCell) {

    this->lookUpTableCell.clear();
    std::vector<int> tmp1;
    this->lookUpTableCell.push_back(tmp1);
    this->lookUpTableCell[0].push_back(this->vertexList[0].getVertexNumber());
    int indexCell = 0;
    for (int i = 1; i < this->numberOfVertex; i++) {//starting at i = 1 vertex
        // calculate diff between first node of current cell, and i'th vertex
        Eigen::Vector3d positionDiff =
                this->vertexList[this->lookUpTableCell[indexCell][0]].getRotationVertex().inverse() *
                this->vertexList[i].getPositionVertex() -
                this->vertexList[this->lookUpTableCell[indexCell][0]].getRotationVertex().inverse() *
                this->vertexList[this->lookUpTableCell[indexCell][0]].getPositionVertex();
        if (positionDiff.norm() > minDistanceForNewCell) {
            //create new cell
            indexCell++;
            std::vector<int> tmp;
            this->lookUpTableCell.push_back(tmp);
            this->lookUpTableCell[indexCell].push_back(this->vertexList[i].getVertexNumber());
        } else {
            //add to current cell
            this->lookUpTableCell[indexCell].push_back(this->vertexList[i].getVertexNumber());
        }
    }
}

void graphSlamSaveStructure::initiallizeSubGraphs(
        std::deque<double> cellSizes) {//this is useful for the beginning of innitialization

    this->createHierachicalGraph(cellSizes[0]);
    cellSizes.pop_front();
    if (!cellSizes.empty()) {
        this->hierachicalGraph->initiallizeSubGraphs(cellSizes);
    }
}

void graphSlamSaveStructure::getListofConnectedVertexAndEdges(std::vector<int> &vertexIndicesofICell,
                                                              std::vector<edge> &listOfContainingEdges,
                                                              std::vector<int> &listOfConnectedVertexes) {//go threw cell I and save edges connected to it
//check if vertex is in edge, if true save the edge
    for (auto &currentEdge : this->edgeList) {// get the vertex number

        if (std::find(vertexIndicesofICell.begin(), vertexIndicesofICell.end(),
                      currentEdge.getFromVertex()) != vertexIndicesofICell.end() ||
            std::find(vertexIndicesofICell.begin(), vertexIndicesofICell.end(),
                      currentEdge.getToVertex()) !=
            vertexIndicesofICell.end()) {//check if edge (from/to) is contained in vertex list
/* edge contains vertex of interest */
            listOfContainingEdges.push_back(currentEdge);
            listOfConnectedVertexes.push_back(currentEdge.getToVertex());
            listOfConnectedVertexes.push_back(currentEdge.getFromVertex());
        }
    }
}

void graphSlamSaveStructure::createHierachicalGraph(double cellSizeDes) {
    this->cellSize = cellSizeDes;
    this->lookupTableCreation(this->cellSize);//creating each cell with containing graph
    hierachicalGraph = new graphSlamSaveStructure(this->degreeOfFreedom);
    // create hierachical graph

    for (int i = 0; i < this->lookUpTableCell.size(); i++) {
        // sort in into vertex list
        vertex addedVertex = this->vertexList[this->lookUpTableCell[i][0]];
        hierachicalGraph->addVertex(i, addedVertex.getPositionVertex(), addedVertex.getRotationVertex(),
                                    addedVertex.getCovariancePosition(), addedVertex.getCovarianceQuaternion(),
                                    addedVertex.getTimeStamp(),
                                    graphSlamSaveStructure::INTEGRATED_POS_USAGE);
    }

    for (int i = 0; i < this->lookUpTableCell.size(); i++) {// go threw vertex list of cells i current cell
        for (int j = i + 1; j < this->lookUpTableCell.size(); j++) {// go threw rest cells to find connecting edges
            graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
            bool connectionExists = createSubGraphBetweenCell(i, j, currentSubGraph);
            if (connectionExists) {
                //optimize graph and calculate the edge between the two representing nodes
                //representing nodes are: first of currentSubGraph and cellVertexLookUpList[i].size()
                std::vector<int> holdStill{0};
                edge resultingEdge = currentSubGraph.getEdgeBetweenNodes(0, this->lookUpTableCell[i].size(), holdStill);
                hierachicalGraph->addEdge(i, j, resultingEdge.getPositionDifference(),
                                          resultingEdge.getRotationDifference(), resultingEdge.getCovariancePosition(),
                                          resultingEdge.getCovarianceQuaternion(),
                                          graphSlamSaveStructure::INTEGRATED_POS_USAGE);
            }
        }
    }
    this->hasHierachicalGraph = true;
}


edge graphSlamSaveStructure::getEdgeBetweenNodes(int fromVertex, int toVertex, std::vector<int> &holdStill) {
    this->optimizeGraphWithSlam(false, holdStill);
    Eigen::MatrixXd errorMatrix;
    Eigen::SparseMatrix<double> informationMatrix;
    Eigen::SparseMatrix<double> jacobianMatrix;
    Eigen::MatrixXd bMatrix;
    Eigen::SparseMatrix<double> hMatrix;
    Eigen::SparseMatrix<double> hMatrixInverse;// without last pose
    Eigen::MatrixXd vectorToAdd;
    Eigen::VectorXd covarianzeSquared;

    informationMatrix = this->getInformationMatrix();
    errorMatrix = this->getErrorMatrix();
    jacobianMatrix = this->getJacobianMatrix();

    hMatrix = jacobianMatrix.transpose() * informationMatrix * jacobianMatrix;

    removeRowColumn(hMatrix, fromVertex);

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > findHInv(hMatrix);

    Eigen::SparseMatrix<double> identityMatrix((this->numberOfVertex - 1) *
                                               this->degreeOfFreedom,
                                               ((this->numberOfVertex - 1) *
                                                this->degreeOfFreedom));

    for (int k = 0;
         k < (this->numberOfVertex - 1) * this->degreeOfFreedom; k++) {
        identityMatrix.insert(k, k) = 1;
    }

    hMatrixInverse = findHInv.solve(identityMatrix);
    covarianzeSquared = hMatrixInverse.diagonal();
//    std::cout << "covarianzeSquared:" << std::endl;
//    std::cout << covarianzeSquared << std::endl;
    for (int i = 0; i <
                    this->numberOfVertex; i++) {
        if (i < fromVertex) {
            int indexI = i;
            this->vertexList[indexI].setCovarianceQuaternion(
                    sqrt(covarianzeSquared[i * this->degreeOfFreedom + 2]));
            Eigen::Vector3d covariancePos(sqrt(covarianzeSquared[i * this->degreeOfFreedom + 0]),
                                          sqrt(covarianzeSquared[i * this->degreeOfFreedom + 1]), 0);
            this->vertexList[indexI].setCovariancePosition(covariancePos);
        } else {
            if (i > fromVertex) {
                int indexI = i - 1;
                this->vertexList[i].setCovarianceQuaternion(
                        sqrt(covarianzeSquared[indexI * this->degreeOfFreedom + 2]));
                Eigen::Vector3d covariancePos(sqrt(covarianzeSquared[indexI * this->degreeOfFreedom + 0]),
                                              sqrt(covarianzeSquared[indexI * this->degreeOfFreedom + 1]),
                                              0);
                this->vertexList[i].setCovariancePosition(covariancePos);
            } else {
                // i == from vertex
                this->vertexList[fromVertex].setCovarianceQuaternion(0);
                Eigen::Vector3d covariancePos(0, 0, 0);
                this->vertexList[fromVertex].setCovariancePosition(covariancePos);
            }

        }
    }
    Eigen::Vector3d covarianceOfPosEdge = this->vertexList[toVertex].getCovariancePosition();
    double covarianceOfRotEdge = this->vertexList[toVertex].getCovarianceQuaternion();

//    std::cout << "Rot diff " << (this->vertexList[fromVertex].getRotationVertex().inverse() *
//                                 this->vertexList[toVertex].getRotationVertex()).matrix() << std::endl;
//    std::cout << "Pos diff " << this->vertexList[fromVertex].getRotationVertex().inverse() *
//                                this->vertexList[toVertex].getPositionVertex() -
//                                this->vertexList[fromVertex].getRotationVertex().inverse() *
//                                this->vertexList[fromVertex].getPositionVertex() << std::endl;

    edge resultingEdge(fromVertex, toVertex, this->vertexList[fromVertex].getRotationVertex().inverse() *
                                             this->vertexList[toVertex].getPositionVertex() -
                                             this->vertexList[fromVertex].getRotationVertex().inverse() *
                                             this->vertexList[fromVertex].getPositionVertex(),
                       this->vertexList[fromVertex].getRotationVertex().inverse() *
                       this->vertexList[toVertex].getRotationVertex(), covarianceOfPosEdge, covarianceOfRotEdge,
                       degreeOfFreedom, graphSlamSaveStructure::INTEGRATED_POS_USAGE);
    this->optimizeGraphWithSlam(false, holdStill);
    return resultingEdge;

}

void graphSlamSaveStructure::removeRowColumn(Eigen::SparseMatrix<double> &matrix,
                                             int rowToRemove) const {// it is assumed that the row means the state(dim 3 or 6)
    //std::cout << "currentMatrix: \n" << matrix.toDense() << std::endl;
    const int newSizeMatrix = (int) matrix.rows() - this->degreeOfFreedom;
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm(matrix.rows());
    perm.setIdentity();
    //std::cout << "Permutation before: \n" << perm.toDenseMatrix() << std::endl;
    for (int k = 0; k < this->degreeOfFreedom; k++) {
        for (int i = rowToRemove * this->degreeOfFreedom; i < matrix.rows() - 1; i++) {
            std::swap(perm.indices()[i], perm.indices()[i + 1]);
        }
    }
    perm = perm.transpose();
    //std::cout << "Permutation after: \n" << perm.toDenseMatrix() << std::endl;
    matrix = matrix.twistedBy(perm);
    matrix = matrix.block(0, 0, newSizeMatrix, newSizeMatrix);

    //std::cout << "After removing: \n" << matrix.toDense() << std::endl;

}

Eigen::MatrixXd graphSlamSaveStructure::transformStateDiffToAddVector(std::vector<vertex> &stateBeforeOptimization,
                                                                      std::vector<vertex> &stateAfterOptimization) const {

    Eigen::MatrixXd vectorToAdd(this->numberOfVertex * this->degreeOfFreedom, 1);
    for (int i = 0; i < this->lookUpTableCell.size(); i++) {
        Eigen::Vector3d posdiff =
                stateAfterOptimization[i].getPositionVertex() - stateBeforeOptimization[i].getPositionVertex();
        double angleDiff = this->getYawAngle(stateAfterOptimization[i].getRotationVertex()) -
                this->getYawAngle(stateBeforeOptimization[i].getRotationVertex());
        for (int j : this->lookUpTableCell[i]) {
            vectorToAdd(j * this->degreeOfFreedom + 0, 0) = posdiff[0];
            vectorToAdd(j * this->degreeOfFreedom + 1, 0) = posdiff[1];
            vectorToAdd(j * this->degreeOfFreedom + 2, 0) = angleDiff;
        }
    }
    return vectorToAdd;
}

graphSlamSaveStructure graphSlamSaveStructure::getSubGraph() {
    return *this->hierachicalGraph;
}

int graphSlamSaveStructure::getCellOfVertexIndex(int vertexIndex) {
    for (int i = 0; i < this->lookUpTableCell.size(); i++) {
        for (int j = 0; j < this->lookUpTableCell[i].size(); j++) {
            if (this->lookUpTableCell[i][j] == vertexIndex) {
                return i;// return the index of cell,
            }
        }
    }
    std::cout << "could not found the vertex index you are looking for" << std::endl;
    exit(-1);
}

bool graphSlamSaveStructure::checkIfElementsOfVectorAreEqual(std::vector<int> &i0,
                                                             std::vector<int> &i1) {// check if an element of i0 is equal to element of i1

    for (int &element : i1) {
        if (std::find(i0.begin(), i0.end(),
                      element) != i0.end()) {
            return true;
        }
    }
    return false;
}

std::vector<int> graphSlamSaveStructure::joinTwoLists(std::vector<int> &i0,
                                                      std::vector<int> &i1) {//create list containing elements of i0 and i1 joined = [i0,i1]
    std::vector<int> iJoined;
    iJoined.reserve(i0.size() + i1.size()); // preallocate memory
    iJoined.insert(iJoined.end(), i0.begin(), i0.end());
    iJoined.insert(iJoined.end(), i1.begin(), i1.end());
    return iJoined;
}

bool graphSlamSaveStructure::createSubGraphBetweenCell(int vertexIndexFrom, int vertexIndexTo,
                                                       graphSlamSaveStructure &currentSubGraph) {
    std::vector<int> vertexIndicesOfICell = this->lookUpTableCell[vertexIndexFrom];
    std::vector<int> vertexIndicesOfJCell = this->lookUpTableCell[vertexIndexTo];
    std::vector<edge> listOfContainingEdges;
    std::vector<int> listOfConnectedVertexes;
    getListofConnectedVertexAndEdges(vertexIndicesOfICell, listOfContainingEdges,
                                     listOfConnectedVertexes);//go threw cell I and save edges connected to it
    bool containingConnection = checkIfElementsOfVectorAreEqual(vertexIndicesOfJCell,
                                                                listOfConnectedVertexes);//check if overlap exists
    if (containingConnection) {
        //create graph from cell i to cell j
        //graphSlamSaveStructure currentSubGraph(this->hierachicalGraph->degreeOfFreedom);
        for (int k = 0; k <
                        this->lookUpTableCell[vertexIndexFrom].size(); k++) {//add vertices of cell i@TODO can be fused with other for loop
            vertex tmpVertex = this->vertexList[this->lookUpTableCell[vertexIndexFrom][k]];
            currentSubGraph.addVertex(k, tmpVertex.getPositionVertex(), tmpVertex.getRotationVertex(),
                                      tmpVertex.getCovariancePosition(), tmpVertex.getCovarianceQuaternion(), 0,
                                      graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        }

        for (int k = 0; k < this->lookUpTableCell[vertexIndexTo].size(); k++) {//add vertices of cell j
            vertex tmpVertex = this->vertexList[this->lookUpTableCell[vertexIndexTo][k]];
            currentSubGraph.addVertex(k + (int) this->lookUpTableCell[vertexIndexFrom].size(),
                                      tmpVertex.getPositionVertex(),
                                      tmpVertex.getRotationVertex(),
                                      tmpVertex.getCovariancePosition(), tmpVertex.getCovarianceQuaternion(), 0,
                                      graphSlamSaveStructure::INTEGRATED_POS_USAGE);
        }
        //get all containing elements of the joined sub graph
        std::vector<int> vertexIndexOfJoinedGraph = joinTwoLists(vertexIndicesOfICell, vertexIndicesOfJCell);
        // look at each edge to be contained at these vertex
        for (auto &currentEdge : this->edgeList) {

            auto itFromVertex = std::find(vertexIndexOfJoinedGraph.begin(), vertexIndexOfJoinedGraph.end(),
                                          currentEdge.getFromVertex());
            auto itToVertex = std::find(vertexIndexOfJoinedGraph.begin(), vertexIndexOfJoinedGraph.end(),
                                        currentEdge.getToVertex());

            if (itFromVertex != vertexIndexOfJoinedGraph.end() &&
                itToVertex != vertexIndexOfJoinedGraph.end()) {
                int indexToVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itToVertex);
                int indexFromVertex = std::distance(vertexIndexOfJoinedGraph.begin(), itFromVertex);
                currentSubGraph.addEdge(indexFromVertex, indexToVertex,
                                        currentEdge.getPositionDifference(),
                                        currentEdge.getRotationDifference(),
                                        currentEdge.getCovariancePosition(),
                                        currentEdge.getCovarianceQuaternion(),
                                        graphSlamSaveStructure::INTEGRATED_POS_USAGE);

            }
        }
        return true;
    } else {
        return false;
    }
}


void
graphSlamSaveStructure::calculateCovarianceInCloseProximity() {//@TODO calculate the covariance CORRECT only for the current cell.(last) and connected Cells
    // get vertex numbers that are of interest:
    int i = 0;
    graphSlamSaveStructure *currentHierachicalGraph = this->hierachicalGraph;
    std::vector<std::vector<std::vector<int>>> lookUpTableCellOfSubGraphs;//[which depth]
    lookUpTableCellOfSubGraphs.push_back(this->lookUpTableCell);
    while (currentHierachicalGraph->hasHierachicalGraph) {
        i++;
        lookUpTableCellOfSubGraphs.push_back(currentHierachicalGraph->lookUpTableCell);
        currentHierachicalGraph = currentHierachicalGraph->hierachicalGraph;
    }
    std::vector<int> listOfContainingVertex;
    std::vector<int> currentListOfCellsOfImportanceLower;
    std::vector<int> currentListOfCellsOfImportanceHigher;
    //get last hierachical graph
    currentHierachicalGraph = this->hierachicalGraph;
    for (int k = 0; k < i; k++) {
        currentHierachicalGraph = currentHierachicalGraph->hierachicalGraph;
    }
    int lastCell = (int) lookUpTableCellOfSubGraphs.back().size() - 1;
    for (auto cellElement : lookUpTableCellOfSubGraphs.back()[lastCell]) {
        currentListOfCellsOfImportanceLower.push_back(cellElement);
    }

    for (int m = 0; m < lookUpTableCellOfSubGraphs.back().size() - 1; m++) {
        if (currentHierachicalGraph->checkIfDirectConnectionExists(m, lastCell)) {
            for (auto containingVertexIndex : lookUpTableCellOfSubGraphs.back()[m]) {
                currentListOfCellsOfImportanceLower.push_back(containingVertexIndex);
            }
        }
    }


    currentHierachicalGraph = this->hierachicalGraph;
    for (int j = i; j > 0; j--) {
        for (int k = 0; k < (j - 1); k++) {//get current hierachical graph
            currentHierachicalGraph = currentHierachicalGraph->hierachicalGraph;
        }
        for (auto cell:currentListOfCellsOfImportanceLower) {
            for (auto cellIndex:lookUpTableCellOfSubGraphs[j - 1][cell]) {
                currentListOfCellsOfImportanceHigher.push_back(cellIndex);
            }
        }
        currentListOfCellsOfImportanceLower = currentListOfCellsOfImportanceHigher;
    }
    listOfContainingVertex = currentListOfCellsOfImportanceLower;
    //sort for highest value stand last
    std::sort(listOfContainingVertex.begin(), listOfContainingVertex.end());
    //create subgraph for cells of listOfContainingVertex
    graphSlamSaveStructure currentSubGraph(this->degreeOfFreedom);

    for (int j = 0; j < listOfContainingVertex.size(); j++) {
        // sort in into vertex list
        vertex addedVertex = this->vertexList[listOfContainingVertex[j]];
        currentSubGraph.addVertex(j, addedVertex.getPositionVertex(), addedVertex.getRotationVertex(),
                                  addedVertex.getCovariancePosition(), addedVertex.getCovarianceQuaternion(), 0,
                                  graphSlamSaveStructure::INTEGRATED_POS_USAGE);
    }

    for (auto &currentEdge : this->edgeList) {

        auto itFromVertex = std::find(listOfContainingVertex.begin(), listOfContainingVertex.end(),
                                      currentEdge.getFromVertex());
        auto itToVertex = std::find(listOfContainingVertex.begin(), listOfContainingVertex.end(),
                                    currentEdge.getToVertex());

        if (itFromVertex != listOfContainingVertex.end() &&
            itToVertex != listOfContainingVertex.end()) {
            int indexToVertex = std::distance(listOfContainingVertex.begin(), itToVertex);
            int indexFromVertex = std::distance(listOfContainingVertex.begin(), itFromVertex);
            currentSubGraph.addEdge(indexFromVertex, indexToVertex,
                                    currentEdge.getPositionDifference(),
                                    currentEdge.getRotationDifference(),
                                    currentEdge.getCovariancePosition(),
                                    currentEdge.getCovarianceQuaternion(),
                                    graphSlamSaveStructure::INTEGRATED_POS_USAGE);

        }
    }
    std::vector<int> holdStill{(int) (listOfContainingVertex.size() - 1)};
    currentSubGraph.optimizeGraphWithSlam(false, holdStill);
    //now put in the covariances to the main graph
    for (auto &currentVertex : currentSubGraph.getVertexList()) {
        this->vertexList[listOfContainingVertex[currentVertex.getVertexNumber()]].setCovariancePosition(
                currentVertex.getCovariancePosition());
        this->vertexList[listOfContainingVertex[currentVertex.getVertexNumber()]].setCovarianceQuaternion(
                currentVertex.getCovarianceQuaternion());
    }

}


bool graphSlamSaveStructure::checkIfDirectConnectionExists(int vertexIndex0, int vertexIndex1) {
    for (auto &currentEdge : this->edgeList) {
        if (currentEdge.getFromVertex() == vertexIndex0 && currentEdge.getToVertex() == vertexIndex1 ||
            currentEdge.getToVertex() == vertexIndex0 && currentEdge.getFromVertex() == vertexIndex1) {
            return true;
        }
    }
    return false;
}

double graphSlamSaveStructure::getYawAngle(Eigen::Quaterniond quaternionYaw) {
    return  tf2::getYaw(tf2::Quaternion(quaternionYaw.x(), quaternionYaw.y(), quaternionYaw.z(), quaternionYaw.w()));
}

void graphSlamSaveStructure::saveGraphJson(std::string nameSavingFile){

    Json::Value keyFrames;

    int currentKeyFrameNumber = 0;
    for(int i = 0 ; i<this->numberOfVertex;i++){
        if(this->vertexList[i].getTypeOfVertex()==graphSlamSaveStructure::POINT_CLOUD_USAGE){
            // if point cloud is used then save this keyframe
            Json::Value positionOfKeyframe;
            positionOfKeyframe["x"] = this->vertexList[i].getPositionVertex()[0];
            positionOfKeyframe["y"] = this->vertexList[i].getPositionVertex()[1];
            positionOfKeyframe["z"] = this->vertexList[i].getPositionVertex()[2];
            positionOfKeyframe["roll"] = 0;
            positionOfKeyframe["pitch"] = 0;
            positionOfKeyframe["yaw"] = this->getYawAngle(this->vertexList[i].getRotationVertex());
            Json::Value pointCloud;

            for(int j = 0 ; j < this->vertexList[i].getPointCloudCorrected()->points.size() ; j++){
                Json::Value tmpPoint;
                tmpPoint["x"] = this->vertexList[i].getPointCloudCorrected()->points[j].x;
                tmpPoint["y"] = this->vertexList[i].getPointCloudCorrected()->points[j].y;
                tmpPoint["z"] = this->vertexList[i].getPointCloudCorrected()->points[j].z;
                pointCloud[j]["point"] = tmpPoint;
            }
            keyFrames[currentKeyFrameNumber]["position"]=positionOfKeyframe;
            keyFrames[currentKeyFrameNumber]["pointCloud"]=pointCloud;
            currentKeyFrameNumber++;
        }
    }

    // create the main object
    Json::Value outputText;
    outputText["keyFrames"] = keyFrames;

    std::ofstream ifs;
    ifs.open("/home/tim/DataForTests/DataFromSimulationJson/"+nameSavingFile);
    ifs << outputText << '\n';
    ifs.close();
}