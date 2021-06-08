//
// Created by tim on 23.02.21.
//

#include "vertex.h"

int vertex::getVertexNumber() const {
    return vertex::vertexNumber;
}

void vertex::setVertexNumber(int vertexNumber) {
    vertex::vertexNumber = vertexNumber;
}

const Eigen::Vector3d &vertex::getPositionVertex() const {
    return positionVertex;
}

void vertex::setPositionVertex(const Eigen::Vector3d &positionVertex) {
    vertex::positionVertex = positionVertex;
}

const Eigen::Quaterniond &vertex::getRotationVertex() const {
    return rotationVertex;
}

void vertex::setRotationVertex(const Eigen::Quaterniond &rotationVertex) {
    vertex::rotationVertex = rotationVertex;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &vertex::getPointCloudRaw() const {
    return vertex::pointCloudRaw;
}

void vertex::setPointCloudRaw(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    *vertex::pointCloudRaw = *pointCloud;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &vertex::getPointCloudCorrected() const {
    return vertex::pointCloudCorrected;
}

void vertex::setPointCloudCorrected(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) {
    *vertex::pointCloudCorrected = *pointCloud;
}

Eigen::Vector3d vertex::getCovariancePosition() const {
    return covariancePosition;
}

void vertex::setCovariancePosition(Eigen::Vector3d covariancePosition) {
    vertex::covariancePosition = covariancePosition;
}

double vertex::getCovarianceQuaternion() const {
    return covarianceQuaternion;
}

void vertex::setCovarianceQuaternion(double covarianceQuaternion) {
    vertex::covarianceQuaternion = covarianceQuaternion;
}

Eigen::Matrix4d vertex::getTransformation(){
    Eigen::Matrix4d transformation;

    transformation << 1, 0, 0, this->positionVertex.x(),
            0, 1, 0, this->positionVertex.y(),
            0, 0, 1, this->positionVertex.z(),
            0, 0, 0, 1;//transformation missing currently
    Eigen::Matrix3d m(this->rotationVertex.toRotationMatrix());
    transformation.block<3, 3>(0, 0) = m;
    return transformation;
}

int vertex::getTypeOfVertex() const {
    return typeOfVertex;
}

void vertex::setTypeOfVertex(int typeOfVertex) {// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    vertex::typeOfVertex = typeOfVertex;
}

double vertex::getTimeStamp() const {
    return timeStamp;
}

void vertex::setTimeStamp(double timeStamp) {
    vertex::timeStamp = timeStamp;
}
