//
// Created by tim on 23.02.21.
//

#include "vertex.h"

int vertex::getKey() const {
    return vertex::keyNumber;
}

void vertex::setKey(int &vertexNumberInput) {
    vertex::keyNumber = vertexNumberInput;
}

Eigen::Vector3d vertex::getPositionVertex() const {
    return positionVertex;
}

void vertex::setPositionVertex(const Eigen::Vector3d &positionVertexInput) {
    vertex::positionVertex = positionVertexInput;
}

Eigen::Quaterniond vertex::getRotationVertex() const {
    return rotationVertex;
}

void vertex::setRotationVertex(const Eigen::Quaterniond &rotationVertexInput) {
    this->rotationVertex = rotationVertexInput;
    this->rotationVertex.normalize();
}

const Eigen::Vector3d vertex::getCovariancePosition() const {
    return covariancePosition;
}

void vertex::setCovariancePosition(Eigen::Vector3d covariancePositionInput) {
    vertex::covariancePosition = covariancePositionInput;
}

double vertex::getCovarianceQuaternion() const {
    return covarianceQuaternion;
}

void vertex::setCovarianceQuaternion(double covarianceQuaternionInput) {
    vertex::covarianceQuaternion = covarianceQuaternionInput;
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

void vertex::setTypeOfVertex(int typeOfVertexInput) {// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    vertex::typeOfVertex = typeOfVertexInput;
}

double vertex::getTimeStamp() const {
    return timeStamp;
}

void vertex::setTimeStamp(double timeStampInput) {
    vertex::timeStamp = timeStampInput;
}

intensityMeasurement vertex::getIntensities() const {
    return intensities;
}

void vertex::setIntensities(const intensityMeasurement &intensitiesInput) {
    vertex::intensities = intensitiesInput;
}
