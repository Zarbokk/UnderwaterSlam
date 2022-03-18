//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_EDGE_H
#define SIMULATION_BLUEROV_EDGE_H

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class edge {
public:
    edge(const int fromVertex, const int toVertex, const Eigen::Vector3d& positionDifference,
         const Eigen::Quaterniond& rotationDifference, const Eigen::Vector3d &covariancePosition,
         const double covarianceQuaternion,
         int degreeOfFreedom, int typeOfEdge) {
        if (degreeOfFreedom == 3) {
            edge::fromVertex = fromVertex;
            edge::toVertex = toVertex;
            edge::positionDifference = positionDifference;
            edge::rotationDifference = rotationDifference;

            edge::covariancePosition = covariancePosition;
            edge::covarianceQuaternion = covarianceQuaternion;

        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
        this->typeOfEdge = typeOfEdge;

    }

    void setEdge(edge &edgeToCopy);

    [[nodiscard]] Eigen::Vector3d getCovariancePosition() const;

    void setCovariancePosition(Eigen::Vector3d &covariancePositionInput);

    [[nodiscard]] double getCovarianceQuaternion() const;

    void setCovarianceQuaternion(double &covarianceQuaternionInput);

    [[nodiscard]] Eigen::Vector3d getPositionDifference() const;

    void setPositionDifference(const Eigen::Vector3d &positionDifferenceInput);

    [[nodiscard]] Eigen::Quaterniond getRotationDifference() const;

    void setRotationDifference(const Eigen::Quaterniond &rotationDifferenceInput);

    [[nodiscard]] int getFromVertex() const;

    void setFromVertex(int &fromVertexInput);

    [[nodiscard]] int getToVertex() const;

    void setToVertex(int &toVertexInput);

    [[nodiscard]] int getTypeOfEdge() const;

    void setTypeOfEdge(int &typeOfEdge);

    Eigen::Matrix4d getTransformation();

private:
    int fromVertex;
    int toVertex;
    Eigen::Vector3d covariancePosition;//estimated covariance for this measurement in x y z
    double covarianceQuaternion;//estimated covariance for this measurement in q = w x y z (rotation)
    Eigen::Vector3d positionDifference;
    Eigen::Quaterniond rotationDifference;

    int typeOfEdge;// 0=Matching    %%%%%%%%%   1 = integratedPosDiff
    //double timeStamp;


};


#endif //SIMULATION_BLUEROV_EDGE_H
