//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_VETREX_H
#define SIMULATION_BLUEROV_VETREX_H
//#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class vertex {

public:
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,
           double timeStamp, int typeOfVertex) {
        vertex::vertexNumber = vertexNumber;
        vertex::positionVertex = positionVertex;
        vertex::rotationVertex = rotationVertex;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        vertex::pointCloudRaw = tmp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZ>);
        vertex::pointCloudCorrected = tmp2;
        vertex::covariancePosition = covariancePosition;
        vertex::covarianceQuaternion = covarianceQuaternion;
        this->typeOfVertex = typeOfVertex;
        this->timeStamp = timeStamp;
    }

    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloudRaw1,
           const Eigen::Vector3d &covariancePosition,
           const double covarianceQuaternion, double timeStamp, int typeOfVertex) {
        vertex::vertexNumber = vertexNumber;
        vertex::positionVertex = positionVertex;
        vertex::rotationVertex = rotationVertex;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        vertex::pointCloudRaw = tmp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZ>);
        vertex::pointCloudCorrected = tmp2;
        vertex::covariancePosition = covariancePosition;
        vertex::covarianceQuaternion = covarianceQuaternion;
        setPointCloudRaw(pointCloudRaw1);
        this->typeOfVertex = typeOfVertex;
        this->timeStamp = timeStamp;
    }

    int getVertexNumber() const;

    void setVertexNumber(int vertexNumber);

    const Eigen::Vector3d &getPositionVertex() const;

    void setPositionVertex(const Eigen::Vector3d &positionVertex);

    const Eigen::Quaterniond &getRotationVertex() const;

    void setRotationVertex(const Eigen::Quaterniond &rotationVertex);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &getPointCloudCorrected() const;

    void setPointCloudCorrected(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &getPointCloudRaw() const;

    void setPointCloudRaw(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    Eigen::Vector3d getCovariancePosition() const;

    void setCovariancePosition(Eigen::Vector3d covariancePosition);

    double getCovarianceQuaternion() const;

    void setCovarianceQuaternion(double covarianceQuaternion);

    Eigen::Matrix4d getTransformation();

    int getTypeOfVertex() const;

    void setTypeOfVertex(int typeOfVertex);

    double getTimeStamp() const;

    void setTimeStamp(double timeStamp);

private:
    int vertexNumber;
    Eigen::Vector3d positionVertex;// position w.r.t. Initiial Starting Position
    Eigen::Quaterniond rotationVertex;// rotation w.r.t. Initial Starting Rotation
    Eigen::Vector3d covariancePosition;
    double covarianceQuaternion;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudRaw;//measurement by edge from this vertex to previous vertex
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCorrected;//measurement by edge from this vertex to previous vertex
    int typeOfVertex;// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff

    double timeStamp;
};


#endif //SIMULATION_BLUEROV_VETREX_H
