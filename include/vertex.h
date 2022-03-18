//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_VETREX_H
#define SIMULATION_BLUEROV_VETREX_H
//#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "generalStructs.h"

class vertex {

public:
    //no point cloud or intensities
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,
           double timeStamp, int typeOfVertex) {
        if (degreeOfFreedom == 3) {
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
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }
    // point cloud and no intensities
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudRaw1,
           const Eigen::Vector3d &covariancePosition,
           const double covarianceQuaternion, double timeStamp, int typeOfVertex) {
        if (degreeOfFreedom == 3) {
            vertex::vertexNumber = vertexNumber;
            vertex::positionVertex = positionVertex;
            vertex::rotationVertex = rotationVertex;
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            vertex::pointCloudRaw = tmp;
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZ>);
            vertex::pointCloudCorrected = tmp2;
            vertex::covariancePosition = covariancePosition;
            vertex::covarianceQuaternion = covarianceQuaternion;
            setPointCloudRawPTRCP(pointCloudRaw1);
            this->typeOfVertex = typeOfVertex;
            this->timeStamp = timeStamp;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }

    // only intensities
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, intensityMeasurement &intensities,
           const Eigen::Vector3d &covariancePosition,
           const double covarianceQuaternion, double timeStamp, int typeOfVertex) {
        if (degreeOfFreedom == 3) {
            vertex::vertexNumber = vertexNumber;
            vertex::positionVertex = positionVertex;
            vertex::rotationVertex = rotationVertex;
            vertex::covariancePosition = covariancePosition;
            vertex::covarianceQuaternion = covarianceQuaternion;
            this->intensities = intensities;
            this->typeOfVertex = typeOfVertex;
            this->timeStamp = timeStamp;
        } else {
            std::cout << "not yet implemented DOF 6" << std::endl;
            std::exit(-1);
        }
    }



    [[nodiscard]] int getVertexNumber() const;

    void setVertexNumber(int &vertexNumberInput);

    [[nodiscard]] Eigen::Vector3d getPositionVertex() const;

    void setPositionVertex(const Eigen::Vector3d &positionVertexInput);

    [[nodiscard]] Eigen::Quaterniond getRotationVertex() const;

    void setRotationVertex(const Eigen::Quaterniond &rotationVertexInput);

    [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudCorrected() const;

    void setPointCloudCorrected(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    [[nodiscard]] pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudRaw() const;

    void setPointCloudRawPTRCP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    [[nodiscard]] const Eigen::Vector3d getCovariancePosition() const;

    void setCovariancePosition(Eigen::Vector3d covariancePositionInput);

    [[nodiscard]] double getCovarianceQuaternion() const;

    void setCovarianceQuaternion(double covarianceQuaternionInput);

    Eigen::Matrix4d getTransformation();

    [[nodiscard]] int getTypeOfVertex() const;

    void setTypeOfVertex(int &typeOfVertexInput);

    [[nodiscard]] double getTimeStamp() const;

    void setTimeStamp(double timeStampInput);

    [[nodiscard]] intensityMeasurement getIntensities() const;

    void setIntensities(const intensityMeasurement &intensitiesInput);


private:
    int vertexNumber;
    Eigen::Vector3d positionVertex;// position w.r.t. Initiial Starting Position
    Eigen::Quaterniond rotationVertex;// rotation w.r.t. Initial Starting Rotation
    Eigen::Vector3d covariancePosition;
    double covarianceQuaternion;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudRaw;//measurement by edge from this vertex to previous vertex
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudCorrected;//measurement by edge from this vertex to previous vertex
    intensityMeasurement intensities;
    int typeOfVertex;// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    double timeStamp;
};


#endif //SIMULATION_BLUEROV_VETREX_H
