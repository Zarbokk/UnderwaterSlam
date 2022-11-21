//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_VETREX_H
#define SIMULATION_BLUEROV_VETREX_H
//#include <vector>
#include "eigen3/Eigen/Geometry"
//#include <Eigen3/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include "generalStructs.h"

class vertex {

public:
    //no point cloud or intensities
    vertex(int vertexNumber, const Eigen::Vector3d &positionVertex, const Eigen::Quaterniond &rotationVertex,
           int degreeOfFreedom, const Eigen::Vector3d &covariancePosition, const double covarianceQuaternion,
           double timeStamp, int typeOfVertex) {
        if (degreeOfFreedom == 3) {
            vertex::keyNumber = vertexNumber;
            vertex::positionVertex = positionVertex;
            vertex::rotationVertex = rotationVertex;
            vertex::rotationVertex.normalize();
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
            vertex::keyNumber = vertexNumber;
            vertex::positionVertex = positionVertex;
            vertex::rotationVertex = rotationVertex;
            vertex::rotationVertex.normalize();
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
            vertex::keyNumber = vertexNumber;
            vertex::positionVertex = positionVertex;
            vertex::rotationVertex = rotationVertex;
            vertex::rotationVertex.normalize();
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



    [[nodiscard]] int getKey() const;

    void setKey(int &vertexNumberInput);

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

    void setTypeOfVertex(int typeOfVertexInput);

    [[nodiscard]] double getTimeStamp() const;

    void setTimeStamp(double timeStampInput);

    [[nodiscard]] intensityMeasurement getIntensities() const;

    void setIntensities(const intensityMeasurement &intensitiesInput);


private:
    int keyNumber;
    Eigen::Vector3d positionVertex;// position w.r.t. Initiial Starting Position
    Eigen::Quaterniond rotationVertex;// rotation w.r.t. Initial Starting Rotation
    Eigen::Vector3d covariancePosition;
    double covarianceQuaternion;
    intensityMeasurement intensities;
    int typeOfVertex;
    double timeStamp;
};


#endif //SIMULATION_BLUEROV_VETREX_H
