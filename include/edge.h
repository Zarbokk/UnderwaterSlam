//
// Created by tim on 23.02.21.
//

#ifndef SIMULATION_BLUEROV_EDGE_H
#define SIMULATION_BLUEROV_EDGE_H

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class edge {
public:
    edge(const int fromVertex, const int toVertex, const Eigen::Vector3d &positionDifference,
         const Eigen::Quaterniond &rotationDifference, const Eigen::Vector3d covariancePosition,
         const double covarianceQuaternion,
         int degreeOfFreedom, int typeOfEdge) {
//        if(abs(rotationDifference.z())>0.2){
//            std::cout << rotationDifference.z()<<"  " << rotationDifference.w()<< std::endl;
//        }
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        edge::pointCloud = tmp;
    }

    edge(const int fromVertex, const int toVertex, const Eigen::Vector3d &positionDifference,
         const Eigen::Quaterniond &rotationDifference, const Eigen::Vector3d covariancePosition,
         const double covarianceQuaternion,
         int degreeOfFreedom, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, const int typeOfEdge) {
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        edge::pointCloud = tmp;
        setPointCloud(pointCloud);
        this->typeOfEdge = typeOfEdge;
    }

    void setEdge(edge &edgeToCopy);

    Eigen::Vector3d getCovariancePosition() const;

    void setCovariancePosition(Eigen::Vector3d covariancePosition);

    double getCovarianceQuaternion() const;

    void setCovarianceQuaternion(double covarianceQuaternion);

    const Eigen::Vector3d &getPositionDifference() const;

    void setPositionDifference(const Eigen::Vector3d &positionDifference);

    const Eigen::Quaterniond &getRotationDifference() const;

    void setRotationDifference(const Eigen::Quaterniond &rotationDifference);

    int getFromVertex() const;

    void setFromVertex(int fromVertex);

    int getToVertex() const;

    void setToVertex(int toVertex);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &getPointCloud() const;

    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);


private:
    int fromVertex;
    int toVertex;
    Eigen::Vector3d covariancePosition;//estimated covarianze for this measurement in x y z
    double covarianceQuaternion;//estimated covarianze for this measurement in q = w x y z (rotation)
    bool hasPointCloud;
public:
    bool getHasPointCloud() const;

    void setHasPointCloud(bool hasPointCloud);

private:
    Eigen::Vector3d positionDifference;
    Eigen::Quaterniond rotationDifference;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;//measurement
    int typeOfEdge;// 0=pointCloud    %%%%%%%%%   1 = integratedPosDiff
    double timeStamp;
public:
    bool isHasPointCloud() const;

    int getTypeOfEdge() const;

    void setTypeOfEdge(int typeOfEdge);

    double getTimeStamp() const;

    void setTimeStamp(double timeStamp);

    Eigen::Matrix4d getTransformation();
};


#endif //SIMULATION_BLUEROV_EDGE_H
