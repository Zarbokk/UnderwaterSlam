//
// Created by tim on 12.02.21.
//

#ifndef SIMULATION_BLUEROV_KEYFRAME_H
#define SIMULATION_BLUEROV_KEYFRAME_H
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
class keyFrame {
public:
    keyFrame()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
        pointCloud = tmp;
    }

    const pcl::PointCloud<pcl::PointXYZ>::Ptr &getPointCloud() const;

    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    const Eigen::Vector3f &getEstimatedPos() const;

    void setEstimatedPos(const Eigen::Vector3f &estimatedPos);

    const Eigen::Quaternionf &getEstimatedRotation() const;

    void setEstimatedRotation(const Eigen::Quaternionf &estimatedRotation);

    const Eigen::Vector3f &getCovarianzPos() const;

    void setCovarianzPos(const Eigen::Vector3f &covarianzPos);

    const Eigen::Vector3f &getCovarianzRpy() const;

    void setCovarianzRpy(const Eigen::Vector3f &covarianzRpy);

    const ros::Time &getTimeKeyframe() const;

    void setTimeKeyframe(const ros::Time &timeKeyframe);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    Eigen::Vector3f estimatedPos;
    Eigen::Quaternionf estimatedRotation;
    Eigen::Vector3f covarianzPos;
    Eigen::Vector3f covarianzRPY;
    ros::Time timeKeyframe;
};


#endif //SIMULATION_BLUEROV_KEYFRAME_H
