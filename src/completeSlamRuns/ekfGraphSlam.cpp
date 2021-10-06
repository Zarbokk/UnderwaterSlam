//
// Created by jurobotics on 13.09.21.
//
#include "ekf.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "generalHelpfulTools.h"
#include <slamToolsRos.h>

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_) : currentEkf(ros::Time::now()), lastUpdateEkf(ros::Time::now()), graphSaved(3) {
        lastUpdateEkf = currentEkf.copyEKF();
        subscriberIMU = n_.subscribe("mavros/imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        subscriberDVL = n_.subscribe("mavros/local_position/velocity_body_frd", 1000, &rosClassEKF::DVLCallback, this);
        subscriberDepth = n_.subscribe("mavros/altitude_frd", 1000, &rosClassEKF::depthCallback, this);
        subscriberFullScan = n_.subscribe("sonar/full_scan", 10000, &rosClassEKF::scanCallback, this);

        publisherPoseEkf = n_.advertise<geometry_msgs::PoseStamped>("publisherPoseEkf", 10);
        publisherTwistEkf = n_.advertise<geometry_msgs::TwistStamped>("publisherTwistEkf", 10);
        publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
        publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        publisherLastPCL = n_.advertise<sensor_msgs::PointCloud2>("lastPCL", 10);
        publisherRegistrationPCL = n_.advertise<sensor_msgs::PointCloud2>("registratedPCL", 10);
        publisherBeforeCorrection = n_.advertise<sensor_msgs::PointCloud2>("beforeCorrection", 10);
        publisherAfterCorrection = n_.advertise<sensor_msgs::PointCloud2>("afterCorrection", 10);


        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
        transformationX180Degree(3, 3) = 1;

        graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                             Eigen::Vector3d(0, 0, 0), 0, 0.0, graphSlamSaveStructure::FIRST_ENTRY);

        std::deque<double> subgraphs{0.1, 1, 3};
        graphSaved.initiallizeSubGraphs(subgraphs, maxTimeOptimization);
        this->sigmaScaling = 1.0;
        this->timeLastFullScan = 0;
        this->maxTimeOptimization = 1.0;
        this->numberOfEdgesBetweenScans = 20;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL3(new pcl::PointCloud<pcl::PointXYZ>);
        this->currentScan = tmpPCL1;
        this->lastScan = tmpPCL2;
        this->Final = tmpPCL3;

    }

private:
    //EKF things
    std::deque<sensor_msgs::Imu::ConstPtr> imuDeque;
    std::deque<mavros_msgs::Altitude::ConstPtr> depthDeque;
    std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDeque;
    ekfClass currentEkf, lastUpdateEkf;
    ros::Subscriber subscriberIMU, subscriberDepth, subscriberDVL, subscriberFullScan;
    ros::Publisher publisherPoseEkf, publisherTwistEkf;
    std::mutex updateSlamMutex;


    //GraphSlam things
    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;
    //PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final;

    //Matrices:
    Eigen::Matrix4d currentTransformation;
    Eigen::Matrix4d initialGuessTransformation;
    Eigen::Matrix4d transformationX180Degree;


    std::vector<vertex> posDiffOverTimeVertices;
    std::deque<edge> posDiffOverTimeEdges;
    std::deque<ImuData> imuDataList;
    std::deque<DvlData> dvlDataList;


    double timeLastFullScan;
    double timeCurrentFullScan;
    double fitnessScore;
    double sigmaScaling;
    double maxTimeOptimization;
    int numberOfEdgesBetweenScans;
    //std::condition_variable imuFree;// dvlFree;
    std::mutex imuFree, dvlFree;
    graphSlamSaveStructure graphSaved;


    void imuCallbackHelper(const sensor_msgs::Imu::ConstPtr &msg) {
        Eigen::Quaterniond tmpRot;
        tmpRot.x() = msg.get()->orientation.x;
        tmpRot.y() = msg.get()->orientation.y;
        tmpRot.z() = msg.get()->orientation.z;
        tmpRot.w() = msg.get()->orientation.w;
        currentEkf.predictionImu(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                                 tmpRot,
                                 msg->header.stamp);

        Eigen::Vector3d euler = generalHelpfulTools::getRollPitchYaw(tmpRot);// roll pitch yaw

        currentEkf.updateIMU(euler.x(), euler.y(), msg.get()->angular_velocity.x, msg.get()->angular_velocity.y,
                             msg.get()->angular_velocity.z, tmpRot, msg->header.stamp);
        pose currentStateEkf = currentEkf.getState();
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = currentStateEkf.position.x();
        poseMsg.pose.position.y = currentStateEkf.position.y();
        poseMsg.pose.position.z = currentStateEkf.position.z();
        Eigen::Quaterniond rotDiff = Eigen::AngleAxisd(currentStateEkf.rotation.x(),
                                                       Eigen::Vector3d::UnitX())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(currentStateEkf.rotation.y(),
                                                         Eigen::Vector3d::UnitY())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(currentStateEkf.rotation.z(), Eigen::Vector3d::UnitZ());
        poseMsg.pose.orientation.x = rotDiff.x();
        poseMsg.pose.orientation.y = rotDiff.y();
        poseMsg.pose.orientation.z = rotDiff.z();
        poseMsg.pose.orientation.w = rotDiff.w();
        this->publisherPoseEkf.publish(poseMsg);
        geometry_msgs::TwistStamped twistMsg;
        twistMsg.twist.linear.x = currentStateEkf.velocity.x();
        twistMsg.twist.linear.y = currentStateEkf.velocity.y();
        twistMsg.twist.linear.z = currentStateEkf.velocity.z();
        twistMsg.twist.angular.x = currentStateEkf.angleVelocity.x();
        twistMsg.twist.angular.y = currentStateEkf.angleVelocity.y();
        twistMsg.twist.angular.z = currentStateEkf.angleVelocity.z();

        this->publisherTwistEkf.publish(twistMsg);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->imuDeque.push_back(msg);
        this->imuCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void depthCallbackHelper(const mavros_msgs::Altitude::ConstPtr &msg) {
        double depth = msg->local;
        currentEkf.updateDepth(depth, msg->header.stamp);
    }

    void depthCallback(const mavros_msgs::Altitude::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->depthDeque.push_back(msg);
        this->depthCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackHelper(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        currentEkf.updateDVL(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, msg->header.stamp);
    }

    void DVLCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->dvlDeque.push_back(msg);
        this->DVLCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void slamCallback(const geometry_msgs::PoseStamped &msg) {
        this->updateSlamMutex.lock();
        //stop updates for now.

        this->currentEkf = this->lastUpdateEkf.copyEKF();
        this->currentEkf.removePastPoses();
        //apply the new update to old EKF(saved earlyer):
        //add Sensor data until time of slam callback is reached.
        //look if one is smaller
        while (msg.header.stamp > this->imuDeque[0]->header.stamp ||
               msg.header.stamp > this->depthDeque[0]->header.stamp ||
               msg.header.stamp > this->dvlDeque[0]->header.stamp) {
            //find which is smallest
            if (this->imuDeque[0]->header.stamp < this->depthDeque[0]->header.stamp &&
                this->imuDeque[0]->header.stamp < this->dvlDeque[0]->header.stamp) {
                //smallest: imuDeque[0]->header.stamp
                this->imuCallbackHelper(this->imuDeque[0]);
                this->imuDeque.pop_front();
            } else if (this->depthDeque[0]->header.stamp < this->imuDeque[0]->header.stamp &&
                       this->depthDeque[0]->header.stamp < this->dvlDeque[0]->header.stamp) {
                //smallest: depthDeque[0]->header.stamp
                this->depthCallbackHelper(this->depthDeque[0]);
                this->depthDeque.pop_front();
            } else {
                //smallest: dvlDeque[0]->header.stamp
                this->DVLCallbackHelper(this->dvlDeque[0]);
                this->dvlDeque.pop_front();
            }
        }
        Eigen::Quaterniond tmpRot;
        tmpRot.x() = msg.pose.orientation.x;
        tmpRot.y() = msg.pose.orientation.y;
        tmpRot.z() = msg.pose.orientation.z;
        tmpRot.w() = msg.pose.orientation.w;
        Eigen::Vector3d orientationRPY = generalHelpfulTools::getRollPitchYaw(tmpRot);
        //update slam data
        this->currentEkf.updateSlam(msg.pose.position.x, msg.pose.position.y, orientationRPY(2), msg.header.stamp);

        //copy the ekf(for later and next slam update)
        this->lastUpdateEkf = this->currentEkf.copyEKF();
        // apply the remaining updates and predictions.


        std::deque<sensor_msgs::Imu::ConstPtr> imuDequeTMP = this->imuDeque;
        std::deque<mavros_msgs::Altitude::ConstPtr> depthDequeTMP = this->depthDeque;
        std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDequeTMP = this->dvlDeque;

        while (!imuDequeTMP.empty() || !depthDequeTMP.empty() || !dvlDequeTMP.empty()) {
            double a, b, c;
            //find if empty, and then add big number
            if (imuDequeTMP.empty()) {
                a = 1000000000000;
            } else {
                a = imuDequeTMP[0]->header.stamp.toSec();
            }
            if (depthDequeTMP.empty()) {
                b = 1000000000000;
            } else {
                b = depthDequeTMP[0]->header.stamp.toSec();
            }
            if (dvlDequeTMP.empty()) {
                c = 1000000000000;
            } else {
                c = dvlDequeTMP[0]->header.stamp.toSec();
            }


            //find which is smallest, then it still has sth in its deque
            if (a < b && a < c) {
                //smallest: imuDeque[0]->header.stamp
                this->imuCallbackHelper(imuDequeTMP[0]);
                imuDequeTMP.pop_front();
            } else if (b < a &&
                       b < c) {
                //smallest: depthDeque[0]->header.stamp
                this->depthCallbackHelper(depthDequeTMP[0]);
                depthDequeTMP.pop_front();
            } else {
                //smallest: dvlDeque[0]->header.stamp
                this->DVLCallbackHelper(dvlDequeTMP[0]);
                dvlDequeTMP.pop_front();
            }
        }
        this->updateSlamMutex.unlock();
    }

    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        this->timeCurrentFullScan = msg->header.stamp.toSec();
        pcl::fromROSMsg(*msg, *this->currentScan);

        pcl::transformPointCloud(*this->currentScan, *this->currentScan, transformationX180Degree);

        if (this->timeLastFullScan == 0) {
            this->timeLastFullScan = this->timeCurrentFullScan;
            this->graphSaved.getVertexByIndex(0)->setTimeStamp(this->timeCurrentFullScan);
            *this->lastScan = *this->currentScan;
            return;
        }




        //calculate vertex and edges
        //get imu/dvl data immediatly
//        std::deque<ImuData> lastImuData(3);
//        this->imuFree.lock();
//        lastImuData = this->imuDataList;
//        this->imuFree.unlock();
//        std::deque<DvlData> lastDvlData(3);
        //std::unique_lock<std::mutex> lck2(this->mutex_);
        //dvlFree.wait(lck2);
//        this->dvlFree.lock();
//        lastDvlData = this->dvlDataList;
//        this->dvlFree.unlock();
        this->posDiffOverTimeEdges.clear();
//        slamToolsRos::calculatePositionOverTime(lastImuData,
//                                                lastDvlData,
//                                                this->posDiffOverTimeEdges, this->timeLastFullScan,
//                                                this->timeCurrentFullScan, 0.1,this->numberOfEdgesBetweenScans);
        this->posDiffOverTimeEdges = this->currentEkf.getLastPoses();
        slamToolsRos::appendEdgesToGraph(this->graphSaved, this->posDiffOverTimeEdges, 0.3, 0.25, maxTimeOptimization);
        this->graphSaved.getVertexList().back().setPointCloudRaw(this->currentScan);
        //correct the scan depending on the Imu and Velocity callback
        slamToolsRos::correctPointCloudAtPos(this->graphSaved.getVertexList().back().getVertexNumber(),
                                             this->graphSaved, 0, 2 * M_PI, true,
                                             Eigen::Matrix4d::Identity());


        int positionLastPcl = 1;
        int sizeOfVertexList = this->graphSaved.getVertexList().size();
        while (true) {
            if (this->graphSaved.getVertexList()[sizeOfVertexList - positionLastPcl - 1].getTypeOfVertex() ==
                graphSlamSaveStructure::POINT_CLOUD_USAGE ||
                this->graphSaved.getVertexList()[sizeOfVertexList - positionLastPcl - 1].getTypeOfVertex() ==
                graphSlamSaveStructure::FIRST_ENTRY) {
                break;
            }
            positionLastPcl++;
        }
        positionLastPcl++;
        //make scan matching with last scan
        this->initialGuessTransformation =
                this->graphSaved.getVertexList()[this->graphSaved.getVertexList().size() -
                                                 positionLastPcl].getTransformation().inverse() *
                this->graphSaved.getVertexList().back().getTransformation();
        this->currentTransformation = scanRegistrationClass::generalizedIcpRegistration(
                this->graphSaved.getVertexList().back().getPointCloudCorrected(), this->lastScan, this->Final,
                this->fitnessScore,
                this->initialGuessTransformation);
        std::cout << "current Fitness Score: " << sqrt(this->fitnessScore) << std::endl;
//        pcl::io::savePCDFileASCII("/home/jurobotics/DataForTests/savingRandomPCL/firstPCL.pcd",*this->graphSaved.getVertexList().back().getPointCloudCorrected());
//        pcl::io::savePCDFileASCII("/home/jurobotics/DataForTests/savingRandomPCL/secondPCL.pcd",*this->lastScan);


        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloudPlotOnly(
                new pcl::PointCloud<pcl::PointXYZ>);
        *tmpCloudPlotOnly = *currentScan;
        //pcl::transformPointCloud(*tmpCloudPlotOnly, *tmpCloudPlotOnly, transformationImu2PCL);
        slamToolsRos::debugPlotting(this->lastScan, this->Final, tmpCloudPlotOnly,
                                    this->graphSaved.getVertexList().back().getPointCloudCorrected(),
                                    this->publisherLastPCL, this->publisherRegistrationPCL,
                                    this->publisherBeforeCorrection, this->publisherAfterCorrection);


        Eigen::Quaterniond qTMP(this->currentTransformation.block<3, 3>(0, 0));
        graphSaved.addEdge(this->graphSaved.getVertexList().size() - positionLastPcl,
                           graphSaved.getVertexList().size() - 1,
                           this->currentTransformation.block<3, 1>(0, 3), qTMP,
                           Eigen::Vector3d(sqrt(this->fitnessScore), sqrt(this->fitnessScore), 0),
                           0.25 * sqrt(this->fitnessScore),
                           graphSlamSaveStructure::POINT_CLOUD_USAGE,
                           maxTimeOptimization);//@TODO still not sure about size

        slamToolsRos::detectLoopClosure(this->graphSaved, this->sigmaScaling, 1.5, maxTimeOptimization);//was 1.0

        //add position and optimize/publish everything
//        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
//                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
//                                            groundTruthSorted, publisherMarkerArrayLoopClosures,
//                                            timeCurrentGroundTruth);

//        slamToolsRos::visualizeCurrentGraph(graphSaved, publisherPathOverTime, publisherKeyFrameClouds,
//                                            publisherMarkerArray, sigmaScaling, publisherPathOverTimeGT,
//                                            groundTruthSorted, publisherMarkerArrayLoopClosures,
//                                            timeCurrentGroundTruth);

        graphSaved.optimizeGraphWithSlamTopDown(false, 0.05, maxTimeOptimization);
//        std::vector<int> holdStill{0};
//        graphSaved.optimizeGraphWithSlam(false, holdStill,maxTimeOptimization);

        graphSaved.calculateCovarianceInCloseProximity(maxTimeOptimization);

        slamToolsRos::visualizeCurrentGraph(this->graphSaved, this->publisherPathOverTime,
                                            this->publisherKeyFrameClouds,
                                            this->publisherMarkerArray, this->sigmaScaling,
                                            this->publisherPathOverTimeGT,
                                            NULL, this->publisherMarkerArrayLoopClosures,
                                            NULL, this->numberOfEdgesBetweenScans);
        std::cout << "next: " << std::endl;
        this->timeLastFullScan = this->timeCurrentFullScan;
        *this->lastScan = *this->graphSaved.getVertexList().back().getPointCloudCorrected();

        geometry_msgs::PoseStamped newMsg;
        newMsg.header.stamp = msg->header.stamp;
        newMsg.pose.position.x = this->graphSaved.getVertexList().back().getPositionVertex().x();
        newMsg.pose.position.y = this->graphSaved.getVertexList().back().getPositionVertex().y();
        newMsg.pose.position.z = 0;

        newMsg.pose.orientation.x = this->graphSaved.getVertexList().back().getRotationVertex().x();
        newMsg.pose.orientation.y = this->graphSaved.getVertexList().back().getRotationVertex().y();
        newMsg.pose.orientation.z = this->graphSaved.getVertexList().back().getRotationVertex().z();
        newMsg.pose.orientation.w = this->graphSaved.getVertexList().back().getRotationVertex().w();

        this->slamCallback(newMsg);


    }


};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekfwithros");
    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_);

    ros::spin();


    return (0);
}
