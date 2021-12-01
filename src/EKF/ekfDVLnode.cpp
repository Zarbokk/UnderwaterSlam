//
// Created by jurobotics on 13.09.21.
//
#include "ekfDVL.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "generalHelpfulTools.h"
#include "waterlinked_dvl/TransducerReportStamped.h"

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_) : currentEkf(ros::Time::now()) {
        this->rotationOfDVL = Eigen::AngleAxisd(3.14159 / 4.0, Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data;

        subscriberIMU = n_.subscribe("mavros/imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        //subscriberDVL = n_.subscribe("transducer_report", 1000, &rosClassEKF::DVLCallbackDVL, this);
        subscriberDVL = n_.subscribe("mavros/local_position/velocity_body_frd", 1000, &rosClassEKF::DVLCallbackMavros, this);

        publisherPoseEkf = n_.advertise<geometry_msgs::PoseStamped>("publisherPoseEkf", 10);
        publisherTwistEkf = n_.advertise<geometry_msgs::TwistStamped>("publisherTwistEkf", 10);
    }

private:
//    std::deque<sensor_msgs::Imu::ConstPtr> imuDeque;
//    std::deque<mavros_msgs::Altitude::ConstPtr> depthDeque;
//    std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDeque;
    ekfClassDVL currentEkf;
    ros::Subscriber subscriberIMU, subscriberDepth, subscriberDVL, subscriberSlamResults;
    ros::Publisher publisherPoseEkf, publisherTwistEkf;
    std::mutex updateSlamMutex;
    Eigen::Quaterniond rotationOfDVL;


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
//        std::cout << msg->header.stamp << std::endl;
        poseMsg.header.stamp = msg->header.stamp;
        this->publisherPoseEkf.publish(poseMsg);
        geometry_msgs::TwistStamped twistMsg;
        twistMsg.twist.linear.x = currentStateEkf.velocity.x();
        twistMsg.twist.linear.y = currentStateEkf.velocity.y();
        twistMsg.twist.linear.z = currentStateEkf.velocity.z();
        twistMsg.twist.angular.x = currentStateEkf.angleVelocity.x();
        twistMsg.twist.angular.y = currentStateEkf.angleVelocity.y();
        twistMsg.twist.angular.z = currentStateEkf.angleVelocity.z();
        twistMsg.header.stamp = msg->header.stamp;
        this->publisherTwistEkf.publish(twistMsg);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->imuCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackDVLHelper(const waterlinked_dvl::TransducerReportStamped::ConstPtr &msg) {
        currentEkf.updateDVL(msg->report.vx, msg->report.vy, msg->report.vz, this->rotationOfDVL, msg->header.stamp);
    }

    void DVLCallbackDVL(const waterlinked_dvl::TransducerReportStamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackDVLHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackMavrosHelper(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        currentEkf.updateDVL(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, this->rotationOfDVL, msg->header.stamp);
    }

    void DVLCallbackMavros(const geometry_msgs::TwistStamped::ConstPtr  &msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackMavrosHelper(msg);
        this->updateSlamMutex.unlock();
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekffordvlwithros");
    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_);

    ros::spin();


    return (0);
}
