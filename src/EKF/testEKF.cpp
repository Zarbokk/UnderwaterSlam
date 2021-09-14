//
// Created by jurobotics on 13.09.21.
//
#include "ekf.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_) : testEKF(ros::Time::now()) {
        subscriberIMU = n_.subscribe("mavros/imu/data", 1000, &rosClassEKF::imuCallback, this);
        subscriberDVL = n_.subscribe("mavros/local_position/velocity_body", 1000, &rosClassEKF::DVLCallback, this);
        subscriberDepth = n_.subscribe("mavros/altitude", 1000, &rosClassEKF::depthCallback, this);
        publisherPoseEkf = n_.advertise<geometry_msgs::PoseStamped>("publisherPoseEkf", 10);
    }

private:
    ekfClass testEKF;
    ros::Subscriber subscriberIMU, subscriberDepth, subscriberDVL;
    ros::Publisher  publisherPoseEkf;

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        testEKF.predictionImu(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                              msg->header.stamp);
        Eigen::Quaterniond tmpRot;
        tmpRot.x() = msg.get()->orientation.x;
        tmpRot.y() = msg.get()->orientation.y;
        tmpRot.z() = msg.get()->orientation.z;
        tmpRot.w() = msg.get()->orientation.w;
        auto euler = tmpRot.toRotationMatrix().eulerAngles(0, 1, 2);
        testEKF.updateIMU(euler.x(), euler.y(), msg.get()->angular_velocity.x, msg.get()->angular_velocity.y,
                          msg.get()->angular_velocity.z, msg->header.stamp);
        pose currentStateEkf = testEKF.getState();
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = currentStateEkf.position.x();
        poseMsg.pose.position.y = currentStateEkf.position.y();
        poseMsg.pose.position.z = currentStateEkf.position.z();
        Eigen::Quaterniond rotDiff = Eigen::AngleAxisd(currentStateEkf.rotation.x(), Eigen::Vector3d::UnitX())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(currentStateEkf.rotation.y(), Eigen::Vector3d::UnitY())//should be added somewhen(6DOF)
                                     * Eigen::AngleAxisd(currentStateEkf.rotation.z(),Eigen::Vector3d::UnitZ());
        poseMsg.pose.orientation.x = rotDiff.x();
        poseMsg.pose.orientation.y = rotDiff.y();
        poseMsg.pose.orientation.z = rotDiff.z();
        poseMsg.pose.orientation.w = rotDiff.w();
        this->publisherPoseEkf.publish(poseMsg);
    }

    void depthCallback(const mavros_msgs::Altitude::ConstPtr &msg) {
        double depth = -msg->local;// change from ENU to NED
        testEKF.updateDepth(depth, msg->header.stamp);
    }

    void DVLCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        testEKF.updateDVL(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, msg->header.stamp);
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
