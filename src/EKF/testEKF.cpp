//
// Created by jurobotics on 13.09.21.
//
#include "ekf.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "generalHelpfulTools.h"

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_) : currentEkf(ros::Time::now()), lastUpdateEkf(ros::Time::now()) {
        lastUpdateEkf = currentEkf.copyEKF();
        subscriberIMU = n_.subscribe("mavros/imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        subscriberDVL = n_.subscribe("mavros/local_position/velocity_body_frd", 1000, &rosClassEKF::DVLCallback, this);
        subscriberDepth = n_.subscribe("mavros/altitude_frd", 1000, &rosClassEKF::depthCallback, this);
        subscriberSlamResults = n_.subscribe("slam_results", 1000, &rosClassEKF::slamCallback, this);

        publisherPoseEkf = n_.advertise<geometry_msgs::PoseStamped>("publisherPoseEkf", 10);
        publisherTwistEkf = n_.advertise<geometry_msgs::TwistStamped>("publisherTwistEkf", 10);

    }

private:
    std::deque<sensor_msgs::Imu::ConstPtr> imuDeque;
    std::deque<mavros_msgs::Altitude::ConstPtr> depthDeque;
    std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDeque;
    ekfClass currentEkf, lastUpdateEkf;
    ros::Subscriber subscriberIMU, subscriberDepth, subscriberDVL, subscriberSlamResults;
    ros::Publisher publisherPoseEkf, publisherTwistEkf;
    std::mutex updateSlamMutex;



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


    void slamCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        //stop updates for now.

        this->currentEkf = this->lastUpdateEkf.copyEKF();
        //apply the new update to old EKF(saved earlyer):
        //add Sensor data until time of slam callback is reached.
        //look if one is smaller
        while (msg->header.stamp > this->imuDeque[0]->header.stamp || msg->header.stamp > this->depthDeque[0]->header.stamp ||
               msg->header.stamp > this->dvlDeque[0]->header.stamp) {
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
        tmpRot.x() = msg->pose.orientation.x;
        tmpRot.y() = msg->pose.orientation.y;
        tmpRot.z() = msg->pose.orientation.z;
        tmpRot.w() = msg->pose.orientation.w;
        Eigen::Vector3d orientationRPY = generalHelpfulTools::getRollPitchYaw(tmpRot);
        //update slam data
        this->currentEkf.updateSlam(msg->pose.position.x, msg->pose.position.y, orientationRPY(2), msg->header.stamp);

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


};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ekfwithros");
    ros::start();
    ros::NodeHandle n_;
    rosClassEKF rosClassForTests(n_);

    ros::spin();


    return (0);
}
