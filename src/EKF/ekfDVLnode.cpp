//
// Created by jurobotics on 13.09.21.
//
#include "ekfDVL.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "../slamTools/generalHelpfulTools.h"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "commonbluerovmsg/resetekf.h"
#include "commonbluerovmsg/heightStamped.h"
//#include <chrono>
#include <thread>
#include "underwaterslam/ekfnoiseConfig.h"
#include <dynamic_reconfigure/server.h>

class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_, double rotationOfDVLZ, std::string imuUsage, std::string dvlUsage) : currentEkf(
            ros::Time::now()) {
        this->rotationOfDVL = Eigen::AngleAxisd(rotationOfDVLZ,
                                                Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data;
        //std::cout << "we are here" << std::endl;
        //std::cout << imuUsage << std::endl;
        if (imuUsage == "external") {
            std::cout << "External IMU used for EKF" << std::endl;
            this->subscriberIMU = n_.subscribe("imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        } else {
            std::cout << "Mavros IMU used for EKF" << std::endl;
            this->subscriberIMU = n_.subscribe("mavros/imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        }

        if (dvlUsage == "external") {
            std::cout << "External DVL used for EKF" << std::endl;
            this->subscriberDVL = n_.subscribe("dvl/transducer_report", 1000, &rosClassEKF::DVLCallbackDVL, this);
        } else {
            std::cout << "Mavros IMU used for EKF" << std::endl;
            this->subscriberVelocityMavros = n_.subscribe("simulatedDVL", 1000, &rosClassEKF::DVLCallbackSimulation,
                                                          this);
        }
        this->subscriberDepth = n_.subscribe("height_baro", 1000, &rosClassEKF::depthSensorCallback, this);
        this->subscriberHeading = n_.subscribe("magnetic_heading", 1000, &rosClassEKF::headingCallback, this);

        this->serviceResetEkf = n_.advertiseService("resetCurrentEKF", &rosClassEKF::resetEKF, this);

        this->publisherPoseEkf = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf", 10);
        this->publisherTwistEkf = n_.advertise<geometry_msgs::TwistWithCovarianceStamped>("publisherTwistEkf", 10);

    }

private:
//    std::deque<sensor_msgs::Imu::ConstPtr> imuDeque;
//    std::deque<mavros_msgs::Altitude::ConstPtr> depthDeque;
//    std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDeque;
    ekfClassDVL currentEkf;
    ros::Subscriber subscriberIMU, subscriberDepth, subscriberHeading, subscriberDVL, subscriberSlamResults, subscriberVelocityMavros;
    ros::Publisher publisherPoseEkf, publisherTwistEkf;
    std::mutex updateSlamMutex;
    Eigen::Quaterniond rotationOfDVL;
    ros::ServiceServer serviceResetEkf;

    void imuCallbackHelper(const sensor_msgs::Imu::ConstPtr &msg) {
        Eigen::Quaterniond tmpRot;
        tmpRot.x() = msg.get()->orientation.x;
        tmpRot.y() = msg.get()->orientation.y;
        tmpRot.z() = msg.get()->orientation.z;
        tmpRot.w() = msg.get()->orientation.w;

//        std::cout << msg->linear_acceleration.x<< " " << msg->linear_acceleration.y<< " " << msg->linear_acceleration.z << std::endl;

        currentEkf.predictionImu(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                                 tmpRot,
                                 msg->header.stamp);

        Eigen::Vector3d euler = generalHelpfulTools::getRollPitchYaw(tmpRot);// roll pitch yaw

        //calculate roll pitch from IMU accel data
//        std::cout << "my Roll: "<< euler.x()*180/M_PI<< std::endl;
//        std::cout << "my Pitch: "<< euler.y()*180/M_PI<< std::endl;
//        std::cout << "my Yaw: "<< euler.z()*180/M_PI<< std::endl;
        currentEkf.updateIMU(euler.x(), euler.y(), msg.get()->angular_velocity.x, msg.get()->angular_velocity.y,
                             msg.get()->angular_velocity.z, tmpRot, msg->header.stamp);
        pose currentStateEkf = currentEkf.getState();
        geometry_msgs::PoseWithCovarianceStamped poseMsg;
        poseMsg.header.frame_id = "map_ned";
        poseMsg.pose.pose.position.x = currentStateEkf.position.x();
        poseMsg.pose.pose.position.y = currentStateEkf.position.y();
        poseMsg.pose.pose.position.z = currentStateEkf.position.z();
        Eigen::Quaterniond rotDiff = currentEkf.getRotationVector();
        poseMsg.pose.pose.orientation.x = rotDiff.x();
        poseMsg.pose.pose.orientation.y = rotDiff.y();
        poseMsg.pose.pose.orientation.z = rotDiff.z();
        poseMsg.pose.pose.orientation.w = rotDiff.w();
//        std::cout << msg->header.stamp << std::endl;
        poseMsg.header.stamp = msg->header.stamp;
        this->publisherPoseEkf.publish(poseMsg);
        geometry_msgs::TwistWithCovarianceStamped twistMsg;
        twistMsg.header.frame_id = "map_ned";
        twistMsg.twist.twist.linear.x = currentStateEkf.velocity.x();
        twistMsg.twist.twist.linear.y = currentStateEkf.velocity.y();
        twistMsg.twist.twist.linear.z = currentStateEkf.velocity.z();
        twistMsg.twist.twist.angular.x = currentStateEkf.angleVelocity.x();
        twistMsg.twist.twist.angular.y = currentStateEkf.angleVelocity.y();
        twistMsg.twist.twist.angular.z = currentStateEkf.angleVelocity.z();
        twistMsg.header.stamp = msg->header.stamp;
        this->publisherTwistEkf.publish(twistMsg);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->imuCallbackHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackDVLHelper(const waterlinked_dvl::TransducerReportStamped::ConstPtr &msg) {
        if (msg->report.status == -1) {
            //if we dont know anything, the ekf should just go to 0, else the IMU gives direction.
            this->currentEkf.updateDVL(0, 0, 0, this->rotationOfDVL, msg->header.stamp);
        } else {
            this->currentEkf.updateDVL(msg->report.vx, msg->report.vy, msg->report.vz, this->rotationOfDVL,
                                       msg->header.stamp);
        }
    }

    void DVLCallbackDVL(const waterlinked_dvl::TransducerReportStamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackDVLHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackSimulationHelper(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        this->currentEkf.updateDVL(msg->vector.x, msg->vector.y, msg->vector.z, Eigen::Quaterniond(1, 0, 0, 0),
                                   msg->header.stamp);
    }

    void DVLCallbackSimulation(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackSimulationHelper(msg);
        this->updateSlamMutex.unlock();
    }

    bool resetEKF(commonbluerovmsg::resetekf::Request &req, commonbluerovmsg::resetekf::Response &res) {
        this->updateSlamMutex.lock();
        this->currentEkf.resetToPos(req.xPos, req.yPos, req.yaw, req.resetCovariances);
        this->updateSlamMutex.unlock();
        res.resetDone = true;
        return true;
    }

    void depthSensorCallback(const commonbluerovmsg::heightStamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->depthSensorHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void depthSensorHelper(const commonbluerovmsg::heightStamped::ConstPtr &msg) {
        this->currentEkf.updateHeight(msg->height, msg->header.stamp);
    };

    void headingCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->headingHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void headingHelper(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        this->currentEkf.updateHeading(msg->vector.z, msg->header.stamp);
    };
public:
    pose getPoseOfEKF() {
        return this->currentEkf.getState();
    }

    void callbackReconfiguration(underwaterslam::ekfnoiseConfig &config, uint32_t level) {
        this->updateSlamMutex.lock();

        this->currentEkf.setProcessNoise(config.processNoiseX, config.processNoiseY, config.processNoiseZ,
                                         config.processNoiseVX, config.processNoiseVY, config.processNoiseVZ,
                                         config.processNoiseRoll, config.processNoisePitch, config.processNoiseYaw,
                                         config.processNoiseVelRoll, config.processNoiseVelPitch,
                                         config.processNoiseVelYaw);

        this->currentEkf.setMeasurementNoiseDVL(config.measurementNoiseDVLVX, config.measurementNoiseDVLVY,
                                                config.measurementNoiseDVLVZ);

        this->currentEkf.setMeasurementNoiseDepth(config.measurementNoiseDepth);

        this->currentEkf.setMeasurementNoiseIMUVel(config.measurementImuVelocityRoll,
                                                   config.measurementImuVelocityPitch,
                                                   config.measurementImuVelocityVelRoll,
                                                   config.measurementImuVelocityVelPitch,
                                                   config.measurementImuVelocityVelYaw);

        this->updateSlamMutex.unlock();
    }
};


Eigen::Quaterniond getQuaternionForMavrosFromRPY(double roll, double pitch, double yaw) {
    return generalHelpfulTools::getQuaternionFromRPY(roll, -pitch, -yaw + M_PI / 2);
}


void spinningRos() {
    ros::spin();
}

int main(int argc, char **argv) {
//    std::this_thread::sleep_for(std::chrono::milliseconds(5000));


    ros::init(argc, argv, "ekffordvlwithros");
    ros::start();
    ros::NodeHandle n_;

    std::string whichIMUUsed;
    if (n_.getParam("/EKFDVL/imu_used", whichIMUUsed)) {
        ROS_INFO("IMU used is: %s", whichIMUUsed.c_str());
    } else {
        std::vector<std::string> keys;
        n_.getParamNames(keys);

        for (int i = 0; i < keys.size(); i++) {
            std::cout << keys[i] << std::endl;
        }
        ROS_ERROR("Failed to get IMU parameter, which to use");
    }

    if (whichIMUUsed != "external" && whichIMUUsed != "px4") {
        ROS_ERROR("You have to use px4 or external as parameter for imu_used");
        exit(-1);
    }

    std::string whichDVLUsed;
    if (n_.getParam("/EKFDVL/dvl_used", whichDVLUsed)) {
        ROS_INFO("DVL used is: %s", whichDVLUsed.c_str());
    } else {
        std::vector<std::string> keys;
        n_.getParamNames(keys);

        for (int i = 0; i < keys.size(); i++) {
            std::cout << keys[i] << std::endl;
        }
        ROS_ERROR("Failed to get DVL parameter, which to use");
    }

    if (whichDVLUsed != "external" && whichDVLUsed != "gazebo") {
        ROS_ERROR("You have to use gazebo or external as parameter for dvl_used");
        exit(-1);
    }


    rosClassEKF rosClassEKFObject(n_, 3.14159 / 4.0, whichIMUUsed, whichDVLUsed);
    //ros::spin();
    std::thread t1(spinningRos);

    dynamic_reconfigure::Server<underwaterslam::ekfnoiseConfig> server;
    dynamic_reconfigure::Server<underwaterslam::ekfnoiseConfig>::CallbackType f;

    f = boost::bind(&rosClassEKF::callbackReconfiguration, &rosClassEKFObject, _1, _2);
    server.setCallback(f);


    ros::Publisher publisherPoseEkf = n_.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    ros::Rate ourRate = ros::Rate(30);
    //sending position to Mavros mavros/vision_pose/pose
    while (ros::ok()) {
        pose currentPoseEkf = rosClassEKFObject.getPoseOfEKF();
        geometry_msgs::PoseStamped msg_vicon_pose;
        msg_vicon_pose.header.stamp = currentPoseEkf.timeLastPrediction;
        msg_vicon_pose.header.frame_id = "map_ned"; //optional. Works fine without frame_id

        Eigen::AngleAxisf rotation_vector180X(180.0 / 180.0 * 3.14159, Eigen::Vector3f(1, 0, 0));
        Eigen::AngleAxisf rotation_vector90Z(90.0 / 180.0 * 3.14159, Eigen::Vector3f(0, 0, 1));
        Eigen::Vector3f positionRotatedForMavros =
                rotation_vector90Z.toRotationMatrix() * rotation_vector180X.toRotationMatrix() *
                currentPoseEkf.position;

        msg_vicon_pose.pose.position.x = positionRotatedForMavros.x();
        msg_vicon_pose.pose.position.y = positionRotatedForMavros.y();
        msg_vicon_pose.pose.position.z = positionRotatedForMavros.z();
        Eigen::Quaterniond currentRotation = getQuaternionForMavrosFromRPY(currentPoseEkf.rotation.x(),
                                                                           currentPoseEkf.rotation.y(),
                                                                           currentPoseEkf.rotation.z());


        msg_vicon_pose.pose.orientation.x = currentRotation.x();
        msg_vicon_pose.pose.orientation.y = currentRotation.y();
        msg_vicon_pose.pose.orientation.z = currentRotation.z();
        msg_vicon_pose.pose.orientation.w = currentRotation.w();
        publisherPoseEkf.publish(msg_vicon_pose);
        ourRate.sleep();
    }
    return (0);
}
