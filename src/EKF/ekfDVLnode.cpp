//
// Created by jurobotics on 13.09.21.
//
#include "ekfDVL.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "generalHelpfulTools.h"
#include "waterlinked_dvl/TransducerReportStamped.h"
#include "commonbluerovmsg/resetekf.h"
//#include <chrono>
//#include <thread>



class rosClassEKF {
public:
    rosClassEKF(ros::NodeHandle n_, double rotationOfDVLZ,std::string imuUsage) : currentEkf(ros::Time::now()) {
        this->rotationOfDVL = Eigen::AngleAxisd(rotationOfDVLZ, Eigen::Vector3d::UnitZ());//yaw rotation for correct alignment of DVL data;
        if(imuUsage == "external"){
            this->subscriberIMU = n_.subscribe("imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        }else{
            this->subscriberIMU = n_.subscribe("mavros/imu/data_frd", 1000, &rosClassEKF::imuCallback, this);
        }

        this->subscriberEKF = n_.subscribe("transducer_report", 1000, &rosClassEKF::DVLCallbackDVL, this);
        this->subscriberVelocityMavros = n_.subscribe("mavros/local_position/velocity_body_frd", 1000, &rosClassEKF::DVLCallbackMavros, this);
        this->subscriberDepth = n_.subscribe("mavros/altitude_frd", 1000, &rosClassEKF::depthSensorCallback, this);
        this->subscriberHeading = n_.subscribe("magnetic_heading", 1000, &rosClassEKF::headingCallback, this);

        this->serviceResetEkf = n_.advertiseService("resetCurrentEKF",&rosClassEKF::resetEKF,this);

        this->publisherPoseEkf = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("publisherPoseEkf", 10);
        this->publisherTwistEkf = n_.advertise<geometry_msgs::TwistWithCovarianceStamped>("publisherTwistEkf", 10);

    }

private:
//    std::deque<sensor_msgs::Imu::ConstPtr> imuDeque;
//    std::deque<mavros_msgs::Altitude::ConstPtr> depthDeque;
//    std::deque<geometry_msgs::TwistStamped::ConstPtr> dvlDeque;
    ekfClassDVL currentEkf;
    ros::Subscriber subscriberIMU, subscriberDepth, subscriberHeading, subscriberEKF, subscriberSlamResults,subscriberVelocityMavros;
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
        this->currentEkf.updateDVL(msg->report.vx, msg->report.vy, msg->report.vz, this->rotationOfDVL, msg->header.stamp);
    }

    void DVLCallbackDVL(const waterlinked_dvl::TransducerReportStamped::ConstPtr &msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackDVLHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void DVLCallbackMavrosHelper(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        this->currentEkf.updateDVL(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, Eigen::Quaterniond(1,0,0,0), msg->header.stamp);
    }

    void DVLCallbackMavros(const geometry_msgs::TwistStamped::ConstPtr  &msg) {
        this->updateSlamMutex.lock();
        this->DVLCallbackMavrosHelper(msg);
        this->updateSlamMutex.unlock();
    }

    bool resetEKF(commonbluerovmsg::resetekf::Request  &req, commonbluerovmsg::resetekf::Response &res){
        this->updateSlamMutex.lock();
        this->currentEkf.resetToPos(req.xPos,req.yPos,req.yaw,req.resetCovariances);
        this->updateSlamMutex.unlock();
        res.resetDone = true;
        return true;
    }

    void depthSensorCallback(const mavros_msgs::Altitude ::ConstPtr  &msg){
        this->updateSlamMutex.lock();
        this->depthSensorHelper(msg);
        this->updateSlamMutex.unlock();
    }
    void depthSensorHelper(const mavros_msgs::Altitude ::ConstPtr  &msg){
        this->currentEkf.updateHeight(msg->local,msg->header.stamp);
    };

    void headingCallback(const geometry_msgs::Vector3Stamped::ConstPtr  &msg){
        this->updateSlamMutex.lock();
        this->headingHelper(msg);
        this->updateSlamMutex.unlock();
    }

    void headingHelper(const geometry_msgs::Vector3Stamped::ConstPtr  &msg){
        this->currentEkf.updateHeading(msg->vector.z,msg->header.stamp);
    };

};


int main(int argc, char **argv) {
//    std::this_thread::sleep_for(std::chrono::milliseconds(5000));


    ros::init(argc, argv, "ekffordvlwithros");
    ros::start();
    ros::NodeHandle n_;

    std::string s;
    if (n_.getParam("/EKFDVL/imu_used", s))
    {
        ROS_INFO("IMU used is: %s", s.c_str());
    }
    else
    {
        std::vector<std::string> keys;
        n_.getParamNames(keys);

        for(int i = 0;i<keys.size();i++){
            std::cout << keys[i]<< std::endl;
        }

        ROS_ERROR("Failed to get IMU parameter, which to use");
    }

    if(s!="external" && s!="px4"){
        ROS_ERROR("You have to use px4 or external as parameter for imu_used");
        exit(-1);
    }

    rosClassEKF rosClassForTests(n_,3.14159 / 4.0,s);

    ros::spin();


    return (0);
}
