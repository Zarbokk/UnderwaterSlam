//
// Created by tim-linux on 04.01.22.
//

#include "ros/ros.h"
#include "ping360_sonar/SonarEcho.h"
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


cv::Mat sonarImage;
ros::Publisher publisher;

class correctionOfSonarData {
public:
    correctionOfSonarData() {
        this->publisherImage = n_.advertise<sensor_msgs::Image>("sonar/imageCorrected", 10);
        this->numberOfFrames = 0;
        this->lastAngle = 0;
        this->sizeImage = 500;
        this->radiusOfVisablePart = (this->sizeImage - 100) / 2;
        this->startTimeOfCorrection = 0;
        this->sonarImage = cv::Mat(sizeImage, sizeImage, CV_8UC1, cv::Scalar(0));
        this->rotationOfSonarOnRobot = 200;
//        this->subscriberDataSonar = n_.subscribe("sonar/intensity", 1000,
//                                                 &correctionOfSonarData::imageDataGenerationCallback, this);
        this->subscriberDataSonar = n_.subscribe("ping360_node/sonar/data", 1000,
                                                 &correctionOfSonarData::imageDataGenerationCallback, this);
        this->subscriberEKFData = n_.subscribe("publisherPoseEkf", 1000, &correctionOfSonarData::ekfPoseCallback, this);

    }

private:
    ros::NodeHandle n_;
    ros::Publisher publisherImage;
    ros::Subscriber subscriberDataSonar, subscriberEKFData;
    std::deque<double> xPositionVector, yPositionVector, zPositionVector, timeVector;//,yawAngleVector,pitchAngleVector,rollAngleVector;
    std::deque<Eigen::Quaterniond> rotationVector;
    double lastAngle, rotationOfSonarOnRobot;
    cv::Mat sonarImage;
    int sizeImage, radiusOfVisablePart,numberOfFrames;
    double startTimeOfCorrection;
    std::mutex updateMutex;

    std::vector<double> linspace(double start_in, double end_in, int num_in) {
        if (num_in < 0) {
            std::cout << "number of linspace negative" << std::endl;
            exit(-1);
        }
        std::vector<double> linspaced;

        double start = start_in;
        double end = end_in;
        auto num = (double) num_in;

        if (num == 0) { return linspaced; }
        if (num == 1) {
            linspaced.push_back(start);
            return linspaced;
        }

        double delta = (end - start) / (num - 1);//stepSize

        for (int i = 0; i < num - 1; ++i) {
            linspaced.push_back(start + delta * i);
        }
        linspaced.push_back(end); // I want to ensure that start and end
        // are exactly the same as the input
        return linspaced;
    }

    void imageDataGenerationCallback(const ping360_sonar::SonarEcho::ConstPtr &msg) {
        if (this->startTimeOfCorrection == 0) {
            this->startTimeOfCorrection = msg->header.stamp.toSec();
        }
        if(this->timeVector.empty()){
            return;
        }
        if (this->lastAngle > msg->angle) {
            std::string dataFolder = "/home/tim-linux/dataFolder/ValentinBunkerDataCorrection/";
//            std::string dataFolder = "/home/tim-linux/dataFolder/correctedFramesGazebo/";
            cv::imwrite(dataFolder + "frameNumber" + std::to_string(this->numberOfFrames) + ".jpg", this->sonarImage);
            this->numberOfFrames++;
            //reset image and start correction
            this->sonarImage = cv::Mat(sizeImage, sizeImage, CV_8UC1, cv::Scalar(0));
            this->startTimeOfCorrection = msg->header.stamp.toSec();
            clearSavingsOfPoses(msg->header.stamp.toSec());


            //add number in top left corner
            cv::rectangle(sonarImage, cv::Point(0, 0), cv::Point(65, 65), 0, cv::FILLED);
            std::string tmp = std::to_string(msg->range);
            cv::putText(sonarImage, tmp, cv::Point(2, 40), cv::FONT_ITALIC, 1, 255);
//            std::cout << " Starting new Rotation: " << std::endl;


        }

        Eigen::Matrix4d poseDiff = calculationPositionDifference(this->startTimeOfCorrection,
                                                                 msg->header.stamp.toSec());
//        std::cout << " Pose diff: " << poseDiff(0, 3) << " " << poseDiff(1, 3) << std::endl;
        double linear_factor = ((double) msg->number_of_samples) / ((double) this->radiusOfVisablePart);
        double pixelToMeter = ((double) msg->range) / ((double) this->radiusOfVisablePart);
        for (int i = 0; i < this->radiusOfVisablePart; i++) {
            double color = 0;

            if (i > 0) {
                color = msg->intensities[i * linear_factor - 1];
            }
//            if (i < 1) {
//                color = 240 / 1.3;
//            }

            double stepSize = 1;
            std::vector<double> linspaceVector = linspace(-stepSize / 2, stepSize / 2, 10);
            for (const auto &value: linspaceVector) {
                double theta = 2 * M_PI * (msg->angle + value) /
                               400.0;// this->rotationOfSonarOnRobot minus because of the coordinate change from z to top to z to bottom
                double x = i * cos(theta);
                double y = i * sin(theta);
                Eigen::Vector4d positionInMeter = Eigen::Vector4d(pixelToMeter * x, pixelToMeter * y, 0, 1);
                Eigen::Vector4d correctedPoseInPixel = (1 / pixelToMeter) * poseDiff * positionInMeter;
//                std::cout << correctedPoseInPixel << std::endl;

                sonarImage.at<uchar>((int) (((double) sonarImage.size[0] / 2.0) - correctedPoseInPixel[0]) - 1,
                                     (int) (((double) sonarImage.size[0] / 2.0) + correctedPoseInPixel[1]) - 1) =
                        color * 1.2;
            }
        }


        this->lastAngle = msg->angle;
        sensor_msgs::ImagePtr imageMessage = cv_bridge::CvImage(std_msgs::Header(), "mono8", sonarImage).toImageMsg();

        this->publisherImage.publish(imageMessage);
    }


    void ekfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(this->updateMutex);
        double currentTimeStamp = msg->header.stamp.toSec();
        // calculate where to put the current new message
        int i = 0;
        if (!this->timeVector.empty()) {
            i = this->timeVector.size();
            while (this->timeVector[i - 1] > currentTimeStamp) {
                i--;
            }
        }

        if (i == this->timeVector.size() || i == 0) {
            Eigen::Quaterniond tmpQuad(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            this->rotationVector.push_back(tmpQuad);
            this->timeVector.push_back(msg->header.stamp.toSec());
            this->xPositionVector.push_back(msg->pose.pose.position.x);
            this->yPositionVector.push_back(msg->pose.pose.position.y);
            this->zPositionVector.push_back(msg->pose.pose.position.z);
        } else {
            std::cout << "we test it" << std::endl;
            exit(0);
        }
    }

    static Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat) {
        tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
        tf2::Matrix3x3 m(tmp);
        double r, p, y;
        m.getRPY(r, p, y);
        Eigen::Vector3d returnVector(r, p, y);
        return returnVector;
    }

    Eigen::Matrix4d calculationPositionDifference(double fromTime, double toTime) {

        std::lock_guard<std::mutex> lock(this->updateMutex);
        //find index of start/end time:
        int indexOfStart = 0;
        while (this->timeVector[indexOfStart] < fromTime && this->timeVector.size() > indexOfStart) {
            indexOfStart++;
        }
        indexOfStart--;
        int indexOfEnd = 0;
        while (this->timeVector[indexOfEnd] < toTime && this->timeVector.size() > indexOfEnd) {
            indexOfEnd++;
        }
        indexOfEnd--;

        Eigen::Matrix4d transformationStart = Eigen::Matrix4d::Zero();
        transformationStart.block<3, 3>(0, 0) = this->rotationVector[indexOfStart].toRotationMatrix();
        transformationStart(0, 3) = this->xPositionVector[indexOfStart];
        transformationStart(1, 3) = this->yPositionVector[indexOfStart];
        transformationStart(2, 3) = this->zPositionVector[indexOfStart];
        transformationStart(3, 3) = 1;

        Eigen::Matrix4d transformationEnd = Eigen::Matrix4d::Zero();
        transformationEnd.block<3, 3>(0, 0) = this->rotationVector[indexOfEnd].toRotationMatrix();
        transformationEnd(0, 3) = this->xPositionVector[indexOfEnd];
        transformationEnd(1, 3) = this->yPositionVector[indexOfEnd];
        transformationEnd(2, 3) = this->zPositionVector[indexOfEnd];
        transformationEnd(3, 3) = 1;
        Eigen::Matrix4d returnMatrix = transformationStart.inverse()*transformationEnd;
        return returnMatrix;
    }

    void clearSavingsOfPoses(double upToTime) {
        std::lock_guard<std::mutex> lock(this->updateMutex);
        while (this->timeVector[0] < upToTime - 1) {//one second puffer
//            this->rollAngleVector.pop_front();
//            this->pitchAngleVector.pop_front();
//            this->yawAngleVector.pop_front();

            this->rotationVector.pop_front();
            this->timeVector.pop_front();
            this->xPositionVector.pop_front();
            this->yPositionVector.pop_front();
            this->zPositionVector.pop_front();
        }
    }

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "conversionofsonardatatoimagewithekf");
    ros::start();
    correctionOfSonarData ourClass;
//    ros::NodeHandle n_;
    //has to be squared
//    int sizeMat = 500;
//    sonarImage = cv::Mat(sizeMat, sizeMat, CV_8UC1, cv::Scalar(0));


    ros::spin();
    return 0;
}
