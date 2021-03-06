#include <slamToolsRos.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <condition_variable>
#include <mutex>
#include <signal.h>

class rosClassSlam {
public:
    rosClassSlam(ros::NodeHandle n_) : graphSaved(3) {
        publisherKeyFrameClouds = n_.advertise<sensor_msgs::PointCloud2>("currentScanTransformed", 10);
        publisherPathOverTime = n_.advertise<nav_msgs::Path>("positionOverTime", 10);
        publisherPathOverTimeGT = n_.advertise<nav_msgs::Path>("positionOverTimeGT", 10);
        publisherMarkerArray = n_.advertise<visualization_msgs::MarkerArray>("covariance", 10);
        publisherMarkerArrayLoopClosures = n_.advertise<visualization_msgs::MarkerArray>("loopClosures", 10);
        publisherLastPCL = n_.advertise<sensor_msgs::PointCloud2>("lastPCL", 10);
        publisherRegistrationPCL = n_.advertise<sensor_msgs::PointCloud2>("registratedPCL", 10);
        publisherBeforeCorrection = n_.advertise<sensor_msgs::PointCloud2>("beforeCorrection", 10);
        publisherAfterCorrection = n_.advertise<sensor_msgs::PointCloud2>("afterCorrection", 10);

        subscriberFullScan = n_.subscribe("sonar/full_scan", 10000, &rosClassSlam::sonarFullScanCallback, this);
        subscriberIMU = n_.subscribe("mavros/imu/data_frd", 10000, &rosClassSlam::imuCallback, this);
        subscriberVelocity = n_.subscribe("mavros/local_position/velocity_body_frd", 10000, &rosClassSlam::velocityCallback,
                                          this);

        Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
        //Eigen::AngleAxisd rotation_vector2(0.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));

        Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
        transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
        transformationX180Degree(3, 3) = 1;

        graphSaved.addVertex(0, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0),
                             Eigen::Vector3d(0, 0, 0), 0, 0.0, graphSlamSaveStructure::FIRST_ENTRY);

        std::deque<double> subgraphs{1, 3};
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
        this->imuDataList.clear();
        this->dvlDataList.clear();

    }


private:
    ros::Subscriber subscriberIMU, subscriberFullScan, subscriberVelocity;
    ros::Publisher publisherKeyFrameClouds, publisherPathOverTime, publisherMarkerArray, publisherPathOverTimeGT, publisherMarkerArrayLoopClosures, publisherLastPCL, publisherRegistrationPCL, publisherBeforeCorrection, publisherAfterCorrection;
    //PCL::
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentScan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastScan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final;

    //Matrices:
    Eigen::Matrix4d currentTransformation;
    //Eigen::Matrix4d transformation90Degree;
    Eigen::Matrix4d initialGuessTransformation;
    Eigen::Matrix4d transformationX180Degree;


    std::vector<vertex> posDiffOverTimeVertices;
    std::vector<edge> posDiffOverTimeEdges;
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


    void sonarFullScanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
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
        std::deque<ImuData> lastImuData(3);
        //std::unique_lock<std::mutex> lck(this->mutex_);
        //std::cout << "test123"<< std::endl;
        //this->imuFree.wait(lck);
        //std::cout << "done waiting"<< std::endl;
        this->imuFree.lock();
        lastImuData = this->imuDataList;
        this->imuFree.unlock();
        std::deque<DvlData> lastDvlData(3);
        //std::unique_lock<std::mutex> lck2(this->mutex_);
        //dvlFree.wait(lck2);
        this->dvlFree.lock();
        lastDvlData = this->dvlDataList;
        this->dvlFree.unlock();
        this->posDiffOverTimeEdges.clear();
        slamToolsRos::calculatePositionOverTime(lastImuData,
                                                lastDvlData,
                                                this->posDiffOverTimeEdges, this->timeLastFullScan,
                                                this->timeCurrentFullScan, 0.1,this->numberOfEdgesBetweenScans);
        slamToolsRos::appendEdgesToGraph(this->graphSaved, posDiffOverTimeEdges, 0.3, 0.25, maxTimeOptimization);
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
        graphSaved.addEdge(this->graphSaved.getVertexList().size() - positionLastPcl, graphSaved.getVertexList().size() - 1,
                           this->currentTransformation.block<3, 1>(0, 3), qTMP,
                           Eigen::Vector3d(sqrt(this->fitnessScore), sqrt(this->fitnessScore), 0),
                           0.25 * sqrt(this->fitnessScore),
                           graphSlamSaveStructure::POINT_CLOUD_USAGE,maxTimeOptimization);//@TODO still not sure about size

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

        graphSaved.optimizeGraphWithSlamTopDown(false, 0.05,maxTimeOptimization);
//        std::vector<int> holdStill{0};
//        graphSaved.optimizeGraphWithSlam(false, holdStill,maxTimeOptimization);

        graphSaved.calculateCovarianceInCloseProximity(maxTimeOptimization);

        slamToolsRos::visualizeCurrentGraph(this->graphSaved, this->publisherPathOverTime,
                                            this->publisherKeyFrameClouds,
                                            this->publisherMarkerArray, this->sigmaScaling,
                                            this->publisherPathOverTimeGT,
                                            NULL, this->publisherMarkerArrayLoopClosures,
                                            NULL,this->numberOfEdgesBetweenScans);
        std::cout << "next: " << std::endl;
        this->timeLastFullScan = this->timeCurrentFullScan;
        *this->lastScan = *this->graphSaved.getVertexList().back().getPointCloudCorrected();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        // create a vector, which is saving the IMU data
        // after that delete the last entry(if the time is before scan of last time)
        // Rotation of IMU callback from mavros to ROS(NED)(with minus here)
        ImuData currentImuDataPoint;
        currentImuDataPoint.ax = msg->linear_acceleration.x;
        currentImuDataPoint.ay = msg->linear_acceleration.y;
        currentImuDataPoint.az = msg->linear_acceleration.z;
        currentImuDataPoint.wx = msg->angular_velocity.x;
        currentImuDataPoint.wy = msg->angular_velocity.y;
        currentImuDataPoint.wz = msg->angular_velocity.z;
        currentImuDataPoint.timeStamp = msg->header.stamp.toSec();
        imuFree.lock();
        this->imuDataList.push_back(currentImuDataPoint);

        while (this->imuDataList[0].timeStamp < this->timeLastFullScan-1.0f) {
            if (this->imuDataList.size() == 0) {
                break;
            }
            this->imuDataList.pop_front();

        }
        imuFree.unlock();

    }

    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        // create a vector, which is saving the velocity data
        // after that delete the last entry(if the time is before scan of last time)
        // Rotation of velocity callback from mavros to ROS(NED)(with minus here)
        DvlData currentDvlDataPoint;
        currentDvlDataPoint.vx = msg->twist.linear.x;
        currentDvlDataPoint.vy = msg->twist.linear.y;
        currentDvlDataPoint.vz = msg->twist.linear.z;
        currentDvlDataPoint.timeStamp = msg->header.stamp.toSec();
        this->dvlFree.lock();
        this->dvlDataList.push_back(currentDvlDataPoint);
        while (this->dvlDataList[0].timeStamp < this->timeLastFullScan-1.0f) {
            if (this->dvlDataList.size() == 0) {
                break;
            }
            this->dvlDataList.pop_front();
        }
        this->dvlFree.unlock();
    }
public:
    void shutDownHook(){
        this->subscriberFullScan.shutdown();
        std::cout << "Optimizing one Last Time(full)" << std::endl;
        std::vector<int> holdStill{0};
        graphSaved.optimizeGraphWithSlam(false, holdStill,maxTimeOptimization);
        slamToolsRos::visualizeCurrentGraph(this->graphSaved, this->publisherPathOverTime,
                                            this->publisherKeyFrameClouds,
                                            this->publisherMarkerArray, this->sigmaScaling,
                                            this->publisherPathOverTimeGT,
                                            NULL, this->publisherMarkerArrayLoopClosures,
                                            NULL,this->numberOfEdgesBetweenScans);


    }
};

sig_atomic_t volatile g_request_shutdown = 0;
void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.
    std::cout << "shutting down:" << std::endl;

    g_request_shutdown=1;
    // All the default sigint handler does is call shutdown()


}


int
main(int argc, char **argv) {


    ros::init(argc, argv, "graphslamwithros", ros::init_options::NoSigintHandler);
    ros::start();
    ros::NodeHandle n_;

    signal(SIGINT, mySigintHandler);
    rosClassSlam rosObjectForSlam(n_);

    while (!g_request_shutdown)
    {
        // Do non-callback stuff

        ros::spinOnce();
        usleep(100000);
    }
    //ros::spin();
    rosObjectForSlam.shutDownHook();
    ros::shutdown();
    return (0);
}
