//
// Created by tim-linux on 01.03.22.
//

#include "softDescriptorRegistration.h"

int main(int argc,
         char **argv) {
    int N = 128;
    int bwIn = N / 2;
    int bwOut = N / 2;
    int degLim = bwOut - 1;
    double cellSize = 0.5;

    softDescriptorRegistration myRegistrationClass(N,bwOut,bwIn,degLim);



    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/after_voxel_second.pcd",
                         *pointCloudInputData1);

    Eigen::Matrix4d transformationPCL;
    //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd rotation_vector2(-90.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationPCL.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationPCL(0, 3) = 4;
    transformationPCL(1, 3) = 10;
    transformationPCL(2, 3) = 0;
    transformationPCL(3, 3) = 1;
    //copy the rotated PCL from PCL1 to PCL2
    pcl::transformPointCloud(*pointCloudInputData1, *pointCloudInputData2, transformationPCL);




    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Eigen::Matrix4d resultingTransformation = myRegistrationClass.registrationOfTwoPCL(pointCloudInputData1,pointCloudInputData2,cellSize);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference complete Registration = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
              << "[ms]" << std::endl;
    std::cout << resultingTransformation <<std::endl;
    return 1;
}