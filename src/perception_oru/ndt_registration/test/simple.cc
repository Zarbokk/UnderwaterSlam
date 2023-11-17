#include <ndt_matcher_p2d.h>
#include <ndt_matcher_d2d_2d.h>
#include <ndt_matcher_d2d.h>
#include <ndt_map.h>
#include <pointcloud_utils.h>

#include <pcl/io/ply_io.h>


#include "pcl/point_cloud.h"
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <iostream>
#include <sstream>
#include <pcl/common/transforms.h>

using namespace std;

int
main (int argc, char** argv)
{
//    if(argc!=9) {
//	std::cout<<"usage "<<argv[0]<<" x y z r p t cloud1.wrl cloud2.wrl\n";
//	return -1;
//    }

//    istringstream roll_c(argv[4]),pitch_c(argv[5]),yaw_c(argv[6]),xoffset_c(argv[1]),yoffset_c(argv[2]),zoffset_c(argv[3]);
    double roll,pitch,yaw,xoffset,yoffset,zoffset;
    roll = 0;
    pitch = 0;
    yaw = 0;

//    roll_c >> roll;
//    pitch_c >> pitch;
//    yaw_c >> yaw;
//    xoffset_c >> xoffset;
//    yoffset_c >> yoffset;
//    zoffset_c >> zoffset;
    xoffset = 2;
    yoffset = 1;
    zoffset = 0;



    printf("X %f Y %f Z %f Roll %f Pitch %f Yaw %f \n",xoffset,yoffset,zoffset,roll,pitch,yaw);	
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud_offset,cloud2, cloud_offset2;
    char fname[50];
    FILE *fout;
    double __res[] = {0.5, 1,2,4};
    std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));

    struct timeval tv_start,tv_end,tv_reg_start,tv_reg_end;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tout;
    Tout.setIdentity();
    //printf("Working");	


    gettimeofday(&tv_start,NULL);
    //we do a single scan to scan registration
    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",
                               cloud);
    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",
                               cloud_offset);
//    pcl::PointCloud<pcl::PointXYZ> cloudShifted = cloud;
//



//
//
    pcl::io::loadPLYFile("/home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_2/0_Threshold.ply",
                         cloud2);
    pcl::io::loadPLYFile("/home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_2/0_ThresholdShifted.ply",
                         cloud_offset2);
//
//    while(cloud2.points.empty()){
//        cloud2.points.pop_back();
//    }
//
//    std::cout << cloud2.points.size()<< std::endl;
//    while(cloud_offset2.empty()){
//        cloud_offset2.points.pop_back();
//    }
//
//
//    for(int i = 0 ; i<cloud.points.size();i++){
//        pcl::PointXYZ tmpPoint;
//        tmpPoint.x = cloud.points[i].x;
//        tmpPoint.y = cloud.points[i].y;
//        tmpPoint.z = cloud.points[i].z;
//        cloud2.points.push_back(tmpPoint);
//    }
//
//
//    for(int i = 0 ; i<cloud_offset.points.size();i++){
//        pcl::PointXYZ tmpPoint;
//        tmpPoint.x = cloud_offset.points[i].x;
//        tmpPoint.y = cloud_offset.points[i].y;
//        tmpPoint.z = cloud_offset.points[i].z;
//        cloud_offset2.points.push_back(tmpPoint);
//    }

    Tout =  Eigen::Translation<double,3>(0,0,0)*
            Eigen::AngleAxis<double>(0.001,Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ()) ;


    pcl::transformPointCloud(cloud, cloud, Tout);
    pcl::transformPointCloud(cloud_offset, cloud_offset, Tout);





//    for(int i = 0 ; i<cloud.points.size();i++){
//        cloud.points[i].z = cloud2.points[0].z;
//    }
//
//    for(int i = 0 ; i<cloud_offset.points.size();i++){
//        cloud_offset.points[i].z = cloud2.points[0].z;
//    }



    Tout =  Eigen::Translation<double,3>(xoffset,yoffset,zoffset)*
        Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;

    std::cout<<"Transform Before: \n"<<Tout.matrix()<<std::endl;
    //lslgeneric::NDTMatcherD2D_2D<pcl::PointXYZ,pcl::PointXYZ> matcherD2D(false, false, resolutions);
    lslgeneric::NDTMatcherD2D_2D<pcl::PointXYZ,pcl::PointXYZ> matcherD2D(false, false, resolutions);
    bool ret = matcherD2D.match(cloud,cloud_offset,Tout,true);

    std::cout<<"Transform: \n"<<Tout.matrix()<<std::endl;
    std::cout <<"for the stop" << std::endl;
    //Tout.setIdentity();

    //Tout =  Eigen::Translation<double,3>(xoffset,yoffset,zoffset)*
    //    Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
    //    Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
    //    Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;



//    snprintf(fname,49,"c_offset.wrl");
//    fout = fopen(fname,"w");
//    fprintf(fout,"#VRML V2.0 utf8\n");
//    lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud,Eigen::Vector3d(1,0,0));
//    lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud_offset,Eigen::Vector3d(1,1,1));
//
//    lslgeneric::transformPointCloudInPlace<pcl::PointXYZ>(Tout,cloud_offset);
//    lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud_offset,Eigen::Vector3d(0,1,0));
//    fclose(fout);

}
