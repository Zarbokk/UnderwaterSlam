//
// Created by tim-linux on 26.03.22.
//

//
// Created by jurobotics on 13.09.21.
//


#include "generalHelpfulTools.h"
#include <slamToolsRos.h>


int main(int argc, char **argv) {

    std::string current_exec_name = argv[0]; // Name of the current exec program
    std::vector<std::string> all_args;

    if (argc > 0) {
        //std::cout << "temp1" << std::endl;
        all_args.assign(argv + 1, argv + argc);
        //std::cout << "12"<< all_args[1]<<std::endl;
    }else{
        std::cout << "no arguments given" << std::endl;
        exit(-1);
    }

//    for(int i=0;i<all_args.size();i++){
//        std::cout << "here: "<< i << " string: " << all_args[i]<< std::endl;
//    }
//    std::cout<< "test" << std::endl;
//    std::cout << "Keyframe: " << all_args[0]<< std::endl;
//    std::cout<< "test1" << std::endl;
//    std::cout << "Keyframe: " << all_args[1]<< std::endl;
//    std::cout<< "test2" << std::endl;








    scanRegistrationClass scanRegistrationObject;

//    for (int numberOfScan = 15; numberOfScan < 100; numberOfScan++) {
//        std::cout << "current KeyFrame: " << numberOfScan << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile(
            all_args[0],
            *scan1);
    pcl::io::loadPCDFile(
            all_args[1],
            *scan2);
    double fitnessX,fitnessY;
    Eigen::Matrix4d estimatedTransformation = scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,-100,false);

    scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0)),true);

//        printf("Press [Enter] key to continue.\n");

//        while(getchar()!='\n'); // option TWO to clean stdin
//        getchar(); // wait for ENTER


//    }


    return (0);
}
