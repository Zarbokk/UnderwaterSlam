//
// Created by tim-linux on 26.03.22.
//

//
// Created by jurobotics on 13.09.21.
//


#include "generalHelpfulTools.h"
#include "slamToolsRos.h"


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
    pcl::PointCloud<pcl::PointXYZ> scan1;
    pcl::PointCloud<pcl::PointXYZ> scan2;

//        std::vector<double> vectorInitialGuess,vectorBestGuess;


//    for(int i = 0 ; i<80 ; i++){


    pcl::io::loadPCDFile(
            all_args[0],
            scan1);
    pcl::io::loadPCDFile(
            all_args[1],
            scan2);
//    pcl::io::loadPCDFile(
//            "/home/tim-linux/dataFolder/gazeboCorrectedEvenAnglesPCLs_2_75/pclKeyFrame"+ std::to_string(i)+".pcd",
//            *scan1);
//    pcl::io::loadPCDFile(
//            "/home/tim-linux/dataFolder/gazeboCorrectedEvenAnglesPCLs_2_75/pclKeyFrame"+ std::to_string(i+1)+".pcd",
//            *scan2);
    double fitnessX,fitnessY;
//    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    Eigen::Matrix4d estimatedTransformation = scanRegistrationObject.sofftRegistration2D(scan1, scan2, fitnessX,
                                                                                         fitnessY, -100, true);
//    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//
//
//
//
//
//    std::cout << "Time difference complete Registration 4 solutions = "
//              << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
//              << "[ms]" << std::endl;

//    vectorBestGuess.push_back((double)(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()));


//    std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
//    scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,std::atan2(estimatedTransformation(1, 0), estimatedTransformation(0, 0)),false);
//    std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
//
//    std::cout << "Time difference complete Registration with initial guess = "
//              << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count()
//              << "[ms]" << std::endl;

//    vectorInitialGuess.push_back((double)(std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count()));
//
//    }


//        printf("Press [Enter] key to continue.\n");

//        while(getchar()!='\n'); // option TWO to clean stdin
//        getchar(); // wait for ENTER


//    }

//    double sum = std::accumulate(vectorInitialGuess.begin(), vectorInitialGuess.end(), 0.0);
//    double mean = sum / vectorInitialGuess.size();
//
//    std::vector<double> diff(vectorInitialGuess.size());
//    std::transform(vectorInitialGuess.begin(), vectorInitialGuess.end(), diff.begin(),
//                   std::bind2nd(std::minus<double>(), mean));
//    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
//    double stdev = std::sqrt(sq_sum / vectorInitialGuess.size());
//
//
//    std::cout << "Initial mean: " << mean << std::endl;
//    std::cout << "Initial stdev: " << stdev << std::endl;
//
//    sum = std::accumulate(vectorBestGuess.begin(), vectorBestGuess.end(), 0.0);
//    mean = sum / vectorBestGuess.size();
//
//    std::vector<double> diff2(vectorBestGuess.size());
//    std::transform(vectorBestGuess.begin(), vectorBestGuess.end(), diff2.begin(),
//                   std::bind2nd(std::minus<double>(), mean));
//     sq_sum = std::inner_product(diff2.begin(), diff2.end(), diff2.begin(), 0.0);
//     stdev = std::sqrt(sq_sum / vectorBestGuess.size());
//
//
//    std::cout << "Best mean: " << mean << std::endl;
//    std::cout << "Best stdev: " << stdev << std::endl;

    return (0);
}
