//
// Created by tim-external on 02.05.23.
//
// scale invariant feature transform SIFT
// speeded up robust features SURF
// ORB
//  feature detection -> Horn -> RANSAC -> Horn

#include "generalHelpfulTools.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
//#include "scanRegistrationClass.h"
#include "fstream"


int main(int argc, char **argv) {



    double customRotation = +0;

    std::string current_exec_name = argv[0]; // Name of the current exec program
    std::vector<std::string> all_args;

    if (argc > 0) {
        //std::cout << "temp1" << std::endl;
        all_args.assign(argv + 1, argv + argc);
        //std::cout << "12"<< all_args[1]<<std::endl;
    } else {
        std::cout << "no arguments given" << std::endl;
        exit(-1);
    }



    int numberOfPoints = stoi(all_args[2]);

    double *voxelData1;
    voxelData1 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);

    std::ifstream voxelDataFile(all_args[0]);
    for (int i = 0; i < numberOfPoints; i++) {
        std::vector<std::string> voxelDataVector = generalHelpfulTools::getNextLineAndSplitIntoTokens(voxelDataFile);
        for (int j = 0; j < numberOfPoints; j++) {
            voxelData1[i + numberOfPoints * j] = std::stod(voxelDataVector[j]);
        }
    }

    double *voxelData2;
    voxelData2 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
    std::ifstream voxelDataShiftedFile(all_args[1]);
    for (int i = 0; i < numberOfPoints; i++) {
        std::vector<std::string> voxelDataShiftedVector = generalHelpfulTools::getNextLineAndSplitIntoTokens(
                voxelDataShiftedFile);
        for (int j = 0; j < numberOfPoints; j++) {
            voxelData2[i + numberOfPoints * j] = std::stod(voxelDataShiftedVector[j]);
            //                std::cout <<voxelDataShifted[i + numberOfPoints * j] << std::endl;
        }
    }


    cv::Mat magTMP1(numberOfPoints, numberOfPoints, CV_64F, voxelData1);
    cv::Mat magTMP2(numberOfPoints, numberOfPoints, CV_64F, voxelData2);

    cv::Mat convertedMat1;
    cv::Mat convertedMat2;

    magTMP1.convertTo(convertedMat1, CV_8U);
    magTMP2.convertTo(convertedMat2, CV_8U);

    cv::cvtColor(convertedMat1,convertedMat1,cv::COLOR_GRAY2BGR);
    cv::cvtColor(convertedMat2,convertedMat2,cv::COLOR_GRAY2BGR);


    cv::Ptr<cv::AKAZE> D = cv::AKAZE::create();

    std::vector<cv::KeyPoint> kpts1, kpts2, matched1, matched2;
    cv::Mat desc1, desc2;

    D->detectAndCompute(convertedMat1, cv::noArray(), kpts1, desc1);
    D->detectAndCompute(convertedMat2, cv::noArray(), kpts2, desc2);

    cv::Mat J; //to save temporal images
    drawKeypoints(convertedMat1, kpts1, J);
    imshow("Keypoints 1", J);
//    imwrite("Keypoints1.jpg", J);
    drawKeypoints(convertedMat2, kpts2, J);
    imshow("Keypoints 2", J);
//    imwrite("Keypoints2.jpg", J);
    int k = cv::waitKey(0);






    return (0);
}
