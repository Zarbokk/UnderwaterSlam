//
// Created by tim-linux on 26.03.22.
//

//
// Created by jurobotics on 13.09.21.
//


#include "generalHelpfulTools.h"
#include "slamToolsRos.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>




void convertMatToDoubleArray(cv::Mat inputImg, double voxelData[]){

    std::vector<uchar> array;
    if (inputImg.isContinuous()) {
        // array.assign(mat.datastart, mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
        array.assign(inputImg.data, inputImg.data + inputImg.total()*inputImg.channels());
    } else {
        for (int i = 0; i < inputImg.rows; ++i) {
            array.insert(array.end(), inputImg.ptr<uchar>(i), inputImg.ptr<uchar>(i)+inputImg.cols*inputImg.channels());
        }
    }

    for(int i = 0 ; i<array.size();i++){
        voxelData[i]=array[i];
//        std::cout << voxelData[i] <<std::endl;
//        std::cout << "i: "<< i <<std::endl;
    }

}


int main(int argc, char **argv) {




//    std::string current_exec_name = argv[0]; // Name of the current exec program
//    std::vector<std::string> all_args;
//
//    if (argc > 0) {
//        //std::cout << "temp1" << std::endl;
//        all_args.assign(argv + 1, argv + argc);
//        //std::cout << "12"<< all_args[1]<<std::endl;
//    }else{
//        std::cout << "no arguments given" << std::endl;
//        exit(-1);
//    }

    cv::Mat img1 = cv::imread("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/FMT/lena_cropped.bmp", cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/FMT/lena_cropped_rotated.bmp", cv::IMREAD_GRAYSCALE);
//    cv::Mat img1 = cv::imread("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/FMT/firstImage.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat img2 = cv::imread("/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/FMT/secondImage.jpg", cv::IMREAD_GRAYSCALE);


    double *voxelData1;
    double *voxelData2;
    voxelData1 = (double *) malloc(sizeof(double) * img1.rows * img1.rows);
    voxelData2 = (double *) malloc(sizeof(double) * img2.rows * img2.rows);

    convertMatToDoubleArray(img1,voxelData1);
    convertMatToDoubleArray(img2,voxelData2);

    scanRegistrationClass scanRegistrationObject(img1.rows, img1.rows / 2, img1.rows / 2, img1.rows / 2 - 1);


    double rotationTest = scanRegistrationObject.sofftRegistrationVoxel2DRotationOnly(voxelData1, voxelData2, 4.5,true);

    double fitnessX;
    double fitnessY;
    Eigen::Vector3d initialGuess(0,0,0);
    Eigen::Vector2d resultTranslation = scanRegistrationObject.sofftRegistrationVoxel2DTranslation(voxelData1, voxelData2, fitnessX,
                                        fitnessY, 1.0,
                                        initialGuess, false, true);



//    Eigen::Matrix4d estimatedTransformation = scanRegistrationObject.sofftRegistration(*scan1,*scan2,fitnessX,fitnessY,-100,true);


    return (0);
}
