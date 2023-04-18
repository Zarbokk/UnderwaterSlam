//
// Created by tim-linux on 26.03.22.
//

//
// Created by jurobotics on 13.09.21.
//
// /home/tim-external/dataFolder/StPereDataset/lowNoise52/scanNumber_0/00_ForShow.jpg /home/tim-external/dataFolder/StPereDataset/lowNoise52/scanNumber_1/00_ForShow.jpg
// /home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_0/00_ForShow.jpg  /home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_1/00_ForShow.jpg
#include "generalHelpfulTools.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include "scanRegistrationClass.h"
#include "sofftCorrelationClass.h"

//#include "soft20/s2_cospmls.h"
//#include "soft20/s2_semi_memo.h"
//#include "soft20/makeweights.h"
//#include "soft20/so3_correlate_fftw.h"
//#include "soft20/soft_fftw.h"
//#include "fftw3.h"
#include "soft20/wrap_fftw.h"

Eigen::Quaterniond returnQuaternion(double z1, double y, double z2) {
    Eigen::AngleAxisd rotation_vectorz1(z1 / 64.0 * 2*3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd rotation_vectory(y / 64.0 * 3.14159, Eigen::Vector3d(0, 1, 0));
    Eigen::AngleAxisd rotation_vectorz2(z2 / 64.0 * 2*3.14159, Eigen::Vector3d(0, 0, 1));


    Eigen::Matrix3d tmpMatrix3d = rotation_vectorz1.toRotationMatrix() * rotation_vectory.toRotationMatrix() *
                                  rotation_vectorz2.toRotationMatrix();
    return Eigen::Quaterniond(tmpMatrix3d);
};

int main(int argc, char **argv) {

    Eigen::Quaterniond quat1 = returnQuaternion(31, 35, 45);
    std::cout << quat1.x() <<" " << quat1.y() <<" "<< quat1.z() <<" "<< quat1.w() << std::endl;
    quat1 = returnQuaternion(1, 35, 1);
    std::cout << quat1.x() <<" " << quat1.y() <<" "<< quat1.z() <<" "<< quat1.w() << std::endl;
    quat1 = returnQuaternion(63, 35, 1);
    std::cout << quat1.x() <<" " << quat1.y() <<" "<< quat1.z() <<" "<< quat1.w() << std::endl;

    return (0);
}
