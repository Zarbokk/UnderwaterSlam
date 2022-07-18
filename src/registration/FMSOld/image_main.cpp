#include "image_registration.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    cv::Mat im0, im1;

//    if (argc < 3)
//    {
//        std::cout << "specify two image files" << std::endl;
//        return 1;
//    }
    im0 = cv::imread("/home/tim-external/Documents/imreg_fmt/firstImage.jpg", cv::IMREAD_COLOR);
    im1 = cv::imread("/home/tim-external/Documents/imreg_fmt/secondImage.jpg", cv::IMREAD_COLOR);

    cv::imshow("im0", im0);
    cv::imshow("im1", im1);

    ImageRegistration image_registration(im0);

    // x, y, rotation, scale
    std::vector<double> transform_params(4, 0.0);
    cv::Mat registered_image;
    image_registration.registerImage(im1, registered_image, transform_params, true);


    std::cout << "x: " << transform_params[0] << ", y: "
              << transform_params[1] << ", rotation: " << transform_params[2]
              << ", scale: " << transform_params[3] << std::endl;

    cv::Mat overlay_image;
    cv::addWeighted(image_registration.getCurrentImage(), 0.5, registered_image, 0.5, 0.0, overlay_image);
    cv::imwrite("/home/tim-external/Documents/imreg_fmt/result.jpg", overlay_image);
    cv::imshow("overlay_image", overlay_image);

    cv::waitKey(0);
    return 0;
}
