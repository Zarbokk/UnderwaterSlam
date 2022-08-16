//
// Created by tim-linux on 01.03.22.
//

#ifndef UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H
#define UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H

#include "sofftCorrelationClass.h"
#include "PeakFinder.h"
#include "generalHelpfulTools.h"
//#include "slamToolsRos.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_2.h>
#include <CGAL/Random.h>


struct angleAndCorrelation {
    double angle, correlation;
};

class softDescriptorRegistration {
public:
    softDescriptorRegistration(int N, int bwOut, int bwIn, int degLim) : sofftCorrelationObject(N, bwOut, bwIn,
                                                                                                degLim) {
        this->N = N;
        this->bwOut = bwOut;
        this->bwIn = bwIn;
        this->degLim = degLim;
        this->resultingCorrelationDouble = (double *) malloc(sizeof(double) * N * N * N);
        this->resultingCorrelationComplex = fftw_alloc_complex(8 * bwOut * bwOut * bwOut);
//        (fftw_complex *) fftw_malloc(
//                sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));
        this->resultingPhaseDiff2D = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N);
        this->resultingShiftPeaks2D = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N);

        this->magnitude1Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData1 = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData2 = (double *) malloc(sizeof(double) * N * N * N);
//        this->spectrum1 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
//        this->spectrum2 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->spectrumOut = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->phase1 = (double *) malloc(sizeof(double) * N * N * N);
        this->phase2 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude1 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2 = (double *) malloc(sizeof(double) * N * N * N);
        resampledMagnitudeSO3_1 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_2 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_1TMP = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitudeSO3_2TMP = (double *) malloc(sizeof(double) * N * N);
        inputSpacialData = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);

//        planToFourierVoxel = fftw_plan_dft_3d(N, N, N, resultingPhaseDiff2D,
//                                              resultingShiftPeaks2D, FFTW_BACKWARD, FFTW_ESTIMATE);
        planFourierToVoxel2D = fftw_plan_dft_2d(N, N, resultingPhaseDiff2D,
                                                resultingShiftPeaks2D, FFTW_BACKWARD, FFTW_ESTIMATE);
//        correlation2DResult = (double *) malloc(sizeof(double) * N * N);


        planVoxelToFourier3D = fftw_plan_dft_3d(N, N, N, inputSpacialData,
                                                spectrumOut, FFTW_FORWARD, FFTW_ESTIMATE);
        planVoxelToFourier2D = fftw_plan_dft_2d(N, N, inputSpacialData,
                                                spectrumOut, FFTW_FORWARD, FFTW_ESTIMATE);
    }

    ~softDescriptorRegistration() {
        sofftCorrelationObject.~sofftCorrelationClass();


//        free(this->resultingCorrelationDouble);
//        fftw_free(this->resultingCorrelationComplex);
//        fftw_free(this->resultingPhaseDiff2D );
//        fftw_free(this->resultingShiftPeaks2D);
//        fftw_free(this->magnitude1Shifted );
//        fftw_free(this->magnitude2Shifted );
//        fftw_free(this->voxelData1 );
//        fftw_free(this->voxelData2 );
//        fftw_free(this->spectrumOut );
//        fftw_free(this->phase1);
//        fftw_free(this->phase2);
//        fftw_free(this->magnitude1 );
//        fftw_free(this->magnitude2 );
//        fftw_free(resampledMagnitudeSO3_1 );
//        fftw_free(resampledMagnitudeSO3_2);
//        fftw_free(resampledMagnitudeSO3_1TMP);
//        fftw_free(resampledMagnitudeSO3_2TMP );
//        fftw_free(inputSpacialData);
//        fftw_destroy_plan(planFourierToVoxel2D);
//        fftw_destroy_plan(planVoxelToFourier3D);
//        fftw_destroy_plan(planVoxelToFourier2D);


    }

    Eigen::Matrix4d registrationOfTwoPCL2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData1,
                                           pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData2, double &fitnessX,
                                           double &fitnessY, double goodGuessAlpha = -100,
                                           bool debug = false);//gives TFMatrix from 1 to 2


    Eigen::Matrix4d registrationOfTwoPCL2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData1,
                                           pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData2,
                                           double &fitnessX, double &fitnessY,
                                           Eigen::Matrix4d initialGuessTransformation,
                                           bool useInitialGuess,
                                           bool debug);

    //-100 only for "no good guess given"
    //initial guess has to be very good, else dont use it.
    double getSpectrumFromPCL3D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData, double voxelData[],
                                double magnitude[], double phase[], double fromTo, int N);

    double getSpectrumFromPCL2D(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData, double voxelData[],
                                double magnitude[], double phase[], double fromTo, bool gaussianBlur = false);

    double
    getSpectrumFromVoxelData2D(double voxelData[], double magnitude[], double phase[], bool gaussianBlur = false);

    void PCL2Voxel(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData, double voxelData[], double fromTo);

    double movePCLtoMiddle(pcl::PointCloud<pcl::PointXYZ> &pointCloudInputData, Eigen::Matrix4d &transformationPCL);

//    Eigen::Matrix4d
//    registrationOfTwoVoxel2D(double voxelData1[], double voxelData2[], double &fitnessX, double &fitnessY,
//                             double goodGuessAlpha, bool debug);


    double
    sofftRegistrationVoxel2DRotationOnly(double voxelData1Input[], double voxelData2Input[], double goodGuessAlpha,
                                         bool debug = false);

    std::vector<double>
    sofftRegistrationVoxel2DListOfPossibleRotations(double voxelData1Input[], double voxelData2Input[],
                                                    bool debug = false);

    Eigen::Vector2d sofftRegistrationVoxel2DTranslation(double voxelData1Input[],
                                                        double voxelData2Input[],
                                                        double &fitnessX, double &fitnessY, double cellSize,
                                                        Eigen::Vector3d initialGuess, bool useInitialGuess,
                                                        double &heightMaximumPeak, bool debug = false);

    Eigen::Matrix4d registrationOfTwoVoxelsSOFFTFast(double voxelData1Input[],
                                                     double voxelData2Input[],
                                                     Eigen::Matrix4d &initialGuess,
                                                     bool useInitialAngle, bool useInitialTranslation,
                                                     double cellSize,
                                                     bool useGauss,
                                                     bool debug = false);

private://here everything is created. malloc is done in the constructor




    int N;//describes the size of the overall voxel system
    int bwOut, bwIn, degLim;
    double *voxelData1;
    double *voxelData2;
//    fftw_complex *spectrum1;
//    fftw_complex *spectrum2;
    fftw_complex *spectrumOut;
    double *magnitude1;
    double *phase1;
    double *magnitude2;
    double *phase2;
    double *magnitude1Shifted;
    double *magnitude2Shifted;
    double *resampledMagnitudeSO3_1;
    double *resampledMagnitudeSO3_2;
    double *resampledMagnitudeSO3_1TMP;
    double *resampledMagnitudeSO3_2TMP;
    sofftCorrelationClass sofftCorrelationObject;
    fftw_complex *resultingCorrelationComplex;
    fftw_complex *resultingPhaseDiff2D;
    fftw_complex *resultingShiftPeaks2D;
    double *resultingCorrelationDouble;
//    fftw_plan planToFourierVoxel;
//    double *correlation2DResult;
    fftw_complex *inputSpacialData;
    fftw_plan planVoxelToFourier3D;
    fftw_plan planVoxelToFourier2D;
    fftw_plan planFourierToVoxel2D;
};


#endif //UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H
