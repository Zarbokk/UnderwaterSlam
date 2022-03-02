//
// Created by tim-linux on 01.03.22.
//

#ifndef UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H
#define UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H

#include "sofftCorrelationClass.h"
#include "PeakFinder.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

struct angleAndCorrelation {
    double angle, correlation;
};

class softDescriptorRegistration {
public:
    softDescriptorRegistration(int N,int bwOut,int bwIn,int degLim) : sofftCorrelationObject(N,bwOut, bwIn, degLim) {
        this->N = N;
        this->bwOut = bwOut;
        this->bwIn = bwIn;
        this->degLim = degLim;
        this->resultingCorrelationDouble = (double *) malloc(sizeof(double) * N * N * N);
        this->resultingCorrelationComplex = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));
        this->resultingPhaseDiff = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->resultingShiftPeaks = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);

        this->magnitude1Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2Shifted = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData1 = (double *) malloc(sizeof(double) * N * N * N);
        this->voxelData2 = (double *) malloc(sizeof(double) * N * N * N);
        this->spectrum1 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->spectrum2 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        this->phase1 = (double *) malloc(sizeof(double) * N * N * N);
        this->phase2 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude1 = (double *) malloc(sizeof(double) * N * N * N);
        this->magnitude2 = (double *) malloc(sizeof(double) * N * N * N);
        resampledMagnitude1 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitude2 = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitude1TMP = (double *) malloc(sizeof(double) * N * N);
        resampledMagnitude2TMP = (double *) malloc(sizeof(double) * N * N);

        planToFourierVoxel = fftw_plan_dft_3d(N, N, N, resultingPhaseDiff,
                                              resultingShiftPeaks, FFTW_BACKWARD, FFTW_ESTIMATE);
//        planToFourierVoxel = fftw_plan_dft_2d(N, N, resultingPhaseDiff,
//                                              resultingShiftPeaks, FFTW_BACKWARD, FFTW_ESTIMATE);
        correlation2DResult = (double *) malloc(sizeof(double) * N * N);
    }

    Eigen::Matrix4d registrationOfTwoPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2,
                                         const double cellSize);//gives TFMatrix from 2 to 1
    double getSpectrumFromPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData, double voxelData[],
                              fftw_complex spectrum[], double magnitude[], double phase[], double fromTo, int N);

private://here everything is created. malloc is done in the constructor




    int N;//describes the size of the overall voxel system
    int bwOut,bwIn,degLim;
    double *voxelData1;
    double *voxelData2;
    fftw_complex *spectrum1;
    fftw_complex *spectrum2;
    double *magnitude1, *phase1;
    double *magnitude2, *phase2;
    double *magnitude1Shifted, *magnitude2Shifted;
    double *resampledMagnitude1, *resampledMagnitude2, *resampledMagnitude1TMP, *resampledMagnitude2TMP;
    sofftCorrelationClass sofftCorrelationObject;
    fftw_complex *resultingCorrelationComplex;
    fftw_complex *resultingPhaseDiff, *resultingShiftPeaks;
    double *resultingCorrelationDouble;
    fftw_plan planToFourierVoxel;
    double *correlation2DResult;

};


#endif //UNDERWATERSLAM_SOFTDESCRIPTORREGISTRATION_H
