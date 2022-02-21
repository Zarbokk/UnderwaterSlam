/***************************************************************************
  **************************************************************************

  SOFT: SO(3) Fourier Transforms
  Version 2.0

  Copyright (c) 2003, 2004, 2007 Peter Kostelec, Dan Rockmore

  This file is part of SOFT.

  SOFT is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  SOFT is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  See the accompanying LICENSE file for details.

  ************************************************************************
  ************************************************************************/

/*
  to test the correlation routines

  - uses the Wigner-d symmetries
  - uses part of SpharmonicKit
  - STRICTLY double SAMPLES of signal and pattern files
  - [result] -> optional -> filename of all the correlation values
                (if you want all of them)
  - bw -> bw of spherical signals
  - isReal - whether data is strictly real (flag = 1), or interleaved ( 0 )

  example: test_soft_fftw_correlate2_wrap signalFile patternFile bw isReal

*/
//#include <stdio.h>
//#include <stdlib.h>
//#include <time.h>
//#include <math.h>
#include <chrono>
#include "fftw3.h"
#include "soft20/wrap_fftw.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

double thetaIncrement(double index, int bandwidth) {
    return M_PI * (2 * index + 1) / ((double) 4 * bandwidth);
}
double phiIncrement(double index, int bandwidth) {
    return M_PI * (2 * index + 1) / ((double) 4 * bandwidth);
}

int main(int argc,
         char **argv) {
    const int numberOfPoints = 128;
    const double fromTo = 30;
    double *voxelData1 = new double[numberOfPoints * numberOfPoints * numberOfPoints];
    double *voxelData2 = new double[numberOfPoints * numberOfPoints * numberOfPoints];


    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/after_voxel_second.pcd",
                         *pointCloudInputData1);



    //90 degree rotation
    Eigen::Matrix4d transformation90Degree;
    Eigen::AngleAxisd rotation_vector2(45.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformation90Degree.block<3, 3>(0, 0) = tmpMatrix3d;
    transformation90Degree(3, 3) = 1;
    //copy the rotated PCL from PCL1 to PCL2
    pcl::transformPointCloud(*pointCloudInputData1, *pointCloudInputData2, transformation90Degree);


    //fill with zeros
    for (int i = 0; i < numberOfPoints; i++) {
        for (int j = 0; j < numberOfPoints; j++) {
            for (int k = 0; k < numberOfPoints; k++) {
                voxelData1[i * (numberOfPoints * numberOfPoints) + j * (numberOfPoints) + k] = 0;
                voxelData2[i * (numberOfPoints * numberOfPoints) + j * (numberOfPoints) + k] = 0;
            }
        }
    }
//    voxelData1[2 * (numberOfPoints * numberOfPoints) + 3 * (numberOfPoints) + 5] = 0;
//    voxelData1[1 * (numberOfPoints * numberOfPoints) + 5 * (numberOfPoints) + 2] = 0;
    for (int i = 0; i < pointCloudInputData1->points.size(); i++) {
        double positionPointX = pointCloudInputData1->points[i].x;
        double positionPointY = pointCloudInputData1->points[i].y;
        double positionPointZ = pointCloudInputData1->points[i].z;
        int indexX = (int) std::round((positionPointX + fromTo) / (fromTo * 2) * numberOfPoints) - 1;
        int indexY = (int) std::round((positionPointY + fromTo) / (fromTo * 2) * numberOfPoints) - 1;
        int indexZ = (int) std::round((0 + fromTo) / (fromTo * 2) * numberOfPoints) - 1;//set to zero
//        std::cout << indexX << " , " << indexY << " , " <<indexZ << std::endl;
        voxelData1[indexY * (numberOfPoints * numberOfPoints) + indexX * (numberOfPoints) + indexZ] = 1;
    }

    for (int i = 0; i < pointCloudInputData2->points.size(); i++) {
        double positionPointX = pointCloudInputData2->points[i].x;
        double positionPointY = pointCloudInputData2->points[i].y;
        double positionPointZ = pointCloudInputData2->points[i].z;
        int indexX = (int) ((positionPointX + fromTo) / (fromTo * 2) * numberOfPoints);
        int indexY = (int) ((positionPointY + fromTo) / (fromTo * 2) * numberOfPoints);
        int indexZ = (int) ((0 + fromTo) / (fromTo * 2) * numberOfPoints);//set to zero
        voxelData2[indexY * (numberOfPoints * numberOfPoints) + indexX * (numberOfPoints) + indexZ] = 1;
    }


    //transform pointcloud to fourier space
    fftw_complex *inputSpacialData1, *inputSpacialData2;
    fftw_complex *outputSpacialData1, *outputSpacialData2;
    double *magnitude1, *magnitude2;

    magnitude1 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints * numberOfPoints);
    magnitude2 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints * numberOfPoints);

    inputSpacialData1 = (fftw_complex *) fftw_malloc(
            sizeof(fftw_complex) * numberOfPoints * numberOfPoints * numberOfPoints);
    inputSpacialData2 = (fftw_complex *) fftw_malloc(
            sizeof(fftw_complex) * numberOfPoints * numberOfPoints * numberOfPoints);
    outputSpacialData1 = (fftw_complex *) fftw_malloc(
            sizeof(fftw_complex) * numberOfPoints * numberOfPoints * numberOfPoints);
    outputSpacialData2 = (fftw_complex *) fftw_malloc(
            sizeof(fftw_complex) * numberOfPoints * numberOfPoints * numberOfPoints);




    //from voxel data to row major
    for (int i = 0; i < numberOfPoints; i++) {
        for (int j = 0; j < numberOfPoints; j++) {
            for (int k = 0; k < numberOfPoints; k++) {
                inputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][0] = voxelData1[k + numberOfPoints *
                                                                                                     (j +
                                                                                                      numberOfPoints *
                                                                                                      i)]; // real part
                inputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][1] = 0; // imaginary part
            }
        }
    }



//
//
    fftw_plan planToFourierVoxelOne, planToFourierVoxelTwo;
    planToFourierVoxelOne = fftw_plan_dft_3d(numberOfPoints, numberOfPoints, numberOfPoints, inputSpacialData1,
                                             outputSpacialData1, FFTW_FORWARD, FFTW_ESTIMATE);
    planToFourierVoxelTwo = fftw_plan_dft_3d(numberOfPoints, numberOfPoints, numberOfPoints, inputSpacialData2,
                                             outputSpacialData2, FFTW_FORWARD, FFTW_ESTIMATE);

//    planToFourier = fftw_plan_dft_r2c_3d(numberOfPoints, numberOfPoints, numberOfPoints, inputSpacialData1,
//                                     outputSpacialData1, FFTW_ESTIMATE);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    fftw_execute(planToFourierVoxelOne);
    fftw_execute(planToFourierVoxelTwo);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
              << "[ms]" << std::endl;

    std::ofstream myFile1, myFile2, myFile3;
    myFile1.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/realFFTW.csv");
    myFile2.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/imaginaryFFTW.csv");
    myFile3.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/voxelData.csv");
    for (int i = 0; i < numberOfPoints; i++) {
        for (int j = 0; j < numberOfPoints; j++) {
            for (int k = 0; k < numberOfPoints; k++) {
//                std::cout << outputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][0] << std::endl;
                myFile1 << outputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][0]; // real part
                myFile1 << "\n";
                myFile2 << outputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][1]; // imaginary part
                myFile2 << "\n";
                myFile3 << voxelData1[k + numberOfPoints * (j + numberOfPoints * i)]; // imaginary part
                myFile3 << "\n";
            }
        }
    }

    myFile1.close();
    myFile2.close();
    myFile3.close();


    double maximumScan1 = 0;
    double maximumScan2 = 0;
    //get magnitude and find maximum of each set
    for (int i = 0; i < numberOfPoints; i++) {
        for (int j = 0; j < numberOfPoints; j++) {
            for (int k = 0; k < numberOfPoints; k++) {
                magnitude1[k + numberOfPoints * (j + numberOfPoints * i)] = sqrt(
                        outputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][0] *
                        outputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][0] +
                        outputSpacialData1[k + numberOfPoints * (j + numberOfPoints * i)][1] *
                        outputSpacialData1[k + numberOfPoints *
                                               (j + numberOfPoints * i)][1]); // real part;
                if (maximumScan1 < magnitude1[k + numberOfPoints * (j + numberOfPoints * i)]) {
                    maximumScan1 = magnitude1[k + numberOfPoints * (j + numberOfPoints * i)];
                }
                magnitude2[k + numberOfPoints * (j + numberOfPoints * i)] = sqrt(
                        outputSpacialData2[k + numberOfPoints * (j + numberOfPoints * i)][0] *
                        outputSpacialData2[k + numberOfPoints * (j + numberOfPoints * i)][0] +
                        outputSpacialData2[k + numberOfPoints * (j + numberOfPoints * i)][1] *
                        outputSpacialData2[k + numberOfPoints *
                                               (j + numberOfPoints * i)][1]); // real part;
                if (maximumScan2 < magnitude2[k + numberOfPoints * (j + numberOfPoints * i)]) {
                    maximumScan2 = magnitude2[k + numberOfPoints * (j + numberOfPoints * i)];
                }

            }
        }
    }

    double globalMaximumMagnitude;
    if (maximumScan2 < maximumScan1) {
        globalMaximumMagnitude = maximumScan1;
    } else {
        globalMaximumMagnitude = maximumScan2;
    }

    //normalize
    for (int i = 0; i < numberOfPoints; i++) {
        for (int j = 0; j < numberOfPoints; j++) {
            for (int k = 0; k < numberOfPoints; k++) {
                magnitude1[k + numberOfPoints * (j + numberOfPoints * i)] =
                        magnitude1[k + numberOfPoints * (j + numberOfPoints * i)] / globalMaximumMagnitude;
                magnitude2[k + numberOfPoints * (j + numberOfPoints * i)] =
                        magnitude2[k + numberOfPoints * (j + numberOfPoints * i)] / globalMaximumMagnitude;
            }
        }
    }

    //resample magnitude from 3D to 2D(on sphere)
    double *resampledMagnitude1, *resampledMagnitude2;

    resampledMagnitude1 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
    resampledMagnitude2 = (double *) malloc(sizeof(double) * numberOfPoints * numberOfPoints);
    //initialize to zero
    for(int i = 0;i<numberOfPoints * numberOfPoints;i++){
        resampledMagnitude1[i]=0;
        resampledMagnitude2[i]=0;
    }

    int minRNumber = 4;
    int maxRNumber = numberOfPoints / 2 - 2;
    int bandwidth = numberOfPoints / 2;


    //double phiIncrement = M_PI / (double) bandwidth;//from 0 to 2*bandwidth-1
    // thetaIncrement defined over main function

    for (int r = minRNumber; r < maxRNumber; r++) {
        for (int j = 0; j < 2*bandwidth; j++) {
            for (int k = 0; k < 2*bandwidth; k++) {
                int xIndex = std::round((double)r*std::sin(thetaIncrement((double)j,bandwidth))*std::cos(phiIncrement((double)k,bandwidth))+bandwidth);
                int yIndex = std::round((double)r*std::sin(thetaIncrement((double)j,bandwidth))*std::sin(phiIncrement((double)k,bandwidth))+bandwidth);
                int zIndex = std::round((double)r*std::cos(thetaIncrement((double)j,bandwidth))+bandwidth);
                resampledMagnitude1[j+k*bandwidth*2] = resampledMagnitude1[j+k*bandwidth*2] + magnitude1[zIndex + numberOfPoints * (yIndex + numberOfPoints * xIndex)];
                resampledMagnitude2[j+k*bandwidth*2] = resampledMagnitude2[j+k*bandwidth*2] + magnitude2[zIndex + numberOfPoints * (yIndex + numberOfPoints * xIndex)];

            }
        }
    }





//    FILE *fp ;
//    int i ;
//    int n, bw, isReal ;
//    double *signal, *pattern ;
//    double alpha, beta, gamma ;
//
//    if (argc < 5 )
//    {
//        printf("test_soft_sym_correlate2_wrap signalFile patternFile bw isReal\n");
//        printf(" isReal = 1: signal and pattern strictly real (no interleaved)\n");
//        printf(" isReal = 0: signal and pattern complex (interleaved)\n");
//        exit(0) ;
//    }
//
//    bw = atoi( argv[3] );
//    n = 2 * bw ;
//
//    isReal = atoi( argv[4] );
//
//    /* allocate space to hold signal, pattern */
//    if ( isReal )
//    {
//        signal = (double *) malloc( sizeof(double) * (n * n) );
//        pattern = (double *) malloc( sizeof(double) * (n * n) );
//    }
//    else
//    {
//        signal = (double *) malloc( sizeof(double) * (2 * n * n) );
//        pattern = (double *) malloc( sizeof(double) * (2 * n * n) );
//    }
//
//    /****
//         At this point, check to see if all the memory has been
//         allocated. If it has not, there's no point in going further.
//    ****/
//
//    if ( (signal == NULL) || (pattern == NULL) )
//    {
//        perror("Error in allocating memory");
//        exit( 1 ) ;
//    }
//
//    printf("Reading in signal file\n");
//    /* read in SIGNAL samples */
//    fp = fopen(argv[1],"r");
//    if ( isReal )
//    {
//        for ( i = 0 ; i < n * n ; i ++ )
//        {
//            fscanf(fp,"%lf", signal + i);
//        }
//    }
//    else
//    {
//        for ( i = 0 ; i < 2 * n * n ; i ++ )
//        {
//            fscanf(fp,"%lf", signal + i);
//        }
//    }
//    fclose( fp );
//
//    printf("Reading in pattern file\n");
//    /* read in PATTERN samples */
//    fp = fopen(argv[2],"r");
//    if ( isReal )
//    {
//        for ( i = 0 ; i < n * n ; i ++ )
//        {
//            fscanf(fp,"%lf", pattern + i);
//        }
//    }
//    else
//    {
//        for ( i = 0 ; i < 2 * n * n ; i ++ )
//        {
//            fscanf(fp,"%lf", pattern + i);
//        }
//    }
//    fclose( fp );
//
//    /* now correlate */
//    softFFTWCor2( bw,
//                  signal,
//                  pattern,
//                  &alpha, &beta, &gamma,
//                  isReal) ;
//
//    /* print results */
//    printf("alpha = %f\nbeta = %f\ngamma = %f\n",
//           alpha, beta, gamma );
//
//    /* clean up */
//    free( pattern );
//    free( signal ) ;

    return 0;

}
