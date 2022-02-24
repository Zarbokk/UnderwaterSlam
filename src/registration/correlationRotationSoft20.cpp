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
//#include "soft20/wrap_fftw.h"
#include "soft20/makeweights.h"
#include "soft20/so3_correlate_fftw.h"
#include "soft20/soft_fftw.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "soft20/s2_cospmls.h"
#include "soft20/s2_semi_memo.h"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )

double thetaIncrement(double index, int bandwidth) {
    return M_PI * (2 * index + 1) / (4.0 * bandwidth);
}

double phiIncrement(double index, int bandwidth) {
    return M_PI * index / bandwidth;
}

double
getSpectrmFromPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData, double voxelData[], fftw_complex spectrum[],
                  double magnitude[], double phase[], double fromTo, int N) {


    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                voxelData[k + N * (j + N * i)] = 0;
            }
        }
    }

    for (int i = 0; i < pointCloudInputData->points.size(); i++) {
        double positionPointX = pointCloudInputData->points[i].x;
        double positionPointY = pointCloudInputData->points[i].y;
        double positionPointZ = pointCloudInputData->points[i].z;
        int indexX = (int) std::round((positionPointX + fromTo) / (fromTo * 2) * N) - 1;
        int indexY = (int) std::round((positionPointY + fromTo) / (fromTo * 2) * N) - 1;
        int indexZ = (int) std::round((positionPointZ + fromTo) / (fromTo * 2) * N) - 1;//set to zero
        voxelData[indexZ + N * (indexX + N * indexY)] = 1;
    }

    fftw_complex *inputSpacialData;
    inputSpacialData = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);

    //from voxel data to row major
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                inputSpacialData[k + N * (j + N * i)][0] = voxelData[k + N * (j + N * i)]; // real part
                inputSpacialData[k + N * (j + N * i)][1] = 0; // imaginary part
            }
        }
    }

    fftw_plan planToFourierVoxel;
    planToFourierVoxel = fftw_plan_dft_3d(N, N, N, inputSpacialData,
                                          spectrum, FFTW_FORWARD, FFTW_ESTIMATE);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    fftw_execute(planToFourierVoxel);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
              << "[ms]" << std::endl;


    //calc magnitude and phase
    double maximumMagnitude = 0;

    //get magnitude and find maximum
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                magnitude[k + N * (j + N * i)] = sqrt(
                        spectrum[k + N * (j + N * i)][0] *
                        spectrum[k + N * (j + N * i)][0] +
                        spectrum[k + N * (j + N * i)][1] *
                        spectrum[k + N *
                                     (j + N * i)][1]); // real part;
                if (maximumMagnitude < magnitude[k + N * (j + N * i)]) {
                    maximumMagnitude = magnitude[k + N * (j + N * i)];
                }

                phase[k + N * (j + N * i)] = atan2(spectrum[k + N * (j + N * i)][1], spectrum[k + N * (j + N * i)][0]);
            }
        }
    }

    free(inputSpacialData);

    return maximumMagnitude;
}


int main(int argc,
         char **argv) {
    int N = 128;
    const double fromTo = 30;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2(new pcl::PointCloud<pcl::PointXYZ>);


    double *voxelData1;
    double *voxelData2;
    fftw_complex *spectrum1;
    fftw_complex *spectrum2;
    double *magnitude1, *phase1;
    double *magnitude2, *phase2;
    double *magnitude1Shifted, *magnitude2Shifted;

    magnitude1Shifted = (double *) malloc(sizeof(double) * N * N * N);
    magnitude2Shifted = (double *) malloc(sizeof(double) * N * N * N);
    voxelData1 = (double *) malloc(sizeof(double) * N * N * N);
    voxelData2 = (double *) malloc(sizeof(double) * N * N * N);
    spectrum1 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
    spectrum2 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
    phase1 = (double *) malloc(sizeof(double) * N * N * N);
    phase2 = (double *) malloc(sizeof(double) * N * N * N);
    magnitude1 = (double *) malloc(sizeof(double) * N * N * N);
    magnitude2 = (double *) malloc(sizeof(double) * N * N * N);

    pcl::io::loadPCDFile("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/after_voxel_second.pcd",
                         *pointCloudInputData1);

    //90 degree rotation
    Eigen::Matrix4d transformationPCL;
    //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd rotation_vector2(0.4, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationPCL.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationPCL(0, 3) = -5;
    transformationPCL(1, 3) = 0;
    transformationPCL(2, 3) = 0;
    transformationPCL(3, 3) = 1;
    //copy the rotated PCL from PCL1 to PCL2
    pcl::transformPointCloud(*pointCloudInputData1, *pointCloudInputData2, transformationPCL);

    double maximumScan1 = getSpectrmFromPCL(pointCloudInputData1, voxelData1, spectrum1, magnitude1, phase1, fromTo,
                                            N);
    double maximumScan2 = getSpectrmFromPCL(pointCloudInputData2, voxelData2, spectrum2, magnitude2, phase2, fromTo,
                                            N);


    std::ofstream myFile1, myFile2, myFile3,myFile4,myFile5,myFile6;
    myFile1.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/magnitudeFFTW1.csv");
    myFile2.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/phaseFFTW1.csv");
    myFile3.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW1.csv");
    myFile4.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/magnitudeFFTW2.csv");
    myFile5.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/phaseFFTW2.csv");
    myFile6.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/voxelDataFFTW2.csv");
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
//                std::cout << outputSpacialData1[k + N * (j + N * i)][0] << std::endl;
                myFile1 << magnitude1[k + N * (j + N * i)]; // real part
                myFile1 << "\n";
                myFile2 << phase1[k + N * (j + N * i)]; // imaginary part
                myFile2 << "\n";
                myFile3 << voxelData1[k + N * (j + N * i)]; // imaginary part
                myFile3 << "\n";
                myFile4 << magnitude2[k + N * (j + N * i)]; // real part
                myFile4 << "\n";
                myFile5 << phase2[k + N * (j + N * i)]; // imaginary part
                myFile5 << "\n";
                myFile6 << voxelData2[k + N * (j + N * i)]; // imaginary part
                myFile6 << "\n";
            }
        }
    }

    myFile1.close();
    myFile2.close();
    myFile3.close();
    myFile4.close();
    myFile5.close();
    myFile6.close();

    double globalMaximumMagnitude;
    if (maximumScan2 < maximumScan1) {
        globalMaximumMagnitude = maximumScan1;
    } else {
        globalMaximumMagnitude = maximumScan2;
    }




    //normalize and shift
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            for (int k = 0; k < N; k++) {
                int indexX = (N / 2 + i) % N;
                int indexY = (N / 2 + j) % N;
                int indexZ = (N / 2 + k) % N;

                magnitude1Shifted[indexZ + N * (indexY + N * indexX)] =
                        magnitude1[k + N * (j + N * i)] / globalMaximumMagnitude;
                magnitude2Shifted[indexZ + N * (indexY + N * indexX)] =
                        magnitude2[k + N * (j + N * i)] / globalMaximumMagnitude;
            }
        }
    }

    //resample magnitude from 3D to 2D(on sphere)
    double *resampledMagnitude1, *resampledMagnitude2,*resampledMagnitude1TMP,*resampledMagnitude2TMP;

    resampledMagnitude1 = (double *) malloc(sizeof(double) * N * N);
    resampledMagnitude2 = (double *) malloc(sizeof(double) * N * N);
    resampledMagnitude1TMP = (double *) malloc(sizeof(double) * N * N);
    resampledMagnitude2TMP = (double *) malloc(sizeof(double) * N * N);
    //initialize to zero
    for (int i = 0; i < N * N; i++) {
        resampledMagnitude1[i] = 0;
        resampledMagnitude2[i] = 0;
        resampledMagnitude1TMP[i] = 0;
        resampledMagnitude2TMP[i] = 0;
    }

    int minRNumber = 4;
    int maxRNumber = N / 2 - 2;
    int bandwidth = N / 2;

    //double phiIncrement = M_PI / (double) bandwidth;//from 0 to 2*bandwidth-1
    // thetaIncrement defined over main function

    for (int r = minRNumber; r < maxRNumber; r++) {
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                int xIndex = std::round((double) r * std::sin(thetaIncrement((double) j + 1, bandwidth)) *
                                        std::cos(phiIncrement((double) k + 1, bandwidth)) + bandwidth) - 1;
                int yIndex = std::round((double) r * std::sin(thetaIncrement((double) j + 1, bandwidth)) *
                                        std::sin(phiIncrement((double) k + 1, bandwidth)) + bandwidth) - 1;
                int zIndex =
                        std::round((double) r * std::cos(thetaIncrement((double) j + 1, bandwidth)) + bandwidth) - 1;
                resampledMagnitude1TMP[k + j * bandwidth * 2] = 255*magnitude1Shifted[zIndex + N * (yIndex + N * xIndex)];
                resampledMagnitude2TMP[k + j * bandwidth * 2] = 255*magnitude2Shifted[zIndex + N * (yIndex + N * xIndex)];
            }
        }
        cv::Mat magTMP1(N, N, CV_64FC1, resampledMagnitude1TMP);
        cv::Mat magTMP2(N, N, CV_64FC1, resampledMagnitude2TMP);
        magTMP1.convertTo(magTMP1,CV_8UC1);
        magTMP2.convertTo(magTMP2,CV_8UC1);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(magTMP1, magTMP1);
        clahe->apply(magTMP2, magTMP2);
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                resampledMagnitude1[k + j * bandwidth * 2] = resampledMagnitude1[k + j * bandwidth * 2]+((double)magTMP1.data[k + j * bandwidth * 2])/255.0;
                resampledMagnitude2[k + j * bandwidth * 2] = resampledMagnitude2[k + j * bandwidth * 2]+((double)magTMP2.data[k + j * bandwidth * 2])/255.0;
            }
        }

//        cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
//        cv::imshow("test", magTMP1);
//        cv::waitKey(0);
//        cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
//        cv::imshow("test", magTMP1);
//        cv::waitKey(0);
    }



    std::ofstream myFile7, myFile8;
    myFile7.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resampledVoxel1.csv");
    myFile8.open("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resampledVoxel2.csv");

    for (int j = 0; j < N; j++) {
        for (int k = 0; k < N; k++) {
            myFile7 << resampledMagnitude1[k + j * bandwidth * 2]; // real part
            myFile7 << "\n";
            myFile8 << resampledMagnitude2[k + j * bandwidth * 2]; // real part
            myFile8 << "\n";
        }
    }
    myFile7.close();
    myFile8.close();
/////////////////      OLD ONE   //////////////////////////////////////
//    int bw, isReal;
//    double alpha, beta, gamma;
//
//    bw = N / 2;//atoi( argv[3] );
//
//
//    isReal = 1;
//
//    /* now correlate */
//    softFFTWCor2(bw,
//                 resampledMagnitude1,
//                 resampledMagnitude2,
//                 &alpha, &beta, &gamma,
//                 isReal);
//
//    /* print results */
//    printf("alpha = %f\nbeta = %f\ngamma = %f\N",
//           alpha, beta, gamma);




    //FILE *fp ;
    int i ;
    int bwIn, bwOut, degLim ;

    fftw_complex *workspace1, *workspace2  ;
    double *workspace3 ;
    double *sigR, *sigI ;
    double *sigCoefR, *sigCoefI ;
    double *patCoefR, *patCoefI ;
    fftw_complex *so3Sig, *so3Coef ;
    fftw_plan p1 ;
    int na[2], inembed[2], onembed[2] ;
    int rank, howmany, istride, idist, ostride, odist ;
    int tmp, maxloc, ii, jj, kk ;
    double maxval, tmpval ;
    double *weights ;
    double *seminaive_naive_tablespace  ;
    double **seminaive_naive_table ;
    fftw_plan dctPlan, fftPlan ;
    int howmany_rank ;
    fftw_iodim dims[1], howmany_dims[1];


    bwIn = N/2;
    bwOut = N/2;
    degLim = bwOut-1;

    sigR = (double *) calloc(N * N, sizeof(double) );
    sigI = (double *) calloc(N * N, sizeof(double) );
    so3Sig = (fftw_complex *) fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
    workspace1 = (fftw_complex *) fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
    workspace2 = (fftw_complex *) fftw_malloc( sizeof(fftw_complex) * ((14*bwIn*bwIn) + (48 * bwIn)));
    workspace3 = (double *) malloc( sizeof(double) * (12 * N + N * bwIn));
    sigCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
    sigCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
    patCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
    patCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
    so3Coef = (fftw_complex *) fftw_malloc( sizeof(fftw_complex) * ((4*bwOut*bwOut*bwOut-bwOut)/3) ) ;


    seminaive_naive_tablespace =
            (double *) malloc(sizeof(double) *
                              (Reduced_Naive_TableSize(bwIn,bwIn) +
                               Reduced_SpharmonicTableSize(bwIn,bwIn)));

    weights = (double *) malloc(sizeof(double) * (4*bwIn));

    /****
         At this point, check to see if all the memory has been
         allocated. If it has not, there's no point in going further.
    ****/

    if ( (seminaive_naive_tablespace == NULL) || (weights == NULL) ||
         (sigR == NULL) || (sigI == NULL) ||
         (so3Coef == NULL) ||
         (workspace1 == NULL) || (workspace2 == NULL) ||
         (workspace3 == NULL) ||
         (sigCoefR == NULL) || (sigCoefI == NULL) ||
         (patCoefR == NULL) || (patCoefI == NULL) ||
         (so3Sig == NULL) )
    {
        perror("Error in allocating memory");
        exit( 1 ) ;
    }

    /* create fftw plans for the S^2 transforms */
    /* first for the dct */
    dctPlan = fftw_plan_r2r_1d( 2*bwIn, weights, workspace3,
                                FFTW_REDFT10, FFTW_ESTIMATE ) ;

    /* now for the fft */
    /*
       IMPORTANT NOTE!!! READ THIS!!!

       Now to make the fft plans.

       Please note that the planning-rigor flag *must be* FFTW_ESTIMATE!
       Why? Well, to try to keep things simple. I am using some of the
       pointers to arrays in rotateFct's arguments in the fftw-planning
       routines. If the planning-rigor is *not* FFTW_ESTIMATE, then
       the arrays will be written over during the planning stage.

       Therefore, unless you are really really sure you know what
       you're doing, keep the rigor as FFTW_ESTIMATE !!!
    */

    /*
      fftw "preamble" ;
      note  that this places in the transposed array
    */

    rank = 1 ;
    dims[0].n = 2*bwIn ;
    dims[0].is = 1 ;
    dims[0].os = 2*bwIn ;
    howmany_rank = 1 ;
    howmany_dims[0].n = 2*bwIn ;
    howmany_dims[0].is = 2*bwIn ;
    howmany_dims[0].os = 1 ;

    fftPlan = fftw_plan_guru_split_dft( rank, dims,
                                        howmany_rank, howmany_dims,
                                        sigR, sigI,
                                        (double *) workspace2,
                                        (double *) workspace2 + (N * N),
                                        FFTW_ESTIMATE );

    /* create plan for inverse SO(3) transform */
    N = 2 * bwOut ;
    howmany = N * N ;
    idist = N ;
    odist = N ;
    rank = 2 ;
    inembed[0] = N ;
    inembed[1] = N * N ;
    onembed[0] = N ;
    onembed[1] = N * N ;
    istride = 1 ;
    ostride = 1 ;
    na[0] = 1 ;
    na[1] = N ;

    p1 = fftw_plan_many_dft( rank, na, howmany,
                             workspace1, inembed,
                             istride, idist,
                             so3Sig, onembed,
                             ostride, odist,
                             FFTW_FORWARD, FFTW_ESTIMATE );


    fprintf(stdout,"Generating seminaive_naive tables...\n");
    seminaive_naive_table = SemiNaive_Naive_Pml_Table(bwIn, bwIn,
                                                      seminaive_naive_tablespace,
                                                      (double *) workspace2);


    /* make quadrature weights for the S^2 transform */
    makeweights( bwIn, weights ) ;

    N = 2 * bwIn ;
    /* read in SIGNAL samples */
    /* first the signal */
    //fp = fopen(argv[1],"r");
    for ( i = 0 ; i < N * N ; i ++ )
    {
        sigR[i] = resampledMagnitude1[i];
        sigI[i] = 0;
    }
    FST_semi_memo( sigR, sigI,
                   sigCoefR, sigCoefI,
                   bwIn, seminaive_naive_table,
                   (double *) workspace2, 0, bwIn,
                   &dctPlan, &fftPlan,
                   weights );

    /* read in SIGNAL samples */
    /* first the signal */
    for ( i = 0 ; i < N * N ; i ++ )
    {
        sigR[i] = resampledMagnitude2[i];
        sigI[i] = 0;
    }

    FST_semi_memo( sigR, sigI,
                   patCoefR, patCoefI,
                   bwIn, seminaive_naive_table,
                   (double *) workspace2, 0, bwIn,
                   &dctPlan, &fftPlan,
                   weights ) ;


    free( seminaive_naive_table ) ;
    free( seminaive_naive_tablespace ) ;



    /* combine coefficients */
    so3CombineCoef_fftw( bwIn, bwOut, degLim,
                         sigCoefR, sigCoefI,
                         patCoefR, patCoefI,
                         so3Coef ) ;




    /* now inverse so(3) */
    Inverse_SO3_Naive_fftw( bwOut,
                            so3Coef,
                            so3Sig,
                            workspace1,
                            workspace2,
                            workspace3,
                            &p1,
                            0 ) ;




    /* now find max value */
    maxval = 0.0 ;
    maxloc = 0 ;
    for ( i = 0 ; i < 8*bwOut*bwOut*bwOut ; i ++ )
    {
        /*
      if (so3Sig[i][0] >= maxval)
      {
      maxval = so3Sig[i][0];
      maxloc = i ;
      }
        */
        tmpval = NORM( so3Sig[i] );
        if ( tmpval > maxval )
        {
            maxval = tmpval;
            maxloc = i ;
        }

    }

    ii = floor( maxloc / (4.*bwOut*bwOut) );
    tmp = maxloc - (ii*4.*bwOut*bwOut);
    jj = floor( tmp / (2.*bwOut) );
    tmp = maxloc - (ii *4*bwOut*bwOut) - jj*(2*bwOut);
    kk = tmp ;

    printf("ii = %d\tjj = %d\tkk = %d\n", ii, jj, kk);

    printf("alpha = %f\nbeta = %f\ngamma = %f\n",
           M_PI*jj/((double) bwOut),
           M_PI*(2*ii+1)/(4.*bwOut),
           M_PI*kk/((double) bwOut) );


    /* now save data -> just the real part because the
     imaginary parts should all be 0 ... right?!? */
    printf("about to save data\n");
    FILE *fp;
    fp = fopen( "/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/resultCorrelation.csv", "w" );
    for( i = 0 ; i < 8*bwOut*bwOut*bwOut ; i ++ )
        fprintf(fp,"%.16f\n", so3Sig[i][0]);
    fclose( fp );


    fftw_destroy_plan( p1 );
    fftw_destroy_plan( fftPlan );
    fftw_destroy_plan( dctPlan );
    free( weights );
    fftw_free( so3Coef ) ;
    free( patCoefI );
    free( patCoefR );
    free( sigCoefI );
    free( sigCoefR );
    free( workspace3 );
    fftw_free( workspace2 );
    fftw_free( workspace1 );
    fftw_free( so3Sig ) ;
    free( sigI );
    free( sigR );
    free(voxelData1);
    free(voxelData2);
    free(spectrum1);
    free(spectrum2);
    free(phase1);
    free(phase2);
    free(magnitude1);
    free(magnitude2);
    free(magnitude1Shifted);
    free(magnitude2Shifted);
    return 0;

}
