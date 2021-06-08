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
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "fftw3.h"
#include "soft20/wrap_fftw.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

int main ( int argc,
           char **argv )
{
    const int numberOfPoints = 128;
    const double fromTo = 30;
    double *voxelData1= new double[numberOfPoints * numberOfPoints * numberOfPoints];
    double *voxelData2= new double[numberOfPoints * numberOfPoints * numberOfPoints];


    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/tim/DataForTests/withoutRotation/after_voxel_1.pcd",
                         *pointCloudInputData1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2(new pcl::PointCloud<pcl::PointXYZ>);

    //90 degree rotation
    Eigen::Matrix4d transformation90Degree;
    Eigen::AngleAxisd rotation_vector2(90.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformation90Degree.block<3, 3>(0, 0) = tmpMatrix3d;
    transformation90Degree(3, 3) = 1;

    pcl::transformPointCloud(*pointCloudInputData1, *pointCloudInputData2, transformation90Degree);



    //fill with zeros
    for(int i = 0; i<numberOfPoints;i++){
        for(int j = 0; j<numberOfPoints;j++){
            for(int k = 0; k<numberOfPoints;k++){
                voxelData1[i * (numberOfPoints * numberOfPoints) + j * (numberOfPoints) + k]=0;
                voxelData2[i * (numberOfPoints * numberOfPoints) + j * (numberOfPoints) + k]=0;
            }
        }
    }

    for(int i = 0; i<pointCloudInputData1->points.size();i++){
        double positionPointX = pointCloudInputData1->points[i].x;
        double positionPointY = pointCloudInputData1->points[i].y;
        double positionPointZ = pointCloudInputData1->points[i].z;
        int indexX = (int)((positionPointX+fromTo)/(fromTo*2)*numberOfPoints);
        int indexY = (int)((positionPointY+fromTo)/(fromTo*2)*numberOfPoints);
        int indexZ = (int)((0+fromTo)/(fromTo*2)*numberOfPoints);//set to zero
        voxelData1[indexX * (numberOfPoints ^ 2) + indexY * (numberOfPoints) + indexZ] = 1;
    }

    for(int i = 0; i<pointCloudInputData2->points.size();i++){
        double positionPointX = pointCloudInputData2->points[i].x;
        double positionPointY = pointCloudInputData2->points[i].y;
        double positionPointZ = pointCloudInputData2->points[i].z;
        int indexX = (int)((positionPointX+fromTo)/(fromTo*2)*numberOfPoints);
        int indexY = (int)((positionPointY+fromTo)/(fromTo*2)*numberOfPoints);
        int indexZ = (int)((0+fromTo)/(fromTo*2)*numberOfPoints);//set to zero
        voxelData2[indexX * (numberOfPoints ^ 2) + indexY * (numberOfPoints) + indexZ] = 1;
    }


    //transform pointcloud to fourier space

    fftw_complex *inputSpacialData1,*inputSpacialData2,*outputSpacialData;

    inputSpacialData1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * numberOfPoints * numberOfPoints * numberOfPoints);
    inputSpacialData2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * numberOfPoints * numberOfPoints * numberOfPoints);
    outputSpacialData = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * numberOfPoints*numberOfPoints*numberOfPoints);

    //from voxel data to row major
    for(int i = 0; i<numberOfPoints;i++){
        for(int j = 0; j<numberOfPoints;j++){
            for(int k = 0; k<numberOfPoints;k++){
                inputSpacialData1[i * (numberOfPoints ^ 2) + j * (numberOfPoints) + k][0]=voxelData1[i * (numberOfPoints ^ 2) + j * (numberOfPoints) + k]; // real part
                inputSpacialData1[i * (numberOfPoints ^ 2) + j * (numberOfPoints) + k][1]=0; // imaginary part
                inputSpacialData2[i * (numberOfPoints ^ 2) + j * (numberOfPoints) + k][0]=voxelData2[i * (numberOfPoints ^ 2) + j * (numberOfPoints) + k]; // real part
                inputSpacialData2[i * (numberOfPoints ^ 2) + j * (numberOfPoints) + k][1]=0; // imaginary part
            }
        }
    }



//
//
//    fftw_plan planToFourier;
//    planToFourier =  fftw_plan_dft_3d(numberOfPoints,numberOfPoints,numberOfPoints,inputSpacialData1,outputSpacialData,FFTW_FORWARD,FFTW_ESTIMATE);
//
//
//    fftw_execute(planToFourier);



















    FILE *fp ;
    int i ;
    int n, bw, isReal ;
    double *signal, *pattern ;
    double alpha, beta, gamma ;

    if (argc < 5 )
    {
        printf("test_soft_sym_correlate2_wrap signalFile patternFile bw isReal\n");
        printf(" isReal = 1: signal and pattern strictly real (no interleaved)\n");
        printf(" isReal = 0: signal and pattern complex (interleaved)\n");
        exit(0) ;
    }

    bw = atoi( argv[3] );
    n = 2 * bw ;

    isReal = atoi( argv[4] );

    /* allocate space to hold signal, pattern */
    if ( isReal )
    {
        signal = (double *) malloc( sizeof(double) * (n * n) );
        pattern = (double *) malloc( sizeof(double) * (n * n) );
    }
    else
    {
        signal = (double *) malloc( sizeof(double) * (2 * n * n) );
        pattern = (double *) malloc( sizeof(double) * (2 * n * n) );
    }

    /****
         At this point, check to see if all the memory has been
         allocated. If it has not, there's no point in going further.
    ****/

    if ( (signal == NULL) || (pattern == NULL) )
    {
        perror("Error in allocating memory");
        exit( 1 ) ;
    }

    printf("Reading in signal file\n");
    /* read in SIGNAL samples */
    fp = fopen(argv[1],"r");
    if ( isReal )
    {
        for ( i = 0 ; i < n * n ; i ++ )
        {
            fscanf(fp,"%lf", signal + i);
        }
    }
    else
    {
        for ( i = 0 ; i < 2 * n * n ; i ++ )
        {
            fscanf(fp,"%lf", signal + i);
        }
    }
    fclose( fp );

    printf("Reading in pattern file\n");
    /* read in PATTERN samples */
    fp = fopen(argv[2],"r");
    if ( isReal )
    {
        for ( i = 0 ; i < n * n ; i ++ )
        {
            fscanf(fp,"%lf", pattern + i);
        }
    }
    else
    {
        for ( i = 0 ; i < 2 * n * n ; i ++ )
        {
            fscanf(fp,"%lf", pattern + i);
        }
    }
    fclose( fp );

    /* now correlate */
    softFFTWCor2( bw,
                  signal,
                  pattern,
                  &alpha, &beta, &gamma,
                  isReal) ;

    /* print results */
    printf("alpha = %f\nbeta = %f\ngamma = %f\n",
           alpha, beta, gamma );

    /* clean up */
    free( pattern );
    free( signal ) ;

    return 0 ;

}
