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

  test program to loop through inverse-forward SO3 transforms
  lots of times to run error checks.

  - uses FFTW and Wigner-d symmetries

  spectral - spatial - spectral


  input: - bandwidth bw
         - loops
  	 - output file containing real and imaginary parts of errors

  example: test_soft_fftw bw loops errorFile

  example: test_soft_fftw 16 10 error.dat

*/


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "fftw3.h"
#include "soft20/utils_so3.h"
#include "soft20/makeweights.h"
#include "soft20/soft_fftw.h"
#include "soft20/csecond.h"
#include <pcl/io/pcd_io.h>

int main( int argc, char **argv ){
    const int numberOfPoints = 128;
    const double fromTo = 30;
    double *voxelData= new double[numberOfPoints*numberOfPoints*numberOfPoints];

    //fill with zeros
    for(int i = 0; i<numberOfPoints;i++){
        for(int j = 0; j<numberOfPoints;j++){
            for(int k = 0; k<numberOfPoints;k++){
                voxelData[i*(numberOfPoints*numberOfPoints)+j*(numberOfPoints)+k]=0;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData(new pcl::PointCloud<pcl::PointXYZ>);


    for(int i = 0; i<pointCloudInputData->points.size();i++){
        double positionPointX = pointCloudInputData->points[i].x;
        double positionPointY = pointCloudInputData->points[i].y;
        double positionPointZ = pointCloudInputData->points[i].z;
        int indexX = (int)((positionPointX+fromTo)/(fromTo*2)*numberOfPoints);
        int indexY = (int)((positionPointY+fromTo)/(fromTo*2)*numberOfPoints);
        int indexZ = (int)((0+fromTo)/(fromTo*2)*numberOfPoints);//set to zero
        voxelData[indexX*(numberOfPoints^2)+indexY*(numberOfPoints)+indexZ] = 1;
    }



    //transform pointcloud to fourier space

    fftw_complex *inputSpacialData,*outputSpacialData;

    inputSpacialData = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * numberOfPoints*numberOfPoints*numberOfPoints);
    outputSpacialData = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * numberOfPoints*numberOfPoints*numberOfPoints);

    //from voxel data to row major
    for(int i = 0; i<numberOfPoints;i++){
        for(int j = 0; j<numberOfPoints;j++){
            for(int k = 0; k<numberOfPoints;k++){
                inputSpacialData[i*(numberOfPoints^2)+j*(numberOfPoints)+k][0]=voxelData[i*(numberOfPoints^2)+j*(numberOfPoints)+k]; // real part
                inputSpacialData[i*(numberOfPoints^2)+j*(numberOfPoints)+k][1]=0; // imaginary part
            }
        }
    }





    fftw_plan planToFourier;
    planToFourier =  fftw_plan_dft_3d(numberOfPoints,numberOfPoints,numberOfPoints,inputSpacialData,outputSpacialData,FFTW_FORWARD,FFTW_ESTIMATE);


    fftw_execute(planToFourier);







    int l, k, j, bw, n, n3 ;
    int loops ;
    fftw_complex *signal ;
    fftw_complex *coeffsIn ;
    fftw_complex *coeffsOut ;
    fftw_complex *workspace_cx ;
    fftw_complex *workspace_cx2 ;
    fftw_plan p1, p2 ;
    int na[2], inembed[2], onembed[2] ;
    int rank, howmany, istride, idist, ostride, odist ;
    int m1, m2 ;
    // int cl, cl2 ;
    //  double fudge ;
    int tmpInt ;
    double *workspace_re ;
    double tstartF, tstopF, runtimeF ;
    double tstartI, tstopI, runtimeI ;
    double total_time ;
    double ave_error;
    double ave_relerror;
    double stddev_error, stddev_relerror;
    double *relerror;
    double *curmax ;
    double granderror, grandrelerror;
    double realtmp, imagtmp ;
    double origmag, tmpmag ;
    double *weights ;
    long int seed ;
    FILE *fp ;

//    if (argc < 3)
//    {
//        fprintf(stdout, "Usage: test_soft_fftw bw loops ");
//        fprintf(stdout, "[error_file]\n");
//        exit(0);
//    }

    bw = numberOfPoints/2;
    loops = 2;
    n = 2 * bw ;
    n3 = n * n * n ;

    /* signal */
    signal = (fftw_complex * ) fftw_malloc( sizeof( fftw_complex ) * n3 ) ;

    /* coefficients totalCoeffs_so3( bw) amount of space */
    coeffsIn = (fftw_complex * ) fftw_malloc(sizeof( fftw_complex ) * totalCoeffs_so3( bw ) ) ;
    coeffsOut = (fftw_complex * ) fftw_malloc(sizeof( fftw_complex ) * totalCoeffs_so3( bw ) ) ;

    /* now for LOTS OF workspace */
    workspace_cx = (fftw_complex * ) fftw_malloc(sizeof( fftw_complex ) * n3 ) ;
    workspace_cx2 = (fftw_complex * ) fftw_malloc(sizeof( fftw_complex ) * n3 ) ;
    workspace_re = ( double * ) malloc(sizeof( double ) *
                                       ( 24 * bw + 2 * bw * bw) );
    /** space for errors **/
    relerror = (double *) malloc(sizeof(double) * loops);
    curmax = (double *) malloc(sizeof(double) * loops);

    /* space for weights */
    weights = (double *) malloc(sizeof(double) * ( 2 * bw ));

    /* check if any problems allocating memory */
    if ( ( signal == NULL) || ( coeffsIn == NULL ) ||
         ( coeffsOut == NULL ) || ( workspace_cx == NULL ) ||
         ( workspace_cx2 == NULL ) || ( workspace_re == NULL ) ||
         ( relerror == NULL ) || ( curmax == NULL ) )
    {
        perror("Error in allocating memory");
        exit( 1 ) ;
    }


    /* create the plans */
    howmany = n*n ;
    idist = n ;
    odist = n ;
    rank = 2 ;
    inembed[0] = n ;
    inembed[1] = n*n ;
    onembed[0] = n ;
    onembed[1] = n*n ;
    istride = 1 ;
    ostride = 1 ;
    na[0] = 1 ;
    na[1] = n ;

    p1 = fftw_plan_many_dft( rank, na, howmany,
                             workspace_cx2, inembed,
                             istride, idist,
                             workspace_cx, onembed,
                             ostride, odist,
                             FFTW_BACKWARD, FFTW_MEASURE );

    p2 = fftw_plan_many_dft( rank, na, howmany,
                             workspace_cx, inembed,
                             istride, idist,
                             signal, onembed,
                             ostride, odist,
                             FFTW_FORWARD, FFTW_MEASURE );

    /* make the weights */
    makeweights2( bw, weights );

    /* generate seed for random number generator */
    time ( &seed ) ;
    srand48( seed ) ;

    /* initialize error */
    granderror = 0.0 ;
    grandrelerror = 0.0 ;

    /* initialize time */
    runtimeF = 0.0 ;
    runtimeI = 0.0 ;

    fprintf(stderr,"About to enter for-loop\n");

    for( k = 0 ; k < loops ; k ++ )
    {
        /* generate random coefficients */
        tmpInt = totalCoeffs_so3( bw ) ;
        for( l = 0 ; l < tmpInt ; l++ )
        {
            coeffsIn[ l ][0] = 2.0 * ( drand48() - 0.5 ) ;
            coeffsIn[ l ][1] = 2.0 * ( drand48() - 0.5 ) ;
        }

        /* generate random coefficients for
       a real-valued signal */
        /*
      for ( l = 0 ; l < bw ; l ++ )
      {
      for ( m1 = -l ; m1 < l + 1 ; m1 ++ )
      for ( m2 = -l ; m2 < 1 ; m2 ++ )
      {
      cl = so3CoefLoc( m1, m2, l, bw );
      coeffsIn[ cl ][0] =  2.0 * ( drand48() - 0.5 ) ;
      coeffsIn[ cl ][1] =  2.0 * ( drand48() - 0.5 ) ;

      if ( ((ABS(m1)+ABS(m2)) % 2) == 0 )
      fudge = 1. ;
      else
      fudge = -1. ;

      cl2 = so3CoefLoc( -m1, -m2, l, bw );
      coeffsIn[ cl2 ][0] = fudge*coeffsIn[ cl ][0]  ;
      coeffsIn[ cl2 ][1] = -fudge*coeffsIn[ cl ][1]  ;
      }
      cl = so3CoefLoc(0, 0, l, bw);
      coeffsIn[ cl ][0] = 2.0 * ( drand48() - 0.5 ) ;
      coeffsIn[ cl ][1] = 0 ;
      }
        */

        /* turn on stopwatch */
        tstartI = csecond( ) ;

        /* now do inverse transform */
        Inverse_SO3_Naive_fftw( bw,
                                coeffsIn,
                                signal,
                                workspace_cx,
                                workspace_cx2,
                                workspace_re,
                                &p2,
                                0 ) ;

        /* turn off stopwatch */
        tstopI = csecond( ) ;
        runtimeI += tstopI - tstartI ;
        fprintf(stderr,"inv time \t = %.4e\n", tstopI - tstartI);

        /* turn on stopwatch */
        tstartF = csecond( ) ;

        /* now do the forward transform */
        Forward_SO3_Naive_fftw( bw,
                                signal,
                                coeffsOut,
                                workspace_cx,
                                workspace_cx2,
                                workspace_re,
                                weights,
                                &p1,
                                0 );

        /* turn off stopwatch */
        tstopF = csecond( ) ;
        runtimeF += tstopF - tstartF ;
        fprintf(stderr,"for time \t = %.4e\n", tstopF - tstartF);

        relerror[ k ] = 0.0 ;
        curmax[ k ] = 0.0 ;
        /* now figure out errors */
        for( j = 0 ; j < tmpInt ; j ++ )
        {
            realtmp = coeffsIn[ j ][0] - coeffsOut[ j ][0] ;
            imagtmp = coeffsIn[ j ][1] - coeffsOut[ j ][1] ;
            origmag = sqrt((coeffsIn[ j ][0]*coeffsIn[ j ][0]) +
                           (coeffsIn[ j ][1]*coeffsIn[ j ][1]));
            tmpmag = sqrt((realtmp*realtmp) +
                          (imagtmp*imagtmp));
            relerror[ k ] =
                    MAX(relerror[ k ] , tmpmag/(origmag + pow(10.0, -50.0)));
            curmax[ k ] = MAX( curmax[ k ], tmpmag );
        }


        fprintf(stderr,"r-o error\t = %.12f\n", curmax[ k ]);
        fprintf(stderr,"(r-o)/o error\t = %.12f\n\n", relerror[ k ]);

        granderror += curmax[ k ];
        grandrelerror += relerror[ k ];
    }


    total_time = runtimeF + runtimeI ;

    ave_error = granderror / ( (double) loops );
    ave_relerror = grandrelerror / ( (double) loops );
    stddev_error = 0.0 ; stddev_relerror = 0.0;
    for( k = 0 ; k < loops ; k ++ )
    {
        stddev_error += pow( ave_error - curmax[ k ] , 2.0 );
        stddev_relerror += pow( ave_relerror - relerror[ k ] , 2.0 );
    }
    /*** this won't work if loops == 1 ***/
    if( loops != 1 )
    {
        stddev_error = sqrt(stddev_error / ( (double) (loops - 1) ) );
        stddev_relerror = sqrt(stddev_relerror / ( (double) (loops - 1) ) );
    }


    fprintf(stderr,"Program: test_soft_fftw\n");
    fprintf(stderr,"Bandwidth = %d\n", bw);

#ifndef WALLCLOCK
    fprintf(stderr,"Total elapsed cpu time :\t\t %.4e seconds.\n",
            total_time);
    fprintf(stderr,"Average cpu forward per iteration:\t %.4e seconds.\n",
            runtimeF/((double) loops));
    fprintf(stderr,"Average cpu inverse per iteration:\t %.4e seconds.\n",
            runtimeI/((double) loops));
#else
    fprintf(stderr,"Total elapsed wall time :\t\t %.4e seconds.\n",
	  total_time);
  fprintf(stderr,"Average wall forward per iteration:\t %.4e seconds.\n",
	  runtimeF/((double) loops));
  fprintf(stderr,"Average wall inverse per iteration:\t %.4e seconds.\n",
	  runtimeI/((double) loops));
#endif

    fprintf(stderr,"Average r-o error:\t\t %.4e\t",
            granderror/((double) loops));
    fprintf(stderr,"std dev: %.4e\n",stddev_error);
    fprintf(stderr,"Average (r-o)/o error:\t\t %.4e\t",
            grandrelerror/((double) loops));
    fprintf(stderr,"std dev: %.4e\n\n",stddev_relerror);


    /* if an error file is asked for (probably just after one loop) */

    fp = fopen( "testfile.csv" , "w" );
    for ( l = 0 ; l < bw ; l ++ )
        for ( m1 = -l ; m1 < l + 1 ; m1 ++ )
            for ( m2 = -l ; m2 < l + 1 ; m2 ++ )
            {
                k = so3CoefLoc( m1, m2, l, bw ) ;
                fprintf( fp, "l = %d  m1 = %d  m2 = %d  %.15f\t%.15f\n",
                         l, m1, m2,
                         coeffsIn[ k ][0] - coeffsOut[ k ][0],
                         coeffsIn[ k ][1] - coeffsOut[ k ][1] );
            }
    fclose( fp ) ;


    /* free up memory (and there's lots of it) */
    fftw_destroy_plan( p2 );
    fftw_destroy_plan( p1 );


    free( weights );
    free( curmax );
    free( relerror );

    free( workspace_re );

    fftw_free( workspace_cx2 );
    fftw_free( workspace_cx );
    fftw_free( coeffsOut );
    fftw_free( coeffsIn );
    fftw_free( signal );

    return 0 ;
}
