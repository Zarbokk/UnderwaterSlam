//
// Created by tim-linux on 01.03.22.
//

#ifndef UNDERWATERSLAM_SOFFTCORRELATIONCLASS_H
#define UNDERWATERSLAM_SOFFTCORRELATIONCLASS_H
#include "fftw3.h"
#include "soft20/s2_cospmls.h"
#include "soft20/s2_semi_memo.h"
#include "soft20/makeweights.h"
#include "soft20/so3_correlate_fftw.h"
#include "soft20/soft_fftw.h"

class sofftCorrelationClass {
public:
    sofftCorrelationClass(int N){
        this->N = N;
    }

private:
    int N;//describes the size of the overall voxel system

    int i;
    int bwIn, bwOut, degLim;

    fftw_complex *workspace1, *workspace2;
    double *workspace3;
    double *sigR, *sigI;
    double *sigCoefR, *sigCoefI;
    double *patCoefR, *patCoefI;
    fftw_complex *so3Sig, *so3Coef;
    fftw_plan p1;
    int na[2], inembed[2], onembed[2];
    int rank, howmany, istride, idist, ostride, odist;
    int tmp, maxloc, ii, jj, kk;
    double maxval, tmpval;
    double *weights;
    double *seminaive_naive_tablespace;
    double **seminaive_naive_table;
    fftw_plan dctPlan, fftPlan;
    int howmany_rank;

};


#endif //UNDERWATERSLAM_SOFFTCORRELATIONCLASS_H
