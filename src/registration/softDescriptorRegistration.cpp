//
// Created by tim-linux on 01.03.22.
//

#include "softDescriptorRegistration.h"

bool compareTwoAngleCorrelation(angleAndCorrelation i1, angleAndCorrelation i2) {
    return (i1.angle < i2.angle);
}

bool compareTwoPeaks(indexPeak i1, indexPeak i2) {
    return (i1.height > i2.height);
}

double thetaIncrement(double index, int bandwidth) {
    return M_PI * (2 * index + 1) / (4.0 * bandwidth);
}

double phiIncrement(double index, int bandwidth) {
    return M_PI * index / bandwidth;
}


double
softDescriptorRegistration::getSpectrmFromPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData, double voxelData[], fftw_complex spectrum[],
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
    free(planToFourierVoxel);
    return maximumMagnitude;
}


Eigen::Matrix4d
softDescriptorRegistration::registrationOfTwoPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData1,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2,
                                                 const double cellSize) {

    double maximumScan1 = this->getSpectrmFromPCL(pointCloudInputData1, this->voxelData1, this->spectrum1, this->magnitude1, this->phase1, cellSize*this->N,
                                                  this->N);
    double maximumScan2 = this->getSpectrmFromPCL(pointCloudInputData2, this->voxelData2, this->spectrum2, this->magnitude2, this->phase2, cellSize*this->N,
                                                  this->N);

    double globalMaximumMagnitude;
    if (maximumScan2 < maximumScan1) {
        globalMaximumMagnitude = maximumScan1;
    } else {
        globalMaximumMagnitude = maximumScan2;
    }

    //normalize and fftshift
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


    //re-initialize to zero
    for (int i = 0; i < N * N; i++) {
        resampledMagnitude1[i] = 0;
        resampledMagnitude2[i] = 0;
        resampledMagnitude1TMP[i] = 0;
        resampledMagnitude2TMP[i] = 0;
    }

    int minRNumber = 4;
    int maxRNumber = N / 2 - 2;
    int bandwidth = N / 2;

    for (int r = minRNumber; r < maxRNumber; r++) {
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                int xIndex = std::round((double) r * std::sin(thetaIncrement((double) j + 1, bandwidth)) *
                                        std::cos(phiIncrement((double) k + 1, bandwidth)) + bandwidth) - 1;
                int yIndex = std::round((double) r * std::sin(thetaIncrement((double) j + 1, bandwidth)) *
                                        std::sin(phiIncrement((double) k + 1, bandwidth)) + bandwidth) - 1;
                int zIndex =
                        std::round((double) r * std::cos(thetaIncrement((double) j + 1, bandwidth)) + bandwidth) - 1;
                resampledMagnitude1TMP[k + j * bandwidth * 2] =
                        255 * magnitude1Shifted[zIndex + N * (yIndex + N * xIndex)];
                resampledMagnitude2TMP[k + j * bandwidth * 2] =
                        255 * magnitude2Shifted[zIndex + N * (yIndex + N * xIndex)];
            }
        }
        cv::Mat magTMP1(N, N, CV_64FC1, resampledMagnitude1TMP);
        cv::Mat magTMP2(N, N, CV_64FC1, resampledMagnitude2TMP);
        magTMP1.convertTo(magTMP1, CV_8UC1);
        magTMP2.convertTo(magTMP2, CV_8UC1);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3);
        clahe->apply(magTMP1, magTMP1);
        clahe->apply(magTMP2, magTMP2);
        for (int j = 0; j < 2 * bandwidth; j++) {
            for (int k = 0; k < 2 * bandwidth; k++) {
                resampledMagnitude1[k + j * bandwidth * 2] = resampledMagnitude1[k + j * bandwidth * 2] +
                                                             ((double) magTMP1.data[k + j * bandwidth * 2]) / 255.0;
                resampledMagnitude2[k + j * bandwidth * 2] = resampledMagnitude2[k + j * bandwidth * 2] +
                                                             ((double) magTMP2.data[k + j * bandwidth * 2]) / 255.0;
            }
        }

    }

    //use sofft descriptor to calculate the correlation
    this->sofftCorrelationObject.correlationOfTwoSignalsInSO3();


    double currentThetaAngle;
    double currentPsiAngle;
    double maxCorrelation = 0;
    std::vector<angleAndCorrelation> correlationOfAngle;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            currentThetaAngle = i * 2.0 * M_PI / N;
            currentPsiAngle = j * 2.0 * M_PI / N;
            //[i + N * j]
            angleAndCorrelation tmpHolding;
            tmpHolding.correlation = so3Sig[i + N * (j + N * 0)][0]; // real part
            if (tmpHolding.correlation > maxCorrelation) {
                maxCorrelation = tmpHolding.correlation;
            }
            tmpHolding.angle = std::fmod(currentThetaAngle + currentPsiAngle + 4 * M_PI, 2 * M_PI);
            correlationOfAngle.push_back(tmpHolding);
        }
    }


    std::sort(correlationOfAngle.begin(), correlationOfAngle.end(), compareTwoAngleCorrelation);

    //std::cout << correlationOfAngle[1].angle << std::endl;
    std::vector<float> correlationAveraged, angleList;
    double currentAverageAngle = correlationOfAngle[0].angle;
    //angleList.push_back(currentAverageAngle);
    int numberOfAngles = 1;
    double averageCorrelation = correlationOfAngle[0].correlation;
    for (int i = 1; i < correlationOfAngle.size(); i++) {

        if (std::abs(currentAverageAngle - correlationOfAngle[i].angle) < 1.0 / N / 4.0) {
            numberOfAngles = numberOfAngles + 1;
            averageCorrelation = averageCorrelation + correlationOfAngle[i].correlation;
        } else {

            correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles / maxCorrelation));
            angleList.push_back((float) currentAverageAngle);
            numberOfAngles = 1;
            averageCorrelation = correlationOfAngle[i].correlation;
            currentAverageAngle = correlationOfAngle[i].angle;

        }
    }
    correlationAveraged.push_back((float) (averageCorrelation / numberOfAngles / maxCorrelation));
    angleList.push_back((float) currentAverageAngle);

    auto minmax = std::min_element(correlationAveraged.begin(), correlationAveraged.end());
    long distanceToMinElement = std::distance(correlationAveraged.begin(), minmax);
    std::rotate(correlationAveraged.begin(), correlationAveraged.begin() + distanceToMinElement,
                correlationAveraged.end());

    std::vector<int> out;

    PeakFinder::findPeaks(correlationAveraged, out, true);
    std::rotate(correlationAveraged.begin(),
                correlationAveraged.begin() + correlationAveraged.size() - distanceToMinElement,
                correlationAveraged.end());
    for (int i = 0; i < out.size(); ++i) {
        out[i] = out[i] + (int) distanceToMinElement;
        if (out[i] >= correlationAveraged.size()) {
            out[i] = out[i] - correlationAveraged.size();
        }
    }
    std::vector<double> xShiftList, yShiftList, heightPeakList, estimatedAngleList;


    // for each angle calculate the shift correlation of that angle
    //for( int angleIndex=0; angleIndex<out.size(); ++angleIndex){
    for (int angleIndex = 0; angleIndex < out.size(); ++angleIndex) {

        double currentAngle = -angleList[out[angleIndex]];//describes angle from A to B, therefore we have to reverse the angle
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputDataTMP2(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4d rotationMatrixTMP;
        //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd tmpRotVec(currentAngle, Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d tmpMatrix3d = tmpRotVec.toRotationMatrix();
        rotationMatrixTMP.block<3, 3>(0, 0) = tmpMatrix3d;
        rotationMatrixTMP(0, 3) = 0;//x
        rotationMatrixTMP(1, 3) = 0;//y
        rotationMatrixTMP(2, 3) = 0;//z
        rotationMatrixTMP(3, 3) = 1;//1
        //copy the rotated PCL from PCL1 to PCL2
        pcl::transformPointCloud(*pointCloudInputData2, *pointCloudInputDataTMP2, rotationMatrixTMP);


        maximumScan1 = getSpectrmFromPCL(pointCloudInputData1, voxelData1, spectrum1, magnitude1, phase1, cellSize*this->N,
                                         N);
        maximumScan2 = getSpectrmFromPCL(pointCloudInputDataTMP2, voxelData2, spectrum2, magnitude2, phase2, cellSize*this->N,
                                         N);


        fftw_complex *resultingPhaseDiff, *resultingShiftPeaks;
        resultingPhaseDiff = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);
        resultingShiftPeaks = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * N * N * N);

        double *resultingCorrelation;
        resultingCorrelation = (double *) malloc(sizeof(double) * N * N * N);
        //calc phase diff and fftshift
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                for (int k = 0; k < N; k++) {

                    int indexX = (N / 2 + i) % N;
                    int indexY = (N / 2 + j) % N;
                    int indexZ = (N / 2 + k) % N;//indexZ + N * (indexY + N * indexX)
                    std::complex<double> tmpComplex;
                    tmpComplex.real(0);
                    tmpComplex.imag(
                            phase1[indexZ + N * (indexY + N * indexX)] - phase2[indexZ + N * (indexY + N * indexX)]);
                    std::complex<double> resultCompexNumber = std::exp(tmpComplex);

                    resultingPhaseDiff[k + N * (j + N * i)][0] = resultCompexNumber.real();
                    resultingPhaseDiff[k + N * (j + N * i)][1] = resultCompexNumber.imag();
                }
            }
        }

        fftw_plan planToFourierVoxel;
        planToFourierVoxel = fftw_plan_dft_3d(N, N, N, resultingPhaseDiff,
                                              resultingShiftPeaks, FFTW_BACKWARD, FFTW_ESTIMATE);
        fftw_execute(planToFourierVoxel);
        double maximumCorrelation = 0;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                for (int k = 0; k < N; k++) {
                    int indexX = (N / 2 + i) % N;
                    int indexY = (N / 2 + j) % N;
                    int indexZ = (N / 2 + k) % N;
                    //calc magnitude and fftshift
                    resultingCorrelation[indexZ + N * (indexY + N * indexX)] = sqrt(
                            resultingShiftPeaks[k + N * (j + N * i)][0] *
                            resultingShiftPeaks[k + N * (j + N * i)][0] +
                            resultingShiftPeaks[k + N * (j + N * i)][1] *
                            resultingShiftPeaks[k + N *
                                                    (j + N * i)][1]); // real part;

                    if (maximumCorrelation < resultingCorrelation[indexZ + N * (indexY + N * indexX)]) {
                        maximumCorrelation = resultingCorrelation[indexZ + N * (indexY + N * indexX)];
                    }
                }
            }
        }


        double *correlation2DResult;
        correlation2DResult = (double *) malloc(sizeof(double) * N * N);
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                correlation2DResult[j + N * i] = resultingCorrelation[N / 2 + N * (j + N * i)];
            }
        }
        std::vector<indexPeak> localMaximaVector;
        PeakFinder::findPeaks2D(correlation2DResult, localMaximaVector, N);

        std::sort(localMaximaVector.begin(), localMaximaVector.end(), compareTwoPeaks);

        std::vector<double> differencePeaks;
        int maximumDiffIterator = 0;
        double maximumDiffValue = 0;
        for (int i = 1; i < localMaximaVector.size(); i++) {
            differencePeaks.push_back(localMaximaVector[i - 1].height - localMaximaVector[i].height);
            if (localMaximaVector[i - 1].height - localMaximaVector[i].height > maximumDiffValue) {
                maximumDiffValue = localMaximaVector[i - 1].height - localMaximaVector[i].height;
                maximumDiffIterator = i - 1;
            }
        }
        maximumDiffIterator = 0;//set it anyway to zero; can be used to know if the result seems valid

        heightPeakList.push_back(localMaximaVector[maximumDiffIterator].height);
        xShiftList.push_back((localMaximaVector[maximumDiffIterator].y - N / 2.0) * cellSize*this->N * 2.0 / N);
        yShiftList.push_back((localMaximaVector[maximumDiffIterator].x - N / 2.0) * cellSize*this->N * 2.0 / N);
        estimatedAngleList.push_back(currentAngle);


        free(resultingPhaseDiff);
        free(resultingShiftPeaks);
        free(resultingCorrelation);
    }

    auto maxElementIter = std::max_element(heightPeakList.begin(), heightPeakList.end());
    int distanceToMaxElement = (int) std::distance(heightPeakList.begin(), maxElementIter);


    std::cout << "#######################################################" << std::endl;
    std::cout << "Estimated  Angle: " << estimatedAngleList[distanceToMaxElement] << std::endl;
    std::cout << "Estimated xShift: " << xShiftList[distanceToMaxElement] << std::endl;
    std::cout << "Estimated yShift: " << yShiftList[distanceToMaxElement] << std::endl;

    Eigen::Matrix4d estimatedRotationScans;//from second scan to first
    //Eigen::AngleAxisd rotation_vector2(65.0 / 180.0 * 3.14159, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd rotation_vectorTMP(estimatedAngleList[distanceToMaxElement], Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d tmpRotMatrix3d = rotation_vectorTMP.toRotationMatrix();
    estimatedRotationScans.block<3, 3>(0, 0) = tmpRotMatrix3d;
    estimatedRotationScans(0, 3) = xShiftList[distanceToMaxElement];
    estimatedRotationScans(1, 3) = yShiftList[distanceToMaxElement];
    estimatedRotationScans(2, 3) = 0;
    estimatedRotationScans(3, 3) = 1;

    std::cout << estimatedRotationScans << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInputData2RotatedTo1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pointCloudInputData2, *pointCloudInputData2RotatedTo1, estimatedRotationScans);

    pcl::io::savePCDFileASCII(
            "/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/rotatedAndThenRegisteredScan.pcd",
            *pointCloudInputData2RotatedTo1);
    pcl::io::savePCDFileASCII("/home/tim-linux/Documents/matlabTestEnvironment/registrationFourier/originalScan.pcd",
                              *pointCloudInputData1);


}
