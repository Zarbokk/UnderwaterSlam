//
// Created by jurobotics on 15.09.21.
//

#include "generalHelpfulTools.h"

Eigen::Vector3d generalHelpfulTools::getRollPitchYaw(Eigen::Quaterniond quat) {
    tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r, p, y);
    return returnVector;
}

Eigen::Matrix4d generalHelpfulTools::getTransformationMatrixFromRPY(double roll, double pitch, double yaw) {
    Eigen::Quaterniond rotationAsQuaternion = generalHelpfulTools::getQuaternionFromRPY(roll, pitch, yaw);
    Eigen::Matrix4d returnMatrix = Eigen::Matrix4d::Identity();
    returnMatrix.block<3, 3>(0, 0) = rotationAsQuaternion.toRotationMatrix();
    return returnMatrix;
}

Eigen::Quaterniond generalHelpfulTools::getQuaternionFromRPY(double roll, double pitch, double yaw) {
//        tf2::Matrix3x3 m;
//        m.setRPY(roll,pitch,yaw);
//        Eigen::Matrix3d m2;
    tf2::Quaternion qtf2;
    qtf2.setRPY(roll, pitch, yaw);
    Eigen::Quaterniond q;
    q.x() = qtf2.x();
    q.y() = qtf2.y();
    q.z() = qtf2.z();
    q.w() = qtf2.w();

//        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
//            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return q;
}

double generalHelpfulTools::angleDiff(double first, double second) {//first-second
    return atan2(sin(first - second), cos(first - second));
}


Eigen::Matrix4d generalHelpfulTools::interpolationTwo4DTransformations(Eigen::Matrix4d &transformation1,
                                                                       Eigen::Matrix4d &transformation2, double &t) {
    if (t < 0 || t > 1) {
        std::cout << "t value not between 0 and 1: " << t << std::endl;
        exit(-1);
    }
    Eigen::Vector3d translation1 = transformation1.block<3, 1>(0, 3);
    Eigen::Vector3d translation2 = transformation2.block<3, 1>(0, 3);
    Eigen::Quaterniond rot1(transformation1.block<3, 3>(0, 0));
    Eigen::Quaterniond rot2(transformation2.block<3, 3>(0, 0));

//    Eigen::Quaterniond rotTMP = rot1.inverse()*rot2;
//    Eigen::Quaterniond resultingRot = rotTMP.slerp(t, Eigen::Quaterniond(1,0,0,0));
//    Eigen::Vector3d resultingTranslation = (translation2-translation1)*t;
//    Eigen::Matrix4d resultingTransformation = Eigen::Matrix4d::Identity();
//    resultingTransformation.block<3, 3>(0, 0) = resultingRot.toRotationMatrix();
//    resultingTransformation.block<3, 1>(0, 3) = resultingTranslation;

    Eigen::Quaterniond resultingRot = rot1.slerp(t, rot2);
    Eigen::Vector3d resultingTranslation = translation1 * t + translation2 * (1.0 - t);

    Eigen::Matrix4d resultingTransformation = Eigen::Matrix4d::Identity();
    resultingTransformation.block<3, 3>(0, 0) = resultingRot.toRotationMatrix();
    resultingTransformation.block<3, 1>(0, 3) = resultingTranslation;

    return resultingTransformation;


}


Eigen::Matrix4d
generalHelpfulTools::getTransformationMatrix(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 1>(0, 3) = translation;
    transformation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    return transformation;
}

double generalHelpfulTools::weighted_mean(const std::vector<double> &data) {
    double mean = 0.0;

    for (int i = 0; i < data.size(); i++) {
        mean += data[i];
    }
    return mean / double(data.size());
}

void generalHelpfulTools::smooth_curve(const std::vector<double> &input, std::vector<double> &smoothedOutput,
                                       int window_half_width) {

    int window_width = 2 * window_half_width + 1;

    int size = input.size();

    std::vector<double> sample(window_width);

    for (int i = 0; i < size; i++) {

        for (int j = 0; j < window_width; j++) {

            int shifted_index = i + j - window_half_width;
            if (shifted_index < 0) shifted_index = 0;
            if (shifted_index > size - 1) shifted_index = size - 1;
            sample[j] = input[shifted_index];

        }

        smoothedOutput.push_back(generalHelpfulTools::weighted_mean(sample));

    }

}


double generalHelpfulTools::createVoxelOfGraph(double voxelData[], int indexStart,
                                               Eigen::Matrix4d transformationInTheEndOfCalculation,
                                               int numberOfPoints, graphSlamSaveStructure &usedGraph,
                                               double ignoreDistanceToRobot, double dimensionOfVoxelData) {
    int *voxelDataIndex;
    voxelDataIndex = (int *) malloc(sizeof(int) * numberOfPoints * numberOfPoints);
    //set zero voxel and index
    for (int i = 0; i < numberOfPoints * numberOfPoints; i++) {
        voxelDataIndex[i] = 0;
        voxelData[i] = 0;
    }


    int i = 0;
    do {
        //calculate the position of each intensity and create an index in two arrays. First in voxel data, and second save number of intensities.


        //get position of current intensityRay
        Eigen::Matrix4d transformationOfIntensityRay =
                usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                usedGraph.getVertexList()->at(indexStart - i).getTransformation();

        //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
        Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                         usedGraph.getVertexList()->at(
                                                                                                                 indexStart -
                                                                                                                 i).getIntensities().angle);

        int ignoreDistance = (int) (ignoreDistanceToRobot /
                                    (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                     ((double) usedGraph.getVertexList()->at(
                                             indexStart - i).getIntensities().intensities.size())));


        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            double distanceOfIntensity =
                    j / ((double) usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities.size()) *
                    ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);

            int incrementOfScan = usedGraph.getVertexList()->at(indexStart - i).getIntensities().increment;
            for (int l = -incrementOfScan - 5; l <= incrementOfScan + 5; l++) {
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);
                double rotationOfPoint = l / 400.0;
                Eigen::Matrix4d rotationForBetterView = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                            rotationOfPoint);
                positionOfIntensity = rotationForBetterView * positionOfIntensity;

                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //calculate index dependent on  DIMENSION_OF_VOXEL_DATA and numberOfPoints the middle
                int indexX =
                        (int) (positionOfIntensity.x() / (dimensionOfVoxelData / 2) * numberOfPoints /
                               2) +
                        numberOfPoints / 2;
                int indexY =
                        (int) (positionOfIntensity.y() / (dimensionOfVoxelData / 2) * numberOfPoints /
                               2) +
                        numberOfPoints / 2;


                if (indexX < numberOfPoints && indexY < numberOfPoints && indexY >= 0 &&
                    indexX >= 0) {
                    //                    std::cout << indexX << " " << indexY << std::endl;
                    //if index fits inside of our data, add that data. Else Ignore
                    voxelDataIndex[indexY + numberOfPoints * indexX] =
                            voxelDataIndex[indexY + numberOfPoints * indexX] + 1;
                    //                    std::cout << "Index: " << voxelDataIndex[indexY + numberOfPoints * indexX] << std::endl;
                    voxelData[indexY + numberOfPoints * indexX] =
                            voxelData[indexY + numberOfPoints * indexX] +
                            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j];
                    //                    std::cout << "Intensity: " << voxelData[indexY + numberOfPoints * indexX] << std::endl;
                    //                    std::cout << "random: " << std::endl;
                }
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    double maximumOfVoxelData = 0;
    for (i = 0; i < numberOfPoints * numberOfPoints; i++) {
        if (voxelDataIndex[i] > 0) {
            voxelData[i] = voxelData[i] / voxelDataIndex[i];
            if (maximumOfVoxelData < voxelData[i]) {
                maximumOfVoxelData = voxelData[i];
            }
            //std::cout << voxelData[i] << std::endl;

        }
    }// @TODO calculate the maximum and normalize "somehow"




    free(voxelDataIndex);
    return maximumOfVoxelData;
}

pcl::PointCloud<pcl::PointXYZ> generalHelpfulTools::createPCLFromGraphOneValue(int indexStart,
                                                          Eigen::Matrix4d transformationInTheEndOfCalculation, graphSlamSaveStructure &usedGraph, double ignoreDistanceToRobo,double thresholdFactorPoint) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    //create array with all intencities.
    // Calculate maximum of intensities.
    // only use maximum of 10% of max value as points
    int i = 0;
    double maximumIntensity = 0;

    int ignoreDistance = (int) (ignoreDistanceToRobo /
                                (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                 ((double) usedGraph.getVertexList()->at(
                                         indexStart - i).getIntensities().intensities.size())));

    do {
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                maximumIntensity) {
                maximumIntensity = usedGraph.getVertexList()->at(
                        indexStart - i).getIntensities().intensities[j];
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);

    double thresholdIntensityScan = maximumIntensity * thresholdFactorPoint;//maximum intensity of 0.9



    i = 0;
    do {
        //find max Position
        int maxPosition = ignoreDistance;
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition]) {
                maxPosition = j;
            }
        }
        if (maxPosition > ignoreDistance &&
            usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[maxPosition] >
            thresholdIntensityScan) {
            Eigen::Matrix4d transformationOfIntensityRay =
                    usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                    usedGraph.getVertexList()->at(indexStart - i).getTransformation();

            //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
            Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                             usedGraph.getVertexList()->at(
                                                                                                                     indexStart -
                                                                                                                     i).getIntensities().angle);

            double distanceOfIntensity =
                    maxPosition / ((double) usedGraph.getVertexList()->at(
                            indexStart - i).getIntensities().intensities.size()) *
                    ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
            Eigen::Vector4d positionOfIntensity(
                    distanceOfIntensity,
                    0,
                    0,
                    1);

            positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                  rotationOfSonarAngleMatrix * positionOfIntensity;
            //create point for PCL
            pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                   (float) positionOfIntensity[1],
                                   (float) positionOfIntensity[2]);
            scan.push_back(tmpPoint);
        }


        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    return scan;
}


pcl::PointCloud<pcl::PointXYZ> generalHelpfulTools::createPCLFromGraphOnlyThreshold(int indexStart,
                                                               Eigen::Matrix4d transformationInTheEndOfCalculation, graphSlamSaveStructure &usedGraph, double ignoreDistanceToRobo,double thresholdFactorPoint) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    //create array with all intencities.
    // Calculate maximum of intensities.


    double maximumIntensity = 0;
    int i = 0;

    int ignoreDistance = (int) (ignoreDistanceToRobo /
                                (usedGraph.getVertexList()->at(indexStart - i).getIntensities().range /
                                 ((double) usedGraph.getVertexList()->at(
                                         indexStart - i).getIntensities().intensities.size())));


    do {
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                maximumIntensity) {
                maximumIntensity = usedGraph.getVertexList()->at(
                        indexStart - i).getIntensities().intensities[j];
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);

    double thresholdIntensityScan = maximumIntensity * thresholdFactorPoint;//maximum intensity of 0.9



    i = 0;
    do {
        Eigen::Matrix4d transformationOfIntensityRay =
                usedGraph.getVertexList()->at(indexStart).getTransformation().inverse() *
                usedGraph.getVertexList()->at(indexStart - i).getTransformation();

        //positionOfIntensity has to be rotated by   this->graphSaved.getVertexList()->at(indexVertex).getIntensities().angle
        Eigen::Matrix4d rotationOfSonarAngleMatrix = generalHelpfulTools::getTransformationMatrixFromRPY(0, 0,
                                                                                                         usedGraph.getVertexList()->at(
                                                                                                                 indexStart -
                                                                                                                 i).getIntensities().angle);
        for (int j = ignoreDistance;
             j < usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities.size(); j++) {
            if (usedGraph.getVertexList()->at(indexStart - i).getIntensities().intensities[j] >
                thresholdIntensityScan) {
                double distanceOfIntensity =
                        j / ((double) usedGraph.getVertexList()->at(
                                indexStart - i).getIntensities().intensities.size()) *
                        ((double) usedGraph.getVertexList()->at(indexStart - i).getIntensities().range);
                Eigen::Vector4d positionOfIntensity(
                        distanceOfIntensity,
                        0,
                        0,
                        1);
                positionOfIntensity = transformationInTheEndOfCalculation * transformationOfIntensityRay *
                                      rotationOfSonarAngleMatrix * positionOfIntensity;
                //create point for PCL
                pcl::PointXYZ tmpPoint((float) positionOfIntensity[0],
                                       (float) positionOfIntensity[1],
                                       (float) positionOfIntensity[2]);
                scan.push_back(tmpPoint);
            }
        }
        i++;
    } while (usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() != FIRST_ENTRY &&
             usedGraph.getVertexList()->at(indexStart - i).getTypeOfVertex() !=
             INTENSITY_SAVED_AND_KEYFRAME);
    return scan;
}


