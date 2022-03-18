//
// Created by tim-linux on 18.03.22.
//

#ifndef UNDERWATERSLAM_GRAPHOVERALLHEADER_H
#define UNDERWATERSLAM_GRAPHOVERALLHEADER_H


#define POINT_CLOUD_SAVED 0
#define INTEGRATED_POSE 1
#define FIRST_ENTRY 2
#define INTENSITY_SAVED 3
#define INTENSITY_BASED_GRAPH 4
#define POINT_CLOUD_BASED_GRAPH 5
#define ONLY_SIMPLE_GRAPH 5


struct intensityMeasurement {
    double time;
    double angle;//in rad
    double increment;
    int size;
    std::vector<double> intensities;
};




#endif //UNDERWATERSLAM_GRAPHOVERALLHEADER_H
