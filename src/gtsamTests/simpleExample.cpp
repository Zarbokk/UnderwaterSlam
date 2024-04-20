/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualISAM2Example.cpp
 * @brief   A visualSLAM example for the structure-from-motion problem on a simulated dataset
 * This version uses iSAM2 to solve the problem incrementally
 * @author  Duy-Nguyen Ta
 */

/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 */

// For loading the data
//#include "SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <vector>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {


    // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
    // and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
    // structure is available that allows the user to set various properties, such as the relinearization threshold
    // and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
    // will approach the batch result.
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.print();
    gtsam::ISAM2* isam = new gtsam::ISAM2(parameters);

    // Create a Factor Graph and Values to hold the new data
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values currentEstimate;

    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.01));

    graph.addPrior(0, gtsam::Pose2(0, 0, 0), priorNoise);


    auto model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1, 1, 1));

    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(0, 1, gtsam::Pose2(1.1, 0,0),model);
    currentEstimate.insert(0,gtsam::Pose2(0,0,0));
    currentEstimate.insert(1,gtsam::Pose2(1.5,0,0));

    isam->update(graph,currentEstimate);
    isam->printStats();
    currentEstimate = isam->calculateEstimate();
    for(int i = 0 ;i < currentEstimate.size();i++){
        currentEstimate.at(i).print();
    }
    gtsam::FactorIndices toRemove;
    toRemove.push_back(gtsam::FactorIndex(1));
    std::cout << "update" << std::endl;
    currentEstimate.clear();
    graph.resize(0);


    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2> >(0, 1, gtsam::Pose2(2.1, 0,0),model);




    isam->update(graph,currentEstimate);
    isam->printStats();

    currentEstimate = isam->calculateEstimate();
    for(int i = 0 ;i < currentEstimate.size();i++){
        currentEstimate.at(i).print();
    }


    return 0;
}