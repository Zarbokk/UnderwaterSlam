/*
 ********************************************************************
 * This file is:                                                    *
 *     2022 Pau Vial @ VICOROB-UdG, Girona, Catalonia               *
 *     2022 Miguel Malagon @ VICOROB-UdG, Girona, Catalonia         *
 *                                                                  *
 * This file is part of GmmRegistration' library, a C++ library     *
 * for acoustic point cloud registration for robotic perception.    *
 *                                                                  *
 * GmmRegistration is free software: you can redistribute it and/or *
 * modify it under the terms of the GNU General Public License as   *
 * published by the Free Software Foundation, either version 3 of   *
 * the License, or (at your option) any later version.              *
 *                                                                  *
 * GmmRegistration is distributed in the hope that it will be       *
 * useful, but WITHOUT ANY WARRANTY; without even the implied       *
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. *
 * See the GNU General Public License for more details.             *
 *                                                                  *
 * You should have received a copy of the GNU General Public        *
 * License along with this program.  If not, see                    *
 * <https://www.gnu.org/licenses/>.                                 *
 *                                                                  *
 * GmmRegistration is:                                              *
 *     2022 Pau Vial @ Institut VICOROB                             *
 *     Universitat de Girona                                        *
 *     Girona, Catalonia                                            *
 *     Copyright (c) 2021-2023 Pau Vial. All rights reserved.       *
 ********************************************************************
 */

/**
 * @file example_p2d_2d.cpp
 * @brief Two-dimensional example for the Points to Distribution point cloud registration method
 * @author Pau Vial
 */
/**
 * A simple 2D Points to Distribution registration example
 *  - The scan is a line generated with random noise
 *  - We give an specific transformation in order to have a ground truth
 *  - The gmm are generated with ndt_constructor giving 2 units cell size and a minimum of 3 points per component
 *  - The components are corrected to have a 0.1 minimum ratio between covvariance eigen values
 *  - We solve using the CholeskyLineSearchNewtonMethod solver with its default parameters
 */

#include <gmm_registration/eigen_fix.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <vector>
#include <time.h>
#include <iostream>

#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/front_end/GaussianMixturesModel.h>
#include <gmm_registration/method/ScanMatchingMethods.h>
#include <gmm_registration/method/PointsToDistribution2D.h>
#include <gmm_registration/solver/Solver.h>
#include <gmm_registration/solver/CholeskyLineSearchNewtonMethod.h>

using namespace std;

int main(int argc, char** argv)
{
    // Ground truth transformation to apply on the scan as (translation_x,translation_y,rotation)
    Eigen::Vector3d t1;
    // t1 << 0.1, 0.25, -0.15;
    t1 << 0.8, -0.6, 0.3;
    Eigen::Matrix3d T = se_exp_map(t1);

    //////////////////
    // IMPORT SCAN

    // Go through the file with the scan to get its length
    string path = std::getenv("GMM_REGISTRATION_PATH");
    string scan_file = path + "/data/scan_5.xyz";
    ifstream fin(scan_file);
    double x, y, z;
    int len = 0;
    while (fin >> x >> y >> z)
    {
        len++;
    }

    // Read scan from file
    vector<Eigen::Vector2d> current_scan(len);
    vector<Eigen::Vector2d> reference_scan(current_scan.size());
    int i = 0;
    ifstream fin2(scan_file);
    while (fin2 >> x >> y >> z)
    {
        current_scan[i] << x, y;
        reference_scan[i] = (T * (Eigen::Vector3d() << current_scan[i](0), current_scan[i](1), 1).finished()).head(2);
        i++;
    }

    /*
    //////////////////
    // GENERATE SYNTHETIC SCAN

    // Generation of the current scan as two lines with noise forming a 90 degree corner
    srand((unsigned)time(NULL));
    vector<Eigen::Vector2d> current_scan(40);
    vector<Eigen::Vector2d> reference_scan(current_scan.size());
    for(int i = 0; i<current_scan.size(); i++){
        current_scan[i] << 0.25*i + 0.1*(double)rand()/RAND_MAX - 0.05, 1 + 0.1*(double)rand()/RAND_MAX - 0.05;
        if(i>=20) current_scan[i] <<  0.1*(double)rand()/RAND_MAX - 0.05, 0.25*(i-19) + 1 + 0.1*(double)rand()/RAND_MAX -
    0.05; reference_scan[i] = rot * current_scan[i] + t1.head(2);
    }
    */

    //////////////////
    // REGISTER

    // Generate a GMM out of the reference scan
    shared_ptr<GaussianMixturesModel<2>> gmm;
    // gmm = ndt_constructor(reference_scan,3,3);
    // gmm = k_means_constructor(reference_scan,4);
    // gmm = em_constructor(reference_scan,4);
    gmm = bayesian_gmm_constructor(reference_scan, 10);
    gmm->balance_covariances(0.05);
    gmm->plot_components_density(0, reference_scan, true);

    // Set both the GMM and the scan to a P2D method
    shared_ptr<PointsToDistribution2D> method(
            new PointsToDistribution2D(gmm, make_shared<std::vector<Eigen::Vector2d>>(current_scan)));
    // method->set_cpu_threads(6);

    // Set the method to the solver with its needed parameters
    unique_ptr<CholeskyLineSearchNewtonMethod<3>> solver(new CholeskyLineSearchNewtonMethod<3>(method));

    // Solve the registration problem starting with a zero seed
    solver->compute_optimum();
    solver->plot_process(0, true);
    Eigen::Vector3d t_opt = solver->get_optimal();
    Eigen::Matrix3d h_opt = solver->get_optimal_uncertainty();

    gmm.reset();
    method.reset();
    solver.reset();
}