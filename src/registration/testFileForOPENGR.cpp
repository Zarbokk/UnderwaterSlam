//
// Created by tim-external on 13.07.22.
//

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/ply_io.h>

#include "gr/algorithms/match4pcsBase.h"
#include "gr/algorithms/FunctorSuper4pcs.h"
#include "gr/utils/geometry.h"
#include <gr/algorithms/PointPairFilter.h>

#include <Eigen/Dense>


int main(int argc, char **argv) {
//    using namespace gr;
//    using namespace std;

    using TrVisitor = gr::DummyTransformVisitor;

    using MatcherType = gr::Match4pcsBase<gr::FunctorSuper4PCS, gr::Point3D<float>, TrVisitor, gr::AdaptivePointFilter, gr::AdaptivePointFilter::Options>;
    using OptionType  = typename MatcherType::OptionsType;
    using SamplerType = gr::UniformDistSampler<gr::Point3D<float> >;

    std::vector<gr::Point3D<float> > set1, set2;
    std::vector<Eigen::Matrix2f> tex_coords1, tex_coords2;
    std::vector<typename gr::Point3D<float>::VectorType> normals1, normals2;
    std::vector<std::string> mtls1, mtls2;

    //fill in set1 and set2

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan1.ply",*scan1);
    pcl::io::loadPLYFile("/home/tim-external/Documents/matlabTestEnvironment/showPointClouds/scan2.ply",*scan2);

    Eigen::Matrix4d transformationX180Degree;
    Eigen::AngleAxisd rotation_vector2(180.0 / 180.0 * 3.14159, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d tmpMatrix3d = rotation_vector2.toRotationMatrix();
    transformationX180Degree.block<3, 3>(0, 0) = tmpMatrix3d;
    transformationX180Degree(3, 3) = 1;

//    pcl::transformPointCloud(*scan1, *scan1, transformationX180Degree);
//    pcl::transformPointCloud(*scan2, *scan2, transformationX180Degree);





    for(int i = 0 ; i<scan1->points.size(); i++){
        gr::Point3D<float> tmpPoint;
        tmpPoint.x() = scan1->points.at(i).x;
        tmpPoint.y() = scan1->points.at(i).y;
        tmpPoint.z() = scan1->points.at(i).z;
        set1.push_back(tmpPoint);
    }
    for(int i = 0 ; i<scan2->points.size(); i++){
        gr::Point3D<float> tmpPoint;
        tmpPoint.x() = scan2->points.at(i).x;
        tmpPoint.y() = scan2->points.at(i).y;
        tmpPoint.z() = scan2->points.at(i).z;
        set2.push_back(tmpPoint);
    }


    // dummy calls, to test symbols accessibility
    // check availability of the Utils functions
    gr::Utils::CleanInvalidNormals(set1, normals1);

    // Our matcher.
    OptionType options;

    // Set parameters.
    typename MatcherType::MatrixType mat;
    double overlap (0.7);
    options.configureOverlap(overlap);


    options.delta = 0.1;
    options.sample_size = 2000;


    typename gr::Point3D<float>::Scalar score = 0;

    constexpr gr::Utils::LogLevel loglvl = gr::Utils::Verbose;
    gr::Utils::Logger logger(loglvl);
    SamplerType sampler;
    TrVisitor visitor;

    MatcherType matcher(options, logger);
    score = matcher.ComputeTransformation(set1, set2, mat, sampler, visitor);

    logger.Log<gr::Utils::Verbose>( "Score: ", score );

    //change to NED

    mat(0,1) = -mat(0,1);
    mat(1,0) = -mat(1,0);
    mat(1,3) = -mat(1,3);


    std::cout << mat << std::endl;
    return 0;
}