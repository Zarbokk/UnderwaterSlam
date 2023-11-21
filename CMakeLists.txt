cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_GTSAM_USE_SYSTEM_EIGEN ON)


project(underwaterslam)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(commonbluerovmsg REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)
#find_package(OpenGR REQUIRED)
#find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(fs2d REQUIRED)
find_package(OpenCV 4.7 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(mocap_msgs)
find_package(gmm_registration)
################################################################################
# 2D Peak Algorithm
################################################################################

#add_library(findpeaks INTERFACE)
#
#target_include_directories(findpeaks INTERFACE
#        src/find-peaks/include
#        )
#
#add_library(UNION_FIND
#        SHARED
#        src/find-peaks/src/union_find.cpp
#        )
#target_include_directories(UNION_FIND PRIVATE
#        src/find-peaks/include
#        )

#target_link_libraries(findpeaks INTERFACE UNION_FIND)

################################################################################
# general interface
################################################################################
add_library(generalinterface INTERFACE)
target_include_directories(generalinterface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/generalInterface/include)

################################################################################
# json
################################################################################
add_library(json SHARED src/json/src/jsoncpp.cpp)
target_include_directories(json PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/json/include)


################################################################################
# generalhelpfullib
################################################################################
add_library(generalhelpfullib SHARED src/generalHelpfulTools/src/generalHelpfulTools.cpp src/generalHelpfulTools/src/PeakFinder.cpp)
ament_target_dependencies(generalhelpfullib tf2 Eigen3 GTSAM)
target_include_directories(generalhelpfullib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/generalHelpfulTools/include)


################################################################################
# posegraph
################################################################################

add_library(posegraph SHARED src/poseGraph/src/edge.cpp src/poseGraph/src/graphSlamSaveStructure.cpp src/poseGraph/src/vertex.cpp)
ament_target_dependencies(posegraph PUBLIC Eigen3 GTSAM)
target_include_directories(posegraph PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/poseGraph/include)
target_link_libraries(posegraph PUBLIC generalinterface json generalhelpfullib gtsam)

################################################################################
# fourier-mellin-registration
################################################################################

add_library(fmt_registration SHARED src/registration/src/image_registration.cpp src/registration/src/image_transforms.cpp src/registration/src/image_dft.cpp)
ament_target_dependencies(fmt_registration PUBLIC Eigen3)
target_link_libraries(fmt_registration PUBLIC ${OpenCV_LIBS} fftw3)
target_include_directories(fmt_registration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/include ${OpenCV_INCLUDE_DIRS}) # ${catkin_INCLUDE_DIRS}


################################################################################
# NDT Stuff
################################################################################

add_library(perception_oru SHARED
        #        ndt_map/include
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_d2d.cpp
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_d2d_2d.cpp
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_d2d_feature.cpp
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_p2d.cpp
        src/perception_oru/ndt_map/include/impl/adaptive_oc_tree.cpp
        src/perception_oru/ndt_map/include/impl/cell_vector.cpp
        src/perception_oru/ndt_map/include/impl/EventCounterData.cpp
        src/perception_oru/ndt_map/include/impl/lazy_grid.cpp
        src/perception_oru/ndt_map/include/impl/ndt_cell.cpp
        src/perception_oru/ndt_map/include/impl/ndt_histogram.cpp
        src/perception_oru/ndt_map/include/impl/ndt_map.cpp
        src/perception_oru/ndt_map/include/impl/ndt_map_hmt.cpp
        src/perception_oru/ndt_map/include/impl/ndt_occupancy_map.cpp
        src/perception_oru/ndt_map/include/impl/oc_tree.cpp

        src/perception_oru/pointcloud_vrml/include/impl/pointcloud_utils.cpp
)

target_include_directories(perception_oru PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/perception_oru/ndt_map/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/perception_oru/ndt_registration/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/perception_oru/pointcloud_vrml/include
        ${OpenCV_INCLUDE_DIRS})

set_target_properties(perception_oru PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(perception_oru OpenMP::OpenMP_CXX )

ament_target_dependencies(perception_oru PCL)

#install(DIRECTORY src/perception_oru/ndt_map/include  src/perception_oru/ndt_registration/include src/perception_oru/pointcloud_vrml/include
#        DESTINATION include)

#install(
#        TARGETS perception_oru
#        EXPORT ${PROJECT_NAME}
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib
#        RUNTIME DESTINATION bin
#        INCLUDES DESTINATION include
#)




#ament_export_targets(perception_oru HAS_LIBRARY_TARGET)

################################################################################
# scan Registration
################################################################################

add_library(registrationlib SHARED src/registration/src/scanRegistrationClass.cpp)
ament_target_dependencies(registrationlib Eigen3 rclcpp fs2d cv_bridge tf2 tf2_geometry_msgs GTSAM PCL PythonLibs)
target_include_directories(registrationlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/include ${OpenCV_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} perception_oru) #${PCL_INCLUDE_DIRS}
target_link_libraries(registrationlib generalhelpfullib fmt_registration ${OpenCV_LIBS} ${PCL_LIBRARIES} gmm_registration perception_oru)# ${PYTHON_LIBRARIES}


################################################################################
# slamtools
################################################################################

add_library(slamtools SHARED src/slamTools/src/slamToolsRos.cpp)
ament_target_dependencies(slamtools Eigen3 rclcpp visualization_msgs nav_msgs GTSAM)
target_include_directories(slamtools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/slamTools/include)
target_link_libraries(slamtools posegraph registrationlib)
################################################################################
# hilbert Map
################################################################################
#add_library(hilbertmaps SHARED src/hilbertMaps/src/hilbertMap.cpp)
#target_link_libraries(hilbertmaps PUBLIC ${Eigen_LIBRARIES})
#target_include_directories(hilbertmaps PUBLIC src/hilbertMaps/include ${catkin_INCLUDE_DIRS})


################################################################################
# Executables
################################################################################

#add_executable(registrationOfTwoPCLICP src/registration/registrationTwoPCLScansIPC.cpp)
#target_link_libraries(registrationOfTwoPCLICP gtsam OpenMP::OpenMP_CXX slamtools registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})


#add_executable(registrationOfTwoPCLs src/registration/registrationOfTwoPCLScans.cpp)
#target_link_libraries(registrationOfTwoPCLs OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#target_include_directories(registrationOfTwoPCLs PUBLIC ${OpenCV_INCLUDE_DIRS} )
#


#add_executable(comparisonMatchingLiveRosBag src/evaluation/comparisonEveryMatchingMethod.cpp )
#target_link_libraries(comparisonMatchingLiveRosBag ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)

#add_executable(extractingCorrectedDataForRegistration src/evaluation/extractingCorrectedDataForRegistration.cpp )
#target_link_libraries(extractingCorrectedDataForRegistration gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(compareEveryMatchingMethodBasedOnFolder src/evaluation/compareEveryMatchingMethodBasedOnFolder.cpp)
#target_link_libraries(compareEveryMatchingMethodBasedOnFolder gtsam posegraph slamtools registrationlib fmt_registration OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS} gmm_registration)

#add_executable(createImageOfWholeBunker src/evaluation/createImageOfWholeBunker.cpp)
#target_link_libraries(createImageOfWholeBunker ${catkin_LIBRARIES} ${OpenCV_LIBS} slamtools registrationlib OpenMP::OpenMP_CXX fftw3)

#add_executable(createImageOfWholeBunker2 src/evaluation/createImageOfWholeBunker2.cpp)
#target_link_libraries(createImageOfWholeBunker2 ${catkin_LIBRARIES} ${OpenCV_LIBS} gtsam slamtools registrationlib OpenMP::OpenMP_CXX fftw3)

#add_executable(continousMapGeneration src/evaluation/continousMapGeneration.cpp )
#target_link_libraries(continousMapGeneration ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)

#add_executable(gtsamExample src/gtsamTests/simpleExample.cpp)
#target_link_libraries(gtsamExample gtsam)

#target_include_directories(gtsamExample PUBLIC ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})


add_executable(rosSlamTest src/completeSlamRuns/rosSlamExp1.cpp)
ament_target_dependencies(rosSlamTest GTSAM geometry_msgs ping360_sonar_msgs nav_msgs commonbluerovmsg )
target_link_libraries(rosSlamTest ${OpenCV_LIBS} posegraph slamtools generalhelpfullib registrationlib)



add_executable(extractingGroundTruthGraphData src/evaluation/extractingGraphDataForPlotting.cpp)
ament_target_dependencies(extractingGroundTruthGraphData GTSAM geometry_msgs ping360_sonar_msgs nav_msgs commonbluerovmsg mocap_msgs)
target_link_libraries(extractingGroundTruthGraphData ${OpenCV_LIBS} posegraph slamtools generalhelpfullib)

#add_executable(testSLAM src/completeSlamRuns/testSLAMRUNS.cpp)
#target_link_libraries(testSLAM gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})


#add_executable(registrationOfTwoImages src/registration/registrationOfTwoImageScans.cpp)
#target_link_libraries(registrationOfTwoImages OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#
#add_executable(registrationOfTwoCSV src/registration/registrationTwoImagesCSV.cpp)
#target_link_libraries(registrationOfTwoCSV OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#
#add_executable(rosslamexp1 src/completeSlamRuns/rosSlamExp1.cpp)
#target_link_libraries(rosslamexp1 gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(rosslamexp2 src/completeSlamRuns/rosSlamExp2.cpp)
#target_link_libraries(rosslamexp2 gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(rosslamexp3 src/completeSlamRuns/rosSlamExp3.cpp)
#target_link_libraries(rosslamexp3 gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
##add_executable(randomShit src/completeSlamRuns/completlyFailedMapMatching.cpp)
##target_link_libraries(randomShit gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
##add_executable(caveDatasetSlam src/completeSlamRuns/caveDatasetSLAM.cpp)
##target_link_libraries(caveDatasetSlam gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(understandingshit src/registration/understanding3Dso3soft.cpp)
#target_link_libraries(understandingshit OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#
#add_executable(testFile1 src/testFiles/testFile1.cpp)
#target_link_libraries(testFile1 OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} gmm_registration)
#
#add_executable(testFile2 src/testFiles/testFile2.cpp)
#target_link_libraries(testFile2 OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} gmm_registration)
#
#add_executable(extractionOfData src/evaluation/extractingCorrectedDataForRegistration2_0.cpp)
#target_link_libraries(extractionOfData gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#
#add_executable(fourierMellinRegistration src/registration/fourierMellinRegistration.cpp)
#target_link_libraries(fourierMellinRegistration OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#
#add_executable(testFeatureRegistration src/registration/testFeatureRegistration.cpp)
#target_link_libraries(testFeatureRegistration OpenMP::OpenMP_CXX ${OpenCV_LIBS} generalhelpfullib ${catkin_LIBRARIES})
#
#add_executable(consecutiveScansSlam src/evaluation/consecutiveScansSLAMCalc.cpp)
#target_link_libraries(consecutiveScansSlam gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(consecutiveScansRegistration src/evaluation/consecutiveScansMatchingMethod.cpp)
#target_link_libraries(consecutiveScansRegistration gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#
#add_executable(createMapOfDataSet src/evaluation/continousMapGeneration.cpp)
#target_link_libraries(createMapOfDataSet gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})


ament_package()