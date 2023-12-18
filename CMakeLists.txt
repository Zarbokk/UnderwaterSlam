cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_GTSAM_USE_SYSTEM_EIGEN ON)


project(underwaterslam)


find_package(ament_cmake REQUIRED)

find_package(rosbag2_cpp REQUIRED)
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
find_package(OpenCV 4.8.0 REQUIRED )
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(gmm_registration REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(micron_driver_ros REQUIRED)
find_package(waterlinked_a50 REQUIRED)
#find_package(perception_oru REQUIRED)

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
set(SOURCE_FILES_ORU
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_d2d.hpp
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_d2d_2d.hpp
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_d2d_feature.hpp
        src/perception_oru/ndt_registration/include/impl/ndt_matcher_p2d.hpp

        src/perception_oru/ndt_map/include/impl/adaptive_oc_tree.hpp
        src/perception_oru/ndt_map/include/impl/cell_vector.hpp
        src/perception_oru/ndt_map/include/impl/lazy_grid.hpp
        src/perception_oru/ndt_map/include/impl/ndt_cell.hpp
        src/perception_oru/ndt_map/include/impl/ndt_histogram.hpp
        src/perception_oru/ndt_map/include/impl/ndt_map.hpp
        src/perception_oru/ndt_map/include/impl/ndt_map_hmt.hpp
        src/perception_oru/ndt_map/include/impl/ndt_occupancy_map.hpp
        src/perception_oru/ndt_map/include/impl/oc_tree.hpp

        src/perception_oru/pointcloud_vrml/include/impl/pointcloud_utils.hpp)

set(INCLUDE_DIRECTORIES_ORU
        ${CMAKE_CURRENT_SOURCE_DIR}/src/perception_oru/ndt_registration/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/perception_oru/ndt_map/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/perception_oru/pointcloud_vrml/include
)

#add_library(perception_oru SHARED
#        #        ndt_map/include
#        ${SOURCE_FILES_ORU}
#)
#
#target_include_directories(perception_oru PUBLIC ${INCLUDE_DIRECTORIES_ORU}
#        ${OpenCV_INCLUDE_DIRS})
#
#set_target_properties(perception_oru PROPERTIES LINKER_LANGUAGE CXX)
#
#target_link_libraries(perception_oru OpenMP::OpenMP_CXX )
#
#ament_target_dependencies(perception_oru PCL)

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

add_library(registrationlib SHARED src/registration/src/scanRegistrationClass.cpp ${SOURCE_FILES_ORU})
ament_target_dependencies(registrationlib Eigen3 rclcpp fs2d cv_bridge tf2 tf2_geometry_msgs GTSAM PCL PythonLibs )
target_include_directories(registrationlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/include ${OpenCV_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${INCLUDE_DIRECTORIES_ORU}) #${PCL_INCLUDE_DIRS}
target_link_libraries(registrationlib generalhelpfullib fmt_registration ${OpenCV_LIBS} ${PCL_LIBRARIES} gmm_registration ) # ${PYTHON_LIBRARIES}


################################################################################
# slamtools
################################################################################

add_library(slamtools SHARED src/slamTools/src/slamToolsRos.cpp)
ament_target_dependencies(slamtools Eigen3 rclcpp visualization_msgs nav_msgs GTSAM)
target_include_directories(slamtools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/slamTools/include)
target_link_libraries(slamtools posegraph registrationlib gmm_registration)
################################################################################
# hilbert Map
################################################################################
#add_library(hilbertmaps SHARED src/hilbertMaps/src/hilbertMap.cpp)
#target_link_libraries(hilbertmaps PUBLIC ${Eigen_LIBRARIES})
#target_include_directories(hilbertmaps PUBLIC src/hilbertMaps/include ${catkin_INCLUDE_DIRS})


################################################################################
# Executables
################################################################################

add_executable(rosSlamTest src/completeSlamRuns/rosSlamExp1.cpp )
ament_target_dependencies(rosSlamTest GTSAM geometry_msgs ping360_sonar_msgs nav_msgs commonbluerovmsg )
target_link_libraries(rosSlamTest ${OpenCV_LIBS} posegraph slamtools generalhelpfullib )
#target_include_directories(rosSlamTest PUBLIC ${INCLUDE_DIRECTORIES_ORU})

add_executable(consecutiveScanTests src/evaluation/consecutiveScansMatchingMethod.cpp )
ament_target_dependencies(consecutiveScanTests GTSAM geometry_msgs ping360_sonar_msgs nav_msgs commonbluerovmsg )
target_link_libraries(consecutiveScanTests ${OpenCV_LIBS} posegraph slamtools generalhelpfullib )

add_executable(extractingGroundTruthGraphData src/evaluation/extractingGraphDataForPlotting.cpp)
ament_target_dependencies(extractingGroundTruthGraphData GTSAM geometry_msgs ping360_sonar_msgs nav_msgs commonbluerovmsg mocap_msgs)
target_link_libraries(extractingGroundTruthGraphData ${OpenCV_LIBS} posegraph slamtools generalhelpfullib)



add_executable(speedComparisonMatchingMethods src/evaluation/compareEveryMatchingMethodBasedOnFolder.cpp )
#ament_target_dependencies(speedComparisonMatchingMethods )
target_link_libraries(speedComparisonMatchingMethods ${OpenCV_LIBS} registrationlib generalhelpfullib slamtools)

add_executable(rosbagReading src/evaluation/testRosbagReading.cpp )
#ament_target_dependencies(speedComparisonMatchingMethods )
ament_target_dependencies(rosbagReading Eigen3 rclcpp cv_bridge tf2 rosbag2_cpp tf2_geometry_msgs ping360_sonar_msgs px4_msgs mocap_msgs micron_driver_ros waterlinked_a50)

add_executable(readInCSV src/evaluation/readInCSVAndPublishToTopics.cpp )
ament_target_dependencies(readInCSV Eigen3 rclcpp tf2 ping360_sonar_msgs )

ament_package()