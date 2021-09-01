cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
project(underwaterslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg  roscpp pcl_ros soft20 ping360_sonar)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)




set(SOURCE_FILES src/registration/scanRegistrationClass.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/poseGraphsrc/graphSlamSaveStructure.cpp src/poseGraphsrc/edge.cpp src/poseGraphsrc/vertex.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/json/jsoncpp.cpp src/slamTools/slamToolsRos.cpp)
set(SOURCE_FILES ${SOURCE_FILES} include/edge.h include/graphSlamSaveStructure.h include/hilbertMap.h include/json.h include/keyFrame.h include/scanRegistrationClass.h include/slamToolsRos.h include/vertex.h)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    image_sonar_ros_plugin
#  DEPENDS system_lib
)

include_directories(
include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

#add_executable(pcl_sonar_transform ../BlueROV2Common/src/point_cloud_publishing_sonar.cpp)
#target_link_libraries(pcl_sonar_transform ${catkin_LIBRARIES})

#add_executable(create_map src/create_map_with_gt.cpp)
#target_link_libraries(create_map ${catkin_LIBRARIES})

#add_executable(graph_optimization_testing src/graphOptimizationTest.cpp src/graphSlamSaveStructure.cpp src/edge.cpp src/vertex.cpp)
#target_link_libraries(graph_optimization_testing ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(firstCompleteSlamTest src/firstCompleteSlamTest.cpp src/scanRegistrationClass.cpp src/graphSlamSaveStructure.cpp src/edge.cpp src/vertex.cpp src/jsoncpp.cpp)
#target_link_libraries(firstCompleteSlamTest ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(slamLabyrinth src/completeSlamRuns/slamOfLabyrinth01.cpp src/registration/scanRegistrationClass.cpp src/poseGraphsrc/graphSlamSaveStructure.cpp src/poseGraphsrc/edge.cpp src/poseGraphsrc/vertex.cpp src/slamTools/slamToolsRos.cpp src/json/jsoncpp.cpp)
#target_link_libraries(slamLabyrinth ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(correctionPCL src/correctionPCL.cpp src/scanRegistrationClass.cpp src/graphSlamSaveStructure.cpp src/edge.cpp src/vertex.cpp src/slamToolsRos.cpp src/jsoncpp.cpp)
#target_link_libraries(correctionPCL ${PCL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(testFile src/testFiles/testFile.cpp src/registration/scanRegistrationClass.cpp src/poseGraphsrc/graphSlamSaveStructure.cpp src/poseGraphsrc/edge.cpp src/poseGraphsrc/vertex.cpp src/slamTools/slamToolsRos.cpp src/json/jsoncpp.cpp src/hilbertMapssrc/hilbertMap.cpp)
target_link_libraries(testFile ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(soft20TestFile src/registration/soft20TestFile.cpp)
target_link_libraries(soft20TestFile fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(correlationRotationSoft20 src/registration/correlationRotationSoft20.cpp)
target_link_libraries(correlationRotationSoft20 fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(hilbertMapTestFile src/testFiles/hilbertMapExample.cpp src/registration/scanRegistrationClass.cpp src/poseGraphsrc/graphSlamSaveStructure.cpp src/poseGraphsrc/edge.cpp src/poseGraphsrc/vertex.cpp src/slamTools/slamToolsRos.cpp src/json/jsoncpp.cpp src/hilbertMapssrc/hilbertMap.cpp)
target_link_libraries(hilbertMapTestFile ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(stPereDatasetSlam src/completeSlamRuns/stPereDatasetSlam.cpp ${SOURCE_FILES})
#target_link_libraries(stPereDatasetSlam ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(graphSlamWithROS src/completeSlamRuns/graphSlamWithROS.cpp ${SOURCE_FILES})
target_link_libraries(graphSlamWithROS ${PCL_LIBRARIES} ${catkin_LIBRARIES})
