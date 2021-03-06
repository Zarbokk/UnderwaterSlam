cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CATKIN_ENABLE_TESTING OFF)
project(underwaterslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg  roscpp pcl_ros soft20 ping360_sonar waterlinked_dvl message_generation cv_bridge commonbluerovmsg dynamic_reconfigure)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)

# add_service_files(
#   FILES
#   resetekf.srv
#   saveGraph.srv
#   Service1.srv
#   Service2.srv
# )
generate_dynamic_reconfigure_options(
        cfg/ekfnoise.cfg
)


#generate_messages(
#        DEPENDENCIES
#        sensor_msgs
#        std_msgs
#)

set(SOURCE_FILES src/registration/scanRegistrationClass.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/poseGraphsrc/graphSlamSaveStructure.cpp src/poseGraphsrc/edge.cpp src/poseGraphsrc/vertex.cpp src/EKF/ekfPos.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/json/jsoncpp.cpp src/slamTools/slamToolsRos.cpp src/slamTools/generalHelpfulTools.cpp)
set(SOURCE_FILES ${SOURCE_FILES} include/edge.h include/graphSlamSaveStructure.h src/hilbertMapssrc/hilbertMap.cpp include/hilbertMap.h include/json.h include/keyFrame.h include/scanRegistrationClass.h include/slamToolsRos.h include/vertex.h src/EKF/ekf.h include/generalHelpfulTools.h src/EKF/ekfDVL.h)
set(SOURCE_FILES ${SOURCE_FILES} include/PeakFinder.h src/slamTools/PeakFinder.cpp include/softDescriptorRegistration.h src/registration/softDescriptorRegistration.cpp)
set(SOURCE_FILES ${SOURCE_FILES} include/sofftCorrelationClass.h src/registration/sofftCorrelationClass.cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS message_runtime
    #image_sonar_ros_plugin
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

#add_executable(compensationSonarData src/movementCorrection/compensationSonarDataWithEKF.cpp)
#target_link_libraries(compensationSonarData fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(softClassTest src/registration/testingOfSoftDescriptorClass.cpp ${SOURCE_FILES})
#target_link_libraries(softClassTest fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(stPereDatasetPublishingIntensities src/testFiles/stPereDatasetPublication.cpp ${SOURCE_FILES})
#target_link_libraries(stPereDatasetPublishingIntensities fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(correctionPCL src/movementCorrection/correctionPCLSTPere.cpp ${SOURCE_FILES})
#target_link_libraries(correctionPCL fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_executable(testFile src/testFiles/testFile.cpp ${SOURCE_FILES})
#target_link_libraries(testFile fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(soft20TestFile src/registration/soft20TestFile.cpp)
#target_link_libraries(soft20TestFile fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(correlationRotationSoft20 src/registration/correlationRotationSoft20.cpp include/PeakFinder.h src/slamTools/PeakFinder.cpp)
#target_link_libraries(correlationRotationSoft20 fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(hilbertMapTestFile src/testFiles/hilbertMapExample.cpp ${SOURCE_FILES})
#target_link_libraries(hilbertMapTestFile fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_executable(graphSlamWithROS src/completeSlamRuns/graphSlamWithROS.cpp ${SOURCE_FILES})
#target_link_libraries(graphSlamWithROS ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(testEKF src/EKF/testEKF.cpp ${SOURCE_FILES} include/pose.h)
#target_link_libraries(testEKF fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(ekfGraphSlam src/completeSlamRuns/ekfGraphSlam.cpp ${SOURCE_FILES})
#target_link_libraries(ekfGraphSlam fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})



add_executable(stPereDatasetRosPub src/stPereDataset/stPereDatasetPublishingRos.cpp ${SOURCE_FILES})
target_link_libraries(stPereDatasetRosPub fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(graphSlamExternalEstimator src/completeSlamRuns/graphSlamWithExternalStateEstimator.cpp ${SOURCE_FILES})
target_link_libraries(graphSlamExternalEstimator fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(EKFDVL src/EKF/ekfDVLnode.cpp ${SOURCE_FILES} include/pose.h src/EKF/ekfDVL.cpp)
target_link_libraries(EKFDVL fftw3 ${catkin_LIBRARIES})

add_executable(registrationOfTwoPCLs src/registration/registrationOfTwoScans.cpp ${SOURCE_FILES})
target_link_libraries(registrationOfTwoPCLs fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(registrationOfTwoPCLICP src/registration/registrationTwoScansIPC.cpp ${SOURCE_FILES})
target_link_libraries(registrationOfTwoPCLICP fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
