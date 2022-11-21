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
find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg roscpp pcl_ros soft20 ping360_sonar waterlinked_dvl message_generation cv_bridge commonbluerovmsg dynamic_reconfigure perception_oru tf2 tf2_ros )
find_package( Boost REQUIRED COMPONENTS system )
find_package( OpenCV REQUIRED )
find_package( PCL 1.10 REQUIRED COMPONENTS common io visualization )
find_package( OpenGR REQUIRED )
find_package( OpenMP REQUIRED )
find_package( GTSAM REQUIRED )
find_package( Eigen3 3.3 REQUIRED )



################################################################################
# general interface
################################################################################
add_library(generalinterface INTERFACE)
target_include_directories(generalinterface INTERFACE src/generalInterface/include)

################################################################################
# json
################################################################################
add_library(json STATIC src/json/src/jsoncpp.cpp)
target_include_directories(json PUBLIC src/json/include)


################################################################################
# generalhelpfullib
################################################################################
add_library(generalhelpfullib STATIC src/generalHelpfulTools/src/generalHelpfulTools.cpp src/generalHelpfulTools/src/PeakFinder.cpp)
target_include_directories(generalhelpfullib PUBLIC ${catkin_INCLUDE_DIRS} src/generalHelpfulTools/include )


################################################################################
# posegraph
################################################################################

add_library(posegraph STATIC src/poseGraph/src/edge.cpp src/poseGraph/src/graphSlamSaveStructure.cpp src/poseGraph/src/vertex.cpp)
target_link_libraries(posegraph PUBLIC generalinterface json generalhelpfullib ${Eigen_LIBRARIES})
target_include_directories(posegraph PUBLIC ${catkin_INCLUDE_DIRS} src/poseGraph/include)

################################################################################
# slamtools
################################################################################

add_library(slamtools STATIC  src/slamTools/src/slamToolsRos.cpp )
target_link_libraries(slamtools PUBLIC ${PCL_LIBRARIES} ${Eigen_LIBRARIES} posegraph)
target_include_directories(slamtools PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src/slamTools/include)


################################################################################
# scan Registration
################################################################################

add_library(registrationlib STATIC src/registration/src/scanRegistrationClass.cpp src/registration/src/sofftCorrelationClass.cpp src/registration/src/softDescriptorRegistration.cpp)
target_link_libraries(registrationlib PUBLIC ${PCL_LIBRARIES} ${Eigen_LIBRARIES} slamtools gr::algo)
target_include_directories(registrationlib PUBLIC src/registration/include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


################################################################################
# hilbert Map
################################################################################
add_library(hilbertmaps STATIC  src/hilbertMaps/src/hilbertMap.cpp)
target_link_libraries(hilbertmaps PUBLIC ${Eigen_LIBRARIES})
target_include_directories(hilbertmaps PUBLIC src/hilbertMaps/include ${catkin_INCLUDE_DIRS})






################################################################################
# Executables
################################################################################

#add_executable(registrationOfTwoPCLICP src/registration/registrationTwoPCLScansIPC.cpp)
#target_link_libraries(registrationOfTwoPCLICP OpenMP::OpenMP_CXX slamtools registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#target_include_directories(registrationOfTwoPCLICP PUBLIC ${OpenCV_INCLUDE_DIRS} )
#
#add_executable(registrationOfTwoPCLs src/registration/registrationOfTwoPCLScans.cpp)
#target_link_libraries(registrationOfTwoPCLs OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#target_include_directories(registrationOfTwoPCLs PUBLIC ${OpenCV_INCLUDE_DIRS} )
#
#add_executable(registrationOfTwoImages src/registration/registrationOfTwoImageScans.cpp )
#target_link_libraries(registrationOfTwoImages OpenMP::OpenMP_CXX slamtools registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})

#add_executable(comparisonMatchingLiveRosBag src/evaluation/comparisonEveryMatchingMethod.cpp )
#target_link_libraries(comparisonMatchingLiveRosBag ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)

#add_executable(extractingCorrectedDataForRegistration src/evaluation/extractingCorrectedDataForRegistration.cpp )
#target_link_libraries(extractingCorrectedDataForRegistration ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)

#add_executable(compareEveryMatchingMethodBasedOnFolder src/evaluation/compareEveryMatchingMethodBasedOnFolder.cpp )
#target_link_libraries(compareEveryMatchingMethodBasedOnFolder ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)
#
#add_executable(createImageOfWholeBunker src/evaluation/createImageOfWholeBunker.cpp)
#target_link_libraries(createImageOfWholeBunker ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)
#
#add_executable(continousMapGeneration src/evaluation/continousMapGeneration.cpp )
#target_link_libraries(continousMapGeneration ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)

add_executable(gtsamExample src/gtsamTests/simpleExample.cpp )
target_link_libraries(gtsamExample gtsam )
#target_include_directories(gtsamExample PUBLIC ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})

add_executable(rosSlamTest src/completeSlamRuns/graphSlamGTSAM.cpp)
target_link_libraries(rosSlamTest gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_include_directories(rosSlamTest PUBLIC ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})