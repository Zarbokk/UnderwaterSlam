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
#find_package(PkgConfig REQUIRED)
#pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)
#include_directories(PkgConfig::FFTW)
#link_libraries     (PkgConfig::FFTW)

catkin_package()

################################################################################
# 2D Peak Algorithm
################################################################################

add_library(findpeaks INTERFACE)

target_include_directories(findpeaks INTERFACE
        src/find-peaks/include
        )

add_library(UNION_FIND
        SHARED
        src/find-peaks/src/union_find.cpp
        )
target_include_directories(UNION_FIND PRIVATE
        src/find-peaks/include
        )

target_link_libraries(findpeaks INTERFACE UNION_FIND)

################################################################################
# general interface
################################################################################
add_library(generalinterface INTERFACE)
target_include_directories(generalinterface INTERFACE src/generalInterface/include)

################################################################################
# json
################################################################################
add_library(json SHARED src/json/src/jsoncpp.cpp)
target_include_directories(json PUBLIC src/json/include)


################################################################################
# generalhelpfullib
################################################################################
add_library(generalhelpfullib SHARED src/generalHelpfulTools/src/generalHelpfulTools.cpp src/generalHelpfulTools/src/PeakFinder.cpp)
target_include_directories(generalhelpfullib PUBLIC ${catkin_INCLUDE_DIRS} src/generalHelpfulTools/include )


################################################################################
# posegraph
################################################################################

add_library(posegraph SHARED src/poseGraph/src/edge.cpp src/poseGraph/src/graphSlamSaveStructure.cpp src/poseGraph/src/vertex.cpp)
target_link_libraries(posegraph PUBLIC generalinterface json generalhelpfullib ${Eigen_LIBRARIES})
target_include_directories(posegraph PUBLIC ${catkin_INCLUDE_DIRS} src/poseGraph/include)

################################################################################
# fourier-mellin-registration
################################################################################

add_library(fmt_registration SHARED  src/registration/src/image_registration.cpp src/registration/src/image_transforms.cpp src/registration/src/image_dft.cpp)
target_link_libraries(fmt_registration PUBLIC ${Eigen_LIBRARIES})
target_include_directories(fmt_registration PUBLIC src/registration/include ${catkin_INCLUDE_DIRS})


################################################################################
# scan Registration
################################################################################

add_library(registrationlib SHARED src/registration/src/scanRegistrationClass.cpp src/registration/src/sofftCorrelationClass.cpp src/registration/src/softDescriptorRegistration.cpp)
target_link_libraries(registrationlib PUBLIC ${PCL_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBS} generalhelpfullib gr::algo findpeaks fmt_registration)
target_include_directories(registrationlib PUBLIC src/registration/include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )

################################################################################
# slamtools
################################################################################

add_library(slamtools SHARED  src/slamTools/src/slamToolsRos.cpp )
target_link_libraries(slamtools PUBLIC ${PCL_LIBRARIES} ${Eigen_LIBRARIES} posegraph registrationlib)
target_include_directories(slamtools PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} src/slamTools/include)

################################################################################
# hilbert Map
################################################################################
add_library(hilbertmaps SHARED  src/hilbertMaps/src/hilbertMap.cpp)
target_link_libraries(hilbertmaps PUBLIC ${Eigen_LIBRARIES})
target_include_directories(hilbertmaps PUBLIC src/hilbertMaps/include ${catkin_INCLUDE_DIRS})





################################################################################
# Executables
################################################################################

add_executable(registrationOfTwoPCLICP src/registration/registrationTwoPCLScansIPC.cpp)
target_link_libraries(registrationOfTwoPCLICP gtsam OpenMP::OpenMP_CXX slamtools registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})


#add_executable(registrationOfTwoPCLs src/registration/registrationOfTwoPCLScans.cpp)
#target_link_libraries(registrationOfTwoPCLs OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})
#target_include_directories(registrationOfTwoPCLs PUBLIC ${OpenCV_INCLUDE_DIRS} )
#


#add_executable(comparisonMatchingLiveRosBag src/evaluation/comparisonEveryMatchingMethod.cpp )
#target_link_libraries(comparisonMatchingLiveRosBag ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)

#add_executable(extractingCorrectedDataForRegistration src/evaluation/extractingCorrectedDataForRegistration.cpp )
#target_link_libraries(extractingCorrectedDataForRegistration gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(compareEveryMatchingMethodBasedOnFolder src/evaluation/compareEveryMatchingMethodBasedOnFolder.cpp )
target_link_libraries(compareEveryMatchingMethodBasedOnFolder gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(createImageOfWholeBunker src/evaluation/createImageOfWholeBunker.cpp)
#target_link_libraries(createImageOfWholeBunker ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)
#
#add_executable(continousMapGeneration src/evaluation/continousMapGeneration.cpp )
#target_link_libraries(continousMapGeneration ${catkin_LIBRARIES} ${OpenCV_LIBS} registrationlib OpenMP::OpenMP_CXX fftw3)

add_executable(gtsamExample src/gtsamTests/simpleExample.cpp )
target_link_libraries(gtsamExample gtsam )

#target_include_directories(gtsamExample PUBLIC ${EIGEN3_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})

#add_executable(rosSlamTest src/completeSlamRuns/graphSlamGTSAM.cpp)
#target_link_libraries(rosSlamTest gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})



#add_executable(testSLAM src/completeSlamRuns/testSLAMRUNS.cpp)
#target_link_libraries(testSLAM gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})



add_executable(registrationOfTwoImages src/registration/registrationOfTwoImageScans.cpp )
target_link_libraries(registrationOfTwoImages OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})

add_executable(registrationOfTwoCSV src/registration/registrationTwoImagesCSV.cpp )
target_link_libraries(registrationOfTwoCSV OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})

add_executable(rosslamexp1 src/completeSlamRuns/rosSlamExp1.cpp)
target_link_libraries(rosslamexp1 gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(rosslamexp2 src/completeSlamRuns/rosSlamExp2.cpp)
target_link_libraries(rosslamexp2 gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(randomShit src/completeSlamRuns/completlyFailedMapMatching.cpp)
#target_link_libraries(randomShit gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(caveDatasetSlam src/completeSlamRuns/caveDatasetSLAM.cpp)
target_link_libraries(caveDatasetSlam gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(understandingshit src/registration/understanding3Dso3soft.cpp )
target_link_libraries(understandingshit OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})

add_executable(testFile1 src/testFiles/testFile1.cpp )
target_link_libraries(testFile1 OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})


add_executable(extractionOfData src/evaluation/extractingCorrectedDataForRegistration2_0.cpp)
target_link_libraries(extractionOfData gtsam posegraph slamtools registrationlib OpenMP::OpenMP_CXX fftw3 ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(fourierMellinRegistration src/registration/fourierMellinRegistration.cpp )
target_link_libraries(fourierMellinRegistration OpenMP::OpenMP_CXX registrationlib ${OpenCV_LIBS} fftw3 ${catkin_LIBRARIES})

add_executable(testFeatureRegistration src/registration/testFeatureRegistration.cpp )
target_link_libraries(testFeatureRegistration OpenMP::OpenMP_CXX ${OpenCV_LIBS} generalhelpfullib ${catkin_LIBRARIES})
