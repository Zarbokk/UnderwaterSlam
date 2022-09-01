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
find_package( catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg roscpp pcl_ros soft20 ping360_sonar waterlinked_dvl message_generation cv_bridge commonbluerovmsg dynamic_reconfigure perception_oru)
find_package( Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )
find_package( PCL 1.10 REQUIRED COMPONENTS common io visualization )
find_package( OpenGR REQUIRED )
find_package( OpenMP REQUIRED )
# add_service_files(
#   FILES
#   resetekf.srv
#   saveGraph.srv
#   Service1.srv
#   Service2.srv
# )


generate_messages(
        DEPENDENCIES
        sensor_msgs
        std_msgs
)

set(SOURCE_FILES src/registration/scanRegistrationClass.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/poseGraph/graphSlamSaveStructure.cpp src/poseGraph/edge.cpp src/poseGraph/vertex.cpp)
set(SOURCE_FILES ${SOURCE_FILES} src/json/jsoncpp.cpp src/slamTools/slamToolsRos.cpp src/slamTools/generalHelpfulTools.cpp)
set(SOURCE_FILES ${SOURCE_FILES}   src/hilbertMapssrc/hilbertMap.cpp )
set(SOURCE_FILES ${SOURCE_FILES}  src/slamTools/PeakFinder.cpp  src/registration/softDescriptorRegistration.cpp)
set(SOURCE_FILES ${SOURCE_FILES}  src/registration/sofftCorrelationClass.cpp )
set(SOURCE_FILES ${SOURCE_FILES} src/registration/FMSOld/image_registration.cpp src/registration/FMSOld/image_transforms.cpp src/registration/FMSOld/image_dft.cpp)

set(SOURCE_FILES_GENERAL_TOOLS  src/slamTools/slamToolsRos.cpp  src/slamTools/generalHelpfulTools.cpp)
# ${SOURCE_FILES_GENERAL_TOOLS}

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS message_runtime
        #image_sonar_ros_plugin
        #DEPENDS system_lib
)

include_directories(
        include
        src/registration/FMSOld
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${OpenGR_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(stPereDatasetRosPub src/stPereDataset/stPereDatasetPublication.cpp  src/slamTools/generalHelpfulTools.cpp)
#target_link_libraries(stPereDatasetRosPub ${catkin_LIBRARIES})

#add_executable(graphSlamIntensitiesOnlyTest src/completeSlamRuns/graphSlam_W_E_E_IntensitiesOnly.cpp ${SOURCE_FILES})
#target_link_libraries(graphSlamIntensitiesOnlyTest fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(registrationOfTwoPCLs src/registration/registrationOfTwoPCLScans.cpp ${SOURCE_FILES})
target_link_libraries(registrationOfTwoPCLs fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(registrationOfTwoPCLICP src/registration/registrationTwoPCLScansIPC.cpp ${SOURCE_FILES})
target_link_libraries(registrationOfTwoPCLICP fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(testGraphOptimization src/testFiles/graphOptimizationTest.cpp ${SOURCE_FILES})
#target_link_libraries(testGraphOptimization fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(2DOurMethod src/evaluation/2DOurMethodTesting.cpp ${SOURCE_FILES})
#target_link_libraries(2DOurMethod gr::algo fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(registrationOfTwoImages src/registration/registrationOfTwoImageScans.cpp ${SOURCE_FILES})
target_link_libraries(registrationOfTwoImages fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(testGR src/registration/testFileForOPENGR.cpp ${SOURCE_FILES})
#target_link_libraries(testGR gr::algo fftw3 ${PCL_LIBRARIES} ${catkin_LIBRARIES})



#add_executable(testFMSOld src/registration/FMSOld/image_main.cpp  ${SOURCE_FILES})
#target_link_libraries(testFMSOld fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(ndtTests src/registration/ndtMatching/simple.cc  )
target_link_libraries(ndtTests  ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(comparisonMatchingLiveRosBag src/evaluation/comparisonEveryMatchingMethod.cpp ${SOURCE_FILES})
target_link_libraries(comparisonMatchingLiveRosBag gr::algo fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(extractingCorrectedDataForRegistration src/evaluation/extractingCorrectedDataForRegistration.cpp ${SOURCE_FILES})
target_link_libraries(extractingCorrectedDataForRegistration gr::algo fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(compareEveryMatchingMethodBasedOnFolder src/evaluation/compareEveryMatchingMethodBasedOnFolder.cpp ${SOURCE_FILES})
target_link_libraries(compareEveryMatchingMethodBasedOnFolder gr::algo fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(createImageOfWholeBunker src/evaluation/createImageOfWholeBunker.cpp ${SOURCE_FILES})
target_link_libraries(createImageOfWholeBunker gr::algo fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(createImageSimulation src/evaluation/createImageSimulation.cpp ${SOURCE_FILES})
#target_link_libraries(createImageSimulation gr::algo fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(testExtraction src/evaluation/testExtraction.cpp ${SOURCE_FILES})
#target_link_libraries(testExtraction gr::algo fftw3 OpenMP::OpenMP_CXX ${PCL_LIBRARIES} ${catkin_LIBRARIES})


