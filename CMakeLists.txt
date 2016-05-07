cmake_minimum_required(VERSION 2.8.3)
project(mcptam)

# Add cmake_modules directory to cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

# Define build type
set(CMAKE_BUILD_TYPE None)

# Set our own specific flags. Want something like RelWithDebInfo but without the
# -DNDEBUG flag that disables asserts
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -march=native -msse3 -O3 -g -fopenmp")

# Find various packages in order to set up environment variables
find_package(catkin REQUIRED COMPONENTS roscpp image_transport sensor_msgs cv_bridge geometry_msgs tf message_filters std_msgs
                                        std_srvs visualization_msgs dynamic_reconfigure pcl_ros message_generation roslint)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(CVD REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(GVars3 REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

message(STATUS "G2O_CORE_LIBRARY=${G2O_CORE_LIBRARY}")

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${SUITESPARSE_INCLUDE_DIRS}
                    ${CVD_INCLUDE_DIR}
                    ${GVars3_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${SUITESPARSE_LIBRARY_DIRS}
                 ${OpenCV_LIB_DIR}
                 ${LAPACK_LIBRARIES}
                 ${BLAS_LIBRARIES}
)

add_message_files(
  DIRECTORY msg
  FILES MapInfo.msg
        MapMakerTiming.msg
        NetworkMapPoint.msg
        NetworkMeasurement.msg
        NetworkKeyFrame.msg
        NetworkMultiKeyFrame.msg
        NetworkOutlier.msg
        SystemInfo.msg
        TrackerTiming.msg
        TrackerState.msg
)

add_service_files( DIRECTORY srv
                   FILES
                   ModifyMap.srv
                   Reset.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} $
  CATKIN_DEPENDS roscpp
                 image_transport
                 sensor_msgs
                 cv_bridge geometry_msgs
                 tf
                 message_filters
                 std_msgs
                 std_srvs
                 visualization_msgs
                 dynamic_reconfigure
                 pcl_ros
                 message_runtime
  DEPENDS SuiteSparse
          CVD
          OpenGL
          GVars3
)

# Check source for formatting
roslint_cpp()

# Tests
roslint_add_test()

set(MCPTAM_FUNCTIONS_LIB_SRC
    src/BundleAdjusterBase.cpp
    src/BundleAdjusterCalib.cpp
    src/BundleAdjusterMulti.cpp
    src/BundleAdjusterSingle.cpp
    src/CalibCornerPatch.cpp
    src/CalibGridCorner.cpp
    src/CalibImageTaylor.cpp
    src/CameraCalibrator.cpp
    src/CameraGroupSubscriber.cpp
    src/ChainBundle.cpp
    src/GLWindow2.cpp
    src/GLWindowMenu.cpp
    src/KeyFrame.cpp
    src/KeyFrameViewer.cpp
    src/Map.cpp
    src/MapMaker.cpp
    src/MapMakerBase.cpp
    src/MapMakerCalib.cpp
    src/MapMakerClient.cpp
    src/MapMakerClientBase.cpp
    src/MapMakerServer.cpp
    src/MapMakerServerBase.cpp
    src/MapPoint.cpp
    src/MiniPatch.cpp
    src/NetworkManager.cpp
    src/PatchFinder.cpp
    src/PoseCalibrator.cpp
    src/Relocaliser.cpp
    src/ShiTomasi.cpp
    src/SmallBlurryImage.cpp
    src/System.cpp
    src/SystemBase.cpp
    src/SystemClient.cpp
    src/SystemFrontendBase.cpp
    src/SystemServer.cpp
    src/TaylorCamera.cpp
    src/Tracker.cpp
    src/TrackerCalib.cpp
    src/VideoSourceMulti.cpp
    src/VideoSourceSingle.cpp
)

set(MCPTAM_SRC
    src/Main.cpp
)

set(MCPTAM_CLIENT_SRC
    src/MainClient.cpp
)

set(MCPTAM_SERVER_SRC
    src/MainServer.cpp
)

set(POSE_CALIBRATOR_SRC
    src/MainPoseCalibrator.cpp
)

set(CAMERA_CALIBRATOR_SRC
    src/MainCameraCalibrator.cpp
)


set(MCPTAM_LIBS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    ${CVD_LIBRARY}
    ${OPENGL_gl_LIBRARY}
    ${OpenCV_LIBS}
    ${GVars3_LIBRARIES}
    ${G2O_STUFF_LIBRARY}
    ${G2O_CORE_LIBRARY}
    ${G2O_SOLVER_CHOLMOD}
    ${LAPACK_LIBRARIES}
    ${BLAS_LIBRARIES}
)

# create a library of all the non-main functions
add_library(mcptam_functions ${MCPTAM_FUNCTIONS_LIB_SRC})
target_link_libraries(mcptam_functions ${MCPTAM_LIBS})

# add executables and link them to the libraries
add_executable(mcptam ${MCPTAM_SRC} )
add_dependencies(mcptam ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam ${MCPTAM_LIBS} mcptam_functions)

add_executable(mcptam_client ${MCPTAM_CLIENT_SRC})
add_dependencies(mcptam_client ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam_client ${MCPTAM_LIBS} mcptam_functions)

add_executable(mcptam_server ${MCPTAM_SERVER_SRC})
add_dependencies(mcptam_server ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam_server ${MCPTAM_LIBS} mcptam_functions)

add_executable(camera_calibrator ${CAMERA_CALIBRATOR_SRC})
add_dependencies(camera_calibrator ${PROJECT_NAME}_gencpp)
target_link_libraries(camera_calibrator ${MCPTAM_LIBS} mcptam_functions)

add_executable(pose_calibrator ${POSE_CALIBRATOR_SRC})
add_dependencies(pose_calibrator ${PROJECT_NAME}_gencpp)
target_link_libraries(pose_calibrator ${MCPTAM_LIBS} mcptam_functions)

install(TARGETS mcptam mcptam_client mcptam_server camera_calibrator pose_calibrator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY masks/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/masks
  PATTERN ".svn" EXCLUDE
)
