cmake_minimum_required(VERSION 2.8.3)
project(mcptam)

# Add cmake_modules directory to cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")
# Define build type
set(CMAKE_BUILD_TYPE None)
# Set our own specific flags. Want something like RelWithDebInfo but without the 
# -DNDEBUG flag that disables asserts
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -march=core2 -msse3 -O3 -g -fopenmp")

# Find various packages in order to set up environment variables
find_package(catkin REQUIRED COMPONENTS roscpp image_transport sensor_msgs cv_bridge geometry_msgs tf message_filters std_msgs std_srvs visualization_msgs dynamic_reconfigure pcl_ros message_generation roslint)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${CVD_INCLUDE_DIR} 
                    ${GVars3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${SUITESPARSE_LIBRARY_DIRS} ${OpenCV_LIB_DIR} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})

add_message_files(
  DIRECTORY msg
  FILES MapInfo.msg MapMakerTiming.msg NetworkMapPoint.msg NetworkMeasurement.msg NetworkKeyFrame.msg NetworkMultiKeyFrame.msg NetworkOutlier.msg
        SystemInfo.msg TrackerTiming.msg TrackerState.msg
)

add_service_files(
  DIRECTORY srv
  FILES ModifyMap.srv Reset.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp image_transport sensor_msgs cv_bridge geometry_msgs tf message_filters std_msgs std_srvs visualization_msgs dynamic_reconfigure pcl_ros message_runtime
  DEPENDS SuiteSparse CVD OpenGL GVars3)

# Check source for formatting
roslint_cpp()

add_executable(mcptam src/Main.cpp src/GLWindow2.cpp src/GLWindowMenu.cpp src/VideoSourceMulti.cpp src/CameraGroupSubscriber.cpp 
                      src/SystemBase.cpp src/SystemFrontendBase.cpp src/System.cpp src/TaylorCamera.cpp src/KeyFrame.cpp src/MapPoint.cpp src/Map.cpp 
                      src/SmallBlurryImage.cpp src/ShiTomasi.cpp src/MapMaker.cpp src/MapMakerBase.cpp src/MapMakerClientBase.cpp src/MapMakerServerBase.cpp 
                      src/PatchFinder.cpp src/Relocaliser.cpp src/Tracker.cpp src/KeyFrameViewer.cpp 
                      src/BundleAdjusterBase.cpp src/MiniPatch.cpp src/BundleAdjusterMulti.cpp src/ChainBundle.cpp)
add_dependencies(mcptam ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} ${GVars3_LIBRARIES}
                             ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})
                             
add_executable(mcptam_client src/MainClient.cpp src/GLWindow2.cpp src/GLWindowMenu.cpp src/VideoSourceMulti.cpp src/CameraGroupSubscriber.cpp  
                             src/SystemBase.cpp src/SystemFrontendBase.cpp src/SystemClient.cpp src/TaylorCamera.cpp src/KeyFrame.cpp src/MapPoint.cpp src/Map.cpp 
                             src/SmallBlurryImage.cpp src/ShiTomasi.cpp src/MapMakerClient.cpp src/MapMakerBase.cpp src/MapMakerClientBase.cpp 
                             src/NetworkManager.cpp src/PatchFinder.cpp src/Relocaliser.cpp src/MiniPatch.cpp 
                             src/Tracker.cpp src/KeyFrameViewer.cpp)
add_dependencies(mcptam_client ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam_client ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} ${GVars3_LIBRARIES}
                                    ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})
 
add_executable(mcptam_server src/MainServer.cpp src/GLWindow2.cpp src/GLWindowMenu.cpp src/VideoSourceMulti.cpp src/CameraGroupSubscriber.cpp   
                             src/SystemBase.cpp src/SystemServer.cpp src/TaylorCamera.cpp src/KeyFrame.cpp src/MapPoint.cpp src/Map.cpp 
                             src/SmallBlurryImage.cpp src/ShiTomasi.cpp src/MapMakerServer.cpp src/MapMakerBase.cpp src/MapMakerServerBase.cpp 
                             src/NetworkManager.cpp src/PatchFinder.cpp src/KeyFrameViewer.cpp  src/MiniPatch.cpp 
                             src/BundleAdjusterBase.cpp src/BundleAdjusterMulti.cpp src/ChainBundle.cpp)
add_dependencies(mcptam_server ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} ${GVars3_LIBRARIES}
                                    ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})

add_executable(camera_calibrator src/MainCameraCalibrator.cpp src/GLWindow2.cpp src/GLWindowMenu.cpp src/VideoSourceSingle.cpp  
                                src/CameraCalibrator.cpp src/CalibImageTaylor.cpp src/CalibCornerPatch.cpp src/TaylorCamera.cpp src/CalibGridCorner.cpp)
add_dependencies(camera_calibrator ${PROJECT_NAME}_gencpp)
target_link_libraries(camera_calibrator ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} ${GVars3_LIBRARIES}
                                        ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})

add_executable(pose_calibrator src/MainPoseCalibrator.cpp src/GLWindow2.cpp src/GLWindowMenu.cpp src/VideoSourceMulti.cpp src/CameraGroupSubscriber.cpp 
                              src/SystemBase.cpp src/PoseCalibrator.cpp src/CalibImageTaylor.cpp src/CalibCornerPatch.cpp src/CalibGridCorner.cpp 
                              src/TrackerCalib.cpp src/MapMakerCalib.cpp src/MapMaker.cpp src/MapMakerBase.cpp src/MapMakerClientBase.cpp src/MapMakerServerBase.cpp
                              src/TaylorCamera.cpp src/KeyFrame.cpp src/MapPoint.cpp src/Map.cpp src/SmallBlurryImage.cpp src/ShiTomasi.cpp  
                              src/PatchFinder.cpp src/Relocaliser.cpp src/Tracker.cpp src/KeyFrameViewer.cpp
                              src/BundleAdjusterBase.cpp src/BundleAdjusterSingle.cpp src/BundleAdjusterCalib.cpp src/ChainBundle.cpp src/MiniPatch.cpp)
add_dependencies(pose_calibrator ${PROJECT_NAME}_gencpp)
target_link_libraries(pose_calibrator ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} ${GVars3_LIBRARIES}
                                     ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})
                                     
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
