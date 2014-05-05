cmake_minimum_required(VERSION 2.8.3)
project(mcptam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake_modules")
set(CMAKE_BUILD_TYPE None)

# Set our own specific flags. Want something like RelWithDebInfo but without the 
# -DNDEBUG flag that disables asserts
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -march=core2 -msse3 -O1 -g")

# Find various packages in order to set up environment variables
find_package(catkin REQUIRED COMPONENTS roscpp image_transport sensor_msgs cv_bridge geometry_msgs tf message_filters std_msgs
                                        std_srvs visualization_msgs dynamic_reconfigure pcl_ros message_generation)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(SUITESPARSE REQUIRED)
find_package(CVD REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(G2O REQUIRED)
find_package(GVars3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${CVD_INCLUDE_DIR} 
                    ${GVars3_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${SUITESPARSE_LIBRARY_DIRS})

add_message_files(
  DIRECTORY msg
  FILES MapInfo.msg NetworkMapPoint.msg NetworkMeasurement.msg NetworkKeyFrame.msg NetworkMultiKeyFrame.msg NetworkOutlier.msg
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
  CATKIN_DEPENDS roscpp image_transport sensor_msgs cv_bridge geometry_msgs tf message_filters std_msgs std_srvs 
                 visualization_msgs dynamic_reconfigure pcl_ros message_runtime
  DEPENDS SuiteSparse CVD OpenGL GVars3)

add_executable(mcptam src/Main.cc src/GLWindow2.cc src/GLWindowMenu.cc src/VideoSourceMulti.cc src/CameraGroupSubscriber.cc 
                      src/SystemBase.cc src/SystemFrontendBase.cc src/System.cc src/TaylorCamera.cc src/KeyFrame.cc src/MapPoint.cc src/Map.cc 
                      src/SmallBlurryImage.cc src/ShiTomasi.cc src/MapMaker.cc src/MapMakerBase.cc src/MapMakerClientBase.cc src/MapMakerServerBase.cc 
                      src/PatchFinder.cc src/Relocaliser.cc src/Tracker.cc src/KeyFrameViewer.cc 
                      src/BundleAdjusterBase.cc src/MiniPatch.cc src/BundleAdjusterMulti.cc src/ChainBundle.cc)
add_dependencies(mcptam ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${GVars3_LIBRARIES}
                             ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD}                             )
                             
add_executable(mcptam_client src/MainClient.cc src/GLWindow2.cc src/GLWindowMenu.cc src/VideoSourceMulti.cc src/CameraGroupSubscriber.cc  
                             src/SystemBase.cc src/SystemFrontendBase.cc src/SystemClient.cc src/TaylorCamera.cc src/KeyFrame.cc src/MapPoint.cc src/Map.cc 
                             src/SmallBlurryImage.cc src/ShiTomasi.cc src/MapMakerClient.cc src/MapMakerBase.cc src/MapMakerClientBase.cc 
                             src/NetworkManager.cc src/PatchFinder.cc src/Relocaliser.cc src/MiniPatch.cc 
                             src/Tracker.cc src/KeyFrameViewer.cc)
add_dependencies(mcptam_client ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam_client ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${GVars3_LIBRARIES})
 
add_executable(mcptam_server src/MainServer.cc src/GLWindow2.cc src/GLWindowMenu.cc src/VideoSourceMulti.cc src/CameraGroupSubscriber.cc   
                             src/SystemBase.cc src/SystemServer.cc src/TaylorCamera.cc src/KeyFrame.cc src/MapPoint.cc src/Map.cc 
                             src/SmallBlurryImage.cc src/ShiTomasi.cc src/MapMakerServer.cc src/MapMakerBase.cc src/MapMakerServerBase.cc 
                             src/NetworkManager.cc src/PatchFinder.cc src/KeyFrameViewer.cc  src/MiniPatch.cc 
                             src/BundleAdjusterBase.cc src/BundleAdjusterMulti.cc src/ChainBundle.cc)
add_dependencies(mcptam_server ${PROJECT_NAME}_gencpp)
target_link_libraries(mcptam_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${GVars3_LIBRARIES}
                                    ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD}                                    )

add_executable(camera_calibrator src/MainCameraCalibrator.cc src/GLWindow2.cc src/GLWindowMenu.cc src/VideoSourceSingle.cc  
                                src/CameraCalibrator.cc src/CalibImageTaylor.cc src/CalibCornerPatch.cc src/TaylorCamera.cc src/CalibGridCorner.cc)
add_dependencies(camera_calibrator ${PROJECT_NAME}_gencpp)
target_link_libraries(camera_calibrator ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${GVars3_LIBRARIES})

add_executable(pose_calibrator src/MainPoseCalibrator.cc src/GLWindow2.cc src/GLWindowMenu.cc src/VideoSourceMulti.cc src/CameraGroupSubscriber.cc 
                              src/SystemBase.cc src/PoseCalibrator.cc src/CalibImageTaylor.cc src/CalibCornerPatch.cc src/CalibGridCorner.cc 
                              src/TrackerCalib.cc src/MapMakerCalib.cc src/MapMaker.cc src/MapMakerBase.cc src/MapMakerClientBase.cc src/MapMakerServerBase.cc
                              src/TaylorCamera.cc src/KeyFrame.cc src/MapPoint.cc src/Map.cc src/SmallBlurryImage.cc src/ShiTomasi.cc  
                              src/PatchFinder.cc src/Relocaliser.cc src/Tracker.cc src/KeyFrameViewer.cc
                              src/BundleAdjusterBase.cc src/BundleAdjusterSingle.cc src/BundleAdjusterCalib.cc src/ChainBundle.cc src/MiniPatch.cc)
add_dependencies(pose_calibrator ${PROJECT_NAME}_gencpp)
target_link_libraries(pose_calibrator ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${GVars3_LIBRARIES}
                                     ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD})
                             
# Start of test and/or experimental programs
#~ add_executable(bundle_test_file_2 experimental/tests/BundleTestFile2.cc src/TaylorCamera.cc src/Map.cc src/KeyFrame.cc src/MapPoint.cc
                                  #~ src/SmallBlurryImage.cc src/ShiTomasi.cc src/BundleAdjusterBase.cc src/MiniPatch.cc 
                                  #~ src/ChainBundle.cc src/BundleAdjusterSingle.cc)
#~ add_dependencies(bundle_test_file_2 ${PROJECT_NAME}_gencpp)
#~ target_link_libraries(bundle_test_file_2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CVD_LIBRARY} ${OPENGL_gl_LIBRARY} ${GVars3_LIBRARIES}
                                     #~ ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CHOLMOD})
                                     #~ 
                                     
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
