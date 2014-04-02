# - Try to find libCVD
#
#  CVD_FOUND - system has libCVD
#  CVD_INCLUDE_DIR - the libCVD include directories
#  CVD_LIBRARY - link these to use libCVD

FIND_PATH(
  CVD_INCLUDE_DIR
  NAMES cvd/cvd_image.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../libcvd
    ${CMAKE_SOURCE_DIR}/../cvd
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  CVD_LIBRARY
  NAMES cvd
  PATHS
    ${CMAKE_SOURCE_DIR}/../libcvd
    ${CMAKE_SOURCE_DIR}/../cvd
    /usr/lib
    /usr/local/lib
) 

IF(CVD_INCLUDE_DIR AND CVD_LIBRARY)
  SET(CVD_FOUND TRUE)
ENDIF()

IF(CVD_FOUND)
   IF(NOT CVD_FIND_QUIETLY)
      MESSAGE(STATUS "Found CVD: ${CVD_LIBRARY}")
   ENDIF()
ELSE()
   IF(CVD_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find CVD")
   ENDIF()
ENDIF()
