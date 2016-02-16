cmake_minimum_required(VERSION 2.6) 

# DoorsDetector
PROJECT(motion_project)
set(LIBRARY_OUTPUT_PATH    ../bin/${CMAKE_BUILD_TYPE})
set(EXECUTABLE_OUTPUT_PATH    ../bin/${CMAKE_BUILD_TYPE})
set (Tutorial_VERSION_MAJOR 0)
set (Tutorial_VERSION_MINOR 0)

FIND_PACKAGE(OpenCV REQUIRED)
IF ( NOT OpenCV_FOUND )
    MESSAGE(FATAL_ERROR "OpenCV not found !")
ENDIF ( NOT OpenCV_FOUND )

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


FIND_PACKAGE(VISP REQUIRED)
IF(NOT VISP_FOUND)
    MESSAGE(FATAL_ERROR "VISP not found !")
ENDIF(NOT VISP_FOUND)

INCLUDE(${VISP_USE_FILE})



INCLUDE_DIRECTORIES(${PTHREAD_INCLUDE_DIR})

FILE( GLOB    source_files	src/*)

# Déclaration de l'exécutable
ADD_EXECUTABLE ( libMotion    ${source_files})

TARGET_LINK_LIBRARIES( libMotion ${OpenCV_LIBS})