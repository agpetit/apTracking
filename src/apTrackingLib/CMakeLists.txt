cmake_minimum_required(VERSION 3.5)
PROJECT(APTrackingLib)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)


SET(APTRACKING_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../)

find_package(OpenCV)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)
LINK_DIRECTORIES(${VISP_LINK_DIRECTORIES})

# Find OpenMP
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
        set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
      set(OpenMP_CXX_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
      set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5")
      set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})
    endif()
endif()

FIND_PACKAGE( OpenMP REQUIRED)
IF(OPENMP_FOUND)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OPENMP_FOUND)

set(CMAKE_MODULE_PATH 
	${CMAKE_SOURCE_DIR}/CMakeModules
    "${APTRACKING_SOURCE_DIR}/cmake"
  )

ADD_DEFINITIONS( -Wall -msse -msse2 -ffast-math -funroll-loops -mfpmath=sse -fomit-frame-pointer)

find_package(Qt5 COMPONENTS Core Widgets Gui OpenGL REQUIRED)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

get_target_property(QtCore_location Qt5::Core LOCATION)
get_target_property(QtGui_location Qt5::Gui LOCATION)
get_target_property(QtOpenGL_location Qt5::OpenGL LOCATION)

#include(${QT5_USE_FILE})
FIND_PACKAGE(GLEW)
IF(NOT GLEW_FOUND)
    MESSAGE(CRITICAL GLEW not found!)
ENDIF(NOT GLEW_FOUND)
FIND_PACKAGE(Lua51)
IF(NOT LUA51_FOUND)
    MESSAGE(CRITICAL Lua not found!)
ENDIF(NOT LUA51_FOUND)
FIND_PACKAGE(OpenSceneGraph REQUIRED osgDB osgGA osgSim osgViewer)
FIND_PACKAGE(OpenGL)
IF(NOT OPENGL_FOUND)
    MESSAGE(CRITICAL OpenGL not found!)
ENDIF(NOT OPENGL_FOUND)
FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

find_package(Boost COMPONENTS system filesystem serialization REQUIRED)

LINK_DIRECTORIES(${GLEW_LIBRARY_DIR})


INCLUDE_DIRECTORIES(
    ${APTRACKING_SOURCE_DIR}/../../build 
    ${APTRACKING_SOURCE_DIR}/.. 
    ${OPENGL_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${VISP_INCLUDE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
  )

set(APTRACKING_DEFINITIONS -std=gnu++0x -DENABLE_LIBRARY_COMPILE)
add_definitions(${APTRACKING_DEFINITIONS})

SET(QT_HEADERS
    ${APTRACKING_SOURCE_DIR}/surrender/sceneviewer.h
    ${APTRACKING_SOURCE_DIR}/surrender/scenemanager.h
)

QT5_WRAP_CPP(QT_SOURCES ${QT_HEADERS})

SET(QT_SOURCES
    ${QT_SOURCES}
    ${APTRACKING_SOURCE_DIR}/surrender/sceneviewer.cpp
    ${APTRACKING_SOURCE_DIR}/surrender/scenemanager.cpp
)


SET(SURRENDER_HEADERS
    ${APTRACKING_SOURCE_DIR}/surrender/camera.h
    ${APTRACKING_SOURCE_DIR}/surrender/fbo.h
    ${APTRACKING_SOURCE_DIR}/surrender/glcontextmanager.h
    ${APTRACKING_SOURCE_DIR}/surrender/modelloader.h
    ${APTRACKING_SOURCE_DIR}/surrender/stateset.h
    ${APTRACKING_SOURCE_DIR}/surrender/logs.h
    ${APTRACKING_SOURCE_DIR}/surrender/utils.h
)

SET(SURRENDER_SOURCES
    ${APTRACKING_SOURCE_DIR}/surrender/camera.cpp
    ${APTRACKING_SOURCE_DIR}/surrender/fbo.cpp
    ${APTRACKING_SOURCE_DIR}/surrender/glcontextmanager.cpp
    ${APTRACKING_SOURCE_DIR}/surrender/modelloader.cpp
    ${APTRACKING_SOURCE_DIR}/surrender/stateset.cpp
    ${APTRACKING_SOURCE_DIR}/surrender/utils.cpp
)

SET(TRACKER_COMMON_HEADERS
    ${APTRACKING_SOURCE_DIR}/structures.h
    ${APTRACKING_SOURCE_DIR}/luaconfig.h
    ${APTRACKING_SOURCE_DIR}/apEdgeMap.h
    ${APTRACKING_SOURCE_DIR}/apLineExtractor.h
    ${APTRACKING_SOURCE_DIR}/apHoughVote.h
    ${APTRACKING_SOURCE_DIR}/vpFeatureGradient.h
    ${APTRACKING_SOURCE_DIR}/apFeatureGradientOrientation.h
    ${APTRACKING_SOURCE_DIR}/apMbTracker.h
    ${APTRACKING_SOURCE_DIR}/apControlPoint.h
    ${APTRACKING_SOURCE_DIR}/vpPointSite.h
    ${APTRACKING_SOURCE_DIR}/apControlPointTracker.h
    ${APTRACKING_SOURCE_DIR}/apDomOrientation.h
    ${APTRACKING_SOURCE_DIR}/vpSE3Kalman.h
    ${APTRACKING_SOURCE_DIR}/apRend.h
    ${APTRACKING_SOURCE_DIR}/apImageFilter.h
    ${APTRACKING_SOURCE_DIR}/apViews.h
    ${APTRACKING_SOURCE_DIR}/apDetector.h
    ${APTRACKING_SOURCE_DIR}/apPFilter.h
    ${APTRACKING_SOURCE_DIR}/ctParticle.h
    ${APTRACKING_SOURCE_DIR}/apContourPoint.h
    ${APTRACKING_SOURCE_DIR}/ctPFilter.h
    ${APTRACKING_SOURCE_DIR}/apLogPolarHist.h
    ${APTRACKING_SOURCE_DIR}/apDetection.h
    ${APTRACKING_SOURCE_DIR}/apLearn.h
    ${APTRACKING_SOURCE_DIR}/apSegmentation.h
    ${APTRACKING_SOURCE_DIR}/apMbtDistanceLineMH.h
    ${APTRACKING_SOURCE_DIR}/apMHMeLine.h
    ${APTRACKING_SOURCE_DIR}/apMeLine.h
    ${APTRACKING_SOURCE_DIR}/apMHMeTracker.h
    ${APTRACKING_SOURCE_DIR}/apMHMeSite.h
    ${APTRACKING_SOURCE_DIR}/apKalmanFilter.h
    ${APTRACKING_SOURCE_DIR}/apCCDTracker.h
    ${APTRACKING_SOURCE_DIR}/apSegMotionCol.h
    ${APTRACKING_SOURCE_DIR}/graph.h
    ${APTRACKING_SOURCE_DIR}/block.h
    ${APTRACKING_SOURCE_DIR}/energy.h
    ${APTRACKING_SOURCE_DIR}/apKernel.h
    ${APTRACKING_SOURCE_DIR}/apRANSAC.h
    ${APTRACKING_SOURCE_DIR}/minE.h
    ${APTRACKING_SOURCE_DIR}/apGMM.h
    ${APTRACKING_SOURCE_DIR}/apKltControlPoint.h
    ${APTRACKING_SOURCE_DIR}/apHoughVoteCircle.h
    ${APTRACKING_SOURCE_DIR}/serialization.h
    ${APTRACKING_SOURCE_DIR}/apHOGDetector.h
    ${APTRACKING_SOURCE_DIR}/p_helper.h
    ${APTRACKING_SOURCE_DIR}/zhelpers.h
    ${APTRACKING_SOURCE_DIR}/vibe-background.h
    
)

SET(TRACKER_COMMON_SOURCES
    ${APTRACKING_SOURCE_DIR}/luaconfig.cpp
    ${APTRACKING_SOURCE_DIR}/apEdgeMap.cpp
    ${APTRACKING_SOURCE_DIR}/apLineExtractor.cpp
    ${APTRACKING_SOURCE_DIR}/apHoughVote.cpp
    ${APTRACKING_SOURCE_DIR}/vpFeatureGradient.cpp
    ${APTRACKING_SOURCE_DIR}/apFeatureGradientOrientation.cpp
    ${APTRACKING_SOURCE_DIR}/apMbTracker.cpp
    ${APTRACKING_SOURCE_DIR}/apControlPoint.cpp
    ${APTRACKING_SOURCE_DIR}/vpPointSite.cpp
    ${APTRACKING_SOURCE_DIR}/apControlPointTracker.cpp
    ${APTRACKING_SOURCE_DIR}/apDomOrientation.cpp
    ${APTRACKING_SOURCE_DIR}/vpSE3Kalman.cpp
    ${APTRACKING_SOURCE_DIR}/apRend.cpp
    ${APTRACKING_SOURCE_DIR}/apImageFilter.cpp
    ${APTRACKING_SOURCE_DIR}/apViews.cpp
    ${APTRACKING_SOURCE_DIR}/apDetector.cpp
    ${APTRACKING_SOURCE_DIR}/apPFilter.cpp
    ${APTRACKING_SOURCE_DIR}/ctParticle.cpp
    ${APTRACKING_SOURCE_DIR}/apContourPoint.cpp
    ${APTRACKING_SOURCE_DIR}/ctPFilter.cpp
    ${APTRACKING_SOURCE_DIR}/apLogPolarHist.cpp
    ${APTRACKING_SOURCE_DIR}/apDetection.cpp
    ${APTRACKING_SOURCE_DIR}/apLearn.cpp
    ${APTRACKING_SOURCE_DIR}/apSegmentation.cpp
    ${APTRACKING_SOURCE_DIR}/apMbtDistanceLineMH.cpp
    ${APTRACKING_SOURCE_DIR}/apMHMeLine.cpp
    ${APTRACKING_SOURCE_DIR}/apMeLine.cpp
    ${APTRACKING_SOURCE_DIR}/apMHMeTracker.cpp
    ${APTRACKING_SOURCE_DIR}/apMHMeSite.cpp
    ${APTRACKING_SOURCE_DIR}/apKalmanFilter.cpp
    ${APTRACKING_SOURCE_DIR}/apCCDTracker.cpp
    ${APTRACKING_SOURCE_DIR}/graph.cpp
    ${APTRACKING_SOURCE_DIR}/maxflow.cpp
    ${APTRACKING_SOURCE_DIR}/apSegMotionCol.cpp
    ${APTRACKING_SOURCE_DIR}/apKernel.cpp
    ${APTRACKING_SOURCE_DIR}/apRANSAC.cpp
    ${APTRACKING_SOURCE_DIR}/minE.cpp
    ${APTRACKING_SOURCE_DIR}/apGMM.cpp
    ${APTRACKING_SOURCE_DIR}/apKltControlPoint.cpp
    ${APTRACKING_SOURCE_DIR}/apHoughVoteCircle.cpp
    ${APTRACKING_SOURCE_DIR}/apHOGDetector.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${TRACKER_COMMON_HEADERS}
    ${QT_HEADERS}
    ${SURRENDER_HEADERS}
    ${TRACKER_COMMON_SOURCES}
    ${QT_SOURCES}
    ${SURRENDER_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${OPENGL_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    Qt5::Widgets
    ${QT5_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
    ${QtCore_location}
    ${QtGui_location}
    ${QtOpenGL_location}
    ${GLEW_LIBRARY}
    ${LUA_LIBRARIES} -lzmq ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${TRACKER_COMMON_HEADERS}" )

set(APTRACKING_SHADER_DIR ${PROJECT_BINARY_DIR}/..)

configure_file (
  "${CMAKE_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )

configure_file (
  "${CMAKE_SOURCE_DIR}/apTrackingLibConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/apTrackingLibConfig.cmake"
  @ONLY
  )

install(TARGETS ${PROJECT_NAME}
        COMPONENT ${PROJECT_NAME}_libraries
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION "include/")

install (DIRECTORY "${APTRACKING_SOURCE_DIR}/surrender" DESTINATION include FILES_MATCHING PATTERN "*.h")

install (FILES "${PROJECT_BINARY_DIR}/config.h" DESTINATION include )
install (FILES "${PROJECT_BINARY_DIR}/apTrackingLibConfig.cmake" DESTINATION cmake )
