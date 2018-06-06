cmake_minimum_required(VERSION 3.5)
PROJECT(apTracking)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)
LINK_DIRECTORIES(${VISP_LINK_DIRECTORIES})

# Find OpenMP
if(APPLE)
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
    "${apTracking_SOURCE_DIR}/cmake"
  )

# -L/usr/local/Cellar/llvm/5.0.1/lib
# -fopenmp
# 
# /Volumes/Data/Dependencies/visp/build_2.10

#ADD_DEFINITIONS( -Wall -msse -msse2 -ffast-math -funroll-loops -mfpmath=sse)
ADD_DEFINITIONS( -Wall -msse -msse2 -ffast-math -funroll-loops -mfpmath=sse -fomit-frame-pointer)

find_package(Qt5)
FIND_PACKAGE(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5OpenGL)


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

# set (OpenIGTLink_DIR /usr/local CACHE PATH "Path to OpenIGTLink's install dir")
# find_package(OpenIGTLink)
# if(OpenIGTLink_FOUND)
#     include_directories(${OpenIGTLink_INCLUDE_DIRS} .)
#     link_directories(${OpenIGTLink_LIBRARY_DIRS})
#     set(IGTLink_LIBRARIES ${OpenIGTLink_LIBRARIES})
# endif()

LINK_DIRECTORIES(${GLEW_LIBRARY_DIR})

SET(QT_SOURCE_FILES surrender/sceneviewer.cpp surrender/scenemanager.cpp)
SET(QT_HEADERS surrender/sceneviewer.h surrender/scenemanager.h)


QT5_WRAP_CPP(QT_SOURCE_FILES ${QT_HEADERS})

  INCLUDE_DIRECTORIES(
    ${OPENGL_INCLUDE_DIR}
	${CMAKE_SOURCE_DIR}/src 
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

SET(SURRENDER_FILES
    surrender/camera.cpp
    surrender/camera.h
    surrender/fbo.cpp
    surrender/fbo.h
    surrender/glcontextmanager.cpp
    surrender/glcontextmanager.h
    surrender/logs.h
    surrender/modelloader.cpp
    surrender/modelloader.h
    surrender/stateset.cpp
    surrender/stateset.h
    surrender/utils.cpp
    surrender/utils.h
)
ADD_DEFINITIONS( -std=gnu++0x)

SET(TRACKER_COMMON_FILES
    structures.h
    luaconfig.cpp                       luaconfig.h
    apEdgeMap.cpp                       apEdgeMap.h
    apLineExtractor.cpp                 apLineExtractor.h
    apHoughVote.cpp                     apHoughVote.h
    vpFeatureGradient.cpp               vpFeatureGradient.h
    apFeatureGradientOrientation.cpp    apFeatureGradientOrientation.h
    apMbTracker.cpp               		apMbTracker.h
    apControlPoint.cpp                  apControlPoint.h
    vpPointSite.cpp                     vpPointSite.h
    apControlPointTracker.cpp           apControlPointTracker.h
    apDomOrientation.cpp                apDomOrientation.h
    vpSE3Kalman.cpp                     vpSE3Kalman.h
    apRend.cpp                          apRend.h
    apImageFilter.cpp                   apImageFilter.h
    apViews.cpp                         apViews.h
    apDetector.cpp                      apDetector.h
    apPFilter.cpp                       apPFilter.h
    ctParticle.cpp                      ctParticle.h
    apContourPoint.cpp                  apContourPoint.h
    ctPFilter.cpp                       ctPFilter.h
    apLogPolarHist.cpp                  apLogPolarHist.h
    apDetection.cpp						apDetection.h
    apLearn.cpp							apLearn.h
    apSegmentation.cpp					apSegmentation.h
    apMbtDistanceLineMH.cpp				apMbtDistanceLineMH.h	
    apMHMeLine.cpp						apMHMeLine.h
    apMeLine.cpp						apMeLine.h
    apMHMeTracker.cpp					apMHMeTracker.h
    apMHMeSite.cpp						apMHMeSite.h
    apKalmanFilter.cpp					apKalmanFilter.h
    apCCDTracker.cpp					apCCDTracker.h
    graph.cpp							graph.h
    									block.h
    maxflow.cpp							energy.h
    #motionTools.cpp						apLibMotion.h
    apSegMotionCol.cpp					apSegMotionCol.h
    apKernel.cpp						apKernel.h
    apRANSAC.cpp						apRANSAC.h
    minE.cpp							minE.h
    apGMM.cpp							apGMM.h
    apKltControlPoint.cpp				apKltControlPoint.h
    apHoughVoteCircle.cpp				apHoughVoteCircle.h
    p_helper.h
    serialization.h
    zhelpers.h
    apHOGDetector.cpp                     apHOGDetector.h
    ${QT_SOURCE_FILES}                  ${QT_SOURCE_HEADERS}
    ${SURRENDER_FILES}
)

#ADD_EXECUTABLE(apCorrTracking
#    apCorrTracking.cpp
#    ${TRACKER_COMMON_FILES}
#  )


#ADD_EXECUTABLE(apHybridTracking
#    apHybridTracking.cpp
#    apMbHybridTracker.cpp               apMbHybridTracker.h
#    ${TRACKER_COMMON_FILES}
#  )
  
ADD_EXECUTABLE(apCOTracking
    apCOTracking.cpp
    ${TRACKER_COMMON_FILES}
  )

ADD_EXECUTABLE(apXrayTracking
    apXrayTracking.cpp
    ${TRACKER_COMMON_FILES}
  )

ADD_EXECUTABLE(apXrayIntensityContourTracking
    apXrayIntensityContourTracking.cpp
    ${TRACKER_COMMON_FILES}
  )

ADD_EXECUTABLE(apXrayContourCorrespondences
    apXrayContourCorrespondences.cpp
    ${TRACKER_COMMON_FILES}
  )


ADD_EXECUTABLE(apCOTrackingCom
    apCOTrackingCom.cpp
    ${TRACKER_COMMON_FILES}
  )

ADD_EXECUTABLE(apCTContourCorrespondences
    apCTContourCorrespondences.cpp
    ${TRACKER_COMMON_FILES}
  )

ADD_EXECUTABLE(apSimilarityRetexturing
    apSimilarityRetexturing.cpp
    ${TRACKER_COMMON_FILES}
  )


ADD_EXECUTABLE(apTrainHOG
    apTrainHOG.cpp
    ${TRACKER_COMMON_FILES}
  )
  
#  ADD_EXECUTABLE(apCOAR
#    apCOAR.cpp
#    ${TRACKER_COMMON_FILES}
#  )

#  ADD_EXECUTABLE(apCOAR_osg
#    apCOAR_osg.cpp
#    ${TRACKER_COMMON_FILES}
#  )
  
  
#TARGET_LINK_LIBRARIES(apCorrTracking
# 	${OPENGL_LIBRARIES}
#    ${OPENSCENEGRAPH_LIBRARIES}
#    ${QT_LIBRARIES}
#    ${GLEW_LIBRARY}
#  )



#TARGET_LINK_LIBRARIES(apHybridTracking
# 	${OPENGL_LIBRARIES}
#    ${OPENSCENEGRAPH_LIBRARIES}
#    ${QT_LIBRARIES}
#    ${GLEW_LIBRARY}
#  )
  
  TARGET_LINK_LIBRARIES(apCOTracking
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

  TARGET_LINK_LIBRARIES(apXrayTracking
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

  TARGET_LINK_LIBRARIES(apXrayIntensityContourTracking
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

  TARGET_LINK_LIBRARIES(apXrayContourCorrespondences
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

  TARGET_LINK_LIBRARIES(apCOTrackingCom
    ${OPENGL_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    Qt5::Widgets
    ${QT5_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
    ${QtCore_location}
    ${QtOpenGL_location}
    ${GLEW_LIBRARY}
    ${LUA_LIBRARIES} -lzmq ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
  )

  TARGET_LINK_LIBRARIES(apCTContourCorrespondences
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

  TARGET_LINK_LIBRARIES(apSimilarityRetexturing
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

  TARGET_LINK_LIBRARIES(apTrainHOG
    ${OPENGL_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    Qt5::Widgets
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
    ${GLEW_LIBRARY}
    ${LUA_LIBRARIES} -lzmq ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
  )
  
#    TARGET_LINK_LIBRARIES(apCOAR
#        ${OPENGL_LIBRARIES}
#    ${OPENSCENEGRAPH_LIBRARIES}
#    ${QT_LIBRARIES}
#    ${GLEW_LIBRARY}
#    ${LUA_LIBRARIES}
#  )

#    TARGET_LINK_LIBRARIES(apCOAR_osg
#        ${OPENGL_LIBRARIES}
#    ${OPENSCENEGRAPH_LIBRARIES}
#    ${QT_LIBRARIES}
#    ${GLEW_LIBRARY}
#    ${LUA_LIBRARIES}
#  )
