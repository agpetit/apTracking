####################################################################################

find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
# Find OpenMP
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
        set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_LINKER_FLAGS "-L/usr/local/opt/llvm/lib")
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
      set(OpenMP_CXX_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
      set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5")
      set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_LINKER_FLAGS "-L/usr/local/opt/llvm/lib")
    endif()
endif()
find_package( OpenMP REQUIRED)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS} ${OpenMP_LINKER_FLAGS}")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS} ${OpenMP_LINKER_FLAGS}")
ENDIF(OPENMP_FOUND)

find_package(Qt5 COMPONENTS Core Widgets Gui OpenGL REQUIRED)
FIND_PACKAGE(GLEW REQUIRED)
FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(OpenSceneGraph REQUIRED osgDB osgGA osgSim osgViewer)
FIND_PACKAGE(OpenGL REQUIRED)
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)

FIND_LIBRARY(APTrackingLib_LIBRARY APTrackingLib
    PATHS @CMAKE_INSTALL_PREFIX@/lib
    NO_DEFAULT_PATH
)

FIND_PATH(APTrackingLib_INCLUDE_DIR apDetection.h
    PATHS @CMAKE_INSTALL_PREFIX@/include
    NO_DEFAULT_PATH
)

set(APTrackingLib_LIBRARIES 
    ${APTrackingLib_LIBRARY}
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
    ${LUA_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
    ${OpenCV_LIBS}
    ${VISP_LIBRARIES}
  ) 
set(APTrackingLib_INCLUDE_DIRS
  ${APTrackingLib_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/src 
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${OPENSCENEGRAPH_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
)
set(APTrackingLib_DEFINITIONS
    -Wall
) 
    
