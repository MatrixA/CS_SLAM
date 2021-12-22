cmake_minimum_required(VERSION 3.0)
project(CS_SLAM)


IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall   -O3")


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_STANDARD 11)


find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS})

find_package(OpenCV 4)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()


MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

find_package(Boost COMPONENTS)

if(NOT Boost_FOUND)
   message(FATAL_ERROR "Boost not found.")
endif()
include_directories(${Boost_INCLUDE_DIRS}) 

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(GTest REQUIRED)



include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${YAMLCPP_INCLUDE_DIRS}
${GTEST_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/EKF.cpp
src/Atlas.cpp
src/Map.cpp
src/ASEKF.cpp
src/RandomVector.cpp
src/MapPoint.cpp
src/KeyFrame.cpp
src/ScanFormer.cpp
# src/MapDrawer.cpp
src/MeasurementPackage.cpp
src/System.cpp
src/Utils.cpp
src/Viewer.cpp
src/Drawer.cpp
# src/FrameDrawer.cpp
include/ASEKF.h
# include/Map.h
include/EKF.h
include/RandomVector.h
include/KeyFrame.h
include/Map.h
include/Drawer.h
# include/MapDrawer.h
# include/MapPoint.h
include/MeasurementPackage.h
include/ScanFormer.h
include/FrameDrawer.h
# include/PoseGraph.h
include/System.h
include/Utils.h
include/Atlas.h
include/Viewer.h
# include/SetParameters.h
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBRARIES}
${CERES_LIBRARIES}
${YAMLCPP_LIBRARIES}
${GTEST_BOTH_LIBRARIES}
pthread
)
message("Linked")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/cave)
add_executable(cave
examples/cave.cpp)
target_link_libraries(cave ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(test
test/test_what.cpp)
target_link_libraries(test ${PROJECT_NAME})





