cmake_minimum_required(VERSION 3.0)
project(CS_SLAM)

set(CMAKE_VERBOSE_MAKEFILE OFF)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall   -O3")

# 设置编译选项
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

# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-g -std=c++14 -Wformat")
set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3" )
add_compile_options(-fPIC)



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

find_package(Boost COMPONENTS)

if(NOT Boost_FOUND)
   message(FATAL_ERROR "Boost not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GTest REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

include_directories(${YAMLCPP_INCLUDE_DIRS}) 
# include_directories(/home/fernando/yaml-cpp/include/yaml-cpp)
include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${GTEST_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
# src/Atlas.cpp
src/Converter.cpp
src/EKF.cpp
src/LocalMap.cpp
src/LocalMapper.cpp
src/MSCKF.cpp
src/RandomVector.cpp
src/MapPoint.cpp
src/KeyFrame.cpp
src/ScanFormer.cpp
# src/MapDrawer.cpp
src/LoopClosing.cpp
src/MeasurementPackage.cpp
src/System.cpp
src/Utils.cpp
src/Viewer.cpp
src/Drawer.cpp
src/Frames.cpp
# src/FrameDrawer.cpp
include/Converter.h
include/EKF.h
include/LocalMap.h
include/LocalMapper.h
include/MSCKF.h
include/RandomVector.h
include/KeyFrame.h
include/Drawer.h
# include/MapDrawer.ha
# include/MapPoint.h
include/MeasurementPackage.h
include/ScanFormer.h
include/FrameDrawer.h
# include/PoseGraph.h
include/System.h
include/Utils.h
include/Frames.h
# include/Atlas.h
include/Viewer.h
include/LoopClosing.h
# include/SetParameters.h
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

link_directories(${YAMLCPP_LIBRARY_DIRS})
# link_directories(/home/fernando/yaml-cpp/build)


target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${YAMLCPP_LIBRARIES}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBRARIES}
${CERES_LIBRARIES}
${GTEST_BOTH_LIBRARIES}
pthread
)



set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/cave)
add_executable(cave
examples/cave.cpp)
target_link_libraries(cave ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(TEST_MSCKF
test/TEST_MSCKF.cpp)
target_link_libraries(TEST_MSCKF ${PROJECT_NAME})
# MSCKF_no_problem

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(TEST_SCANMATCHING
test/TEST_SCANMATCHING.cpp)
target_link_libraries(TEST_SCANMATCHING ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(TEST_DRAWER
test/TEST_DRAWER.cpp)
target_link_libraries(TEST_DRAWER ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(TEST_EKF
test/TEST_EKF.cpp)
target_link_libraries(TEST_EKF ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(TEST_RANDOMVECTOR
test/TEST_RANDOMVECTOR.cpp)
target_link_libraries(TEST_RANDOMVECTOR ${PROJECT_NAME})