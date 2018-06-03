cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(camera_test C CXX)

#boost
#set (Boost_USE_STATIC_LIBS ON)
option(Boost_USE_STATIC_LIBS "Build with static BOOST libraries instead of dynamic" ON)
find_package ( Boost 1.47 REQUIRED COMPONENTS 
filesystem 
system
thread
serialization)
include_directories( ${Boost_INCLUDE_DIR} )
link_directories(${Boost_LIBRARIES_DIRS})
message(info ${Boost_LIBRARIES})
message(info ${Boost_LIBRARIES_DIRS})

#opencv
find_package( OpenCV 2.4.9 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#flycapture
set(FlyCaptureRoot "C:/Program Files/Point Grey Research/FlyCapture2")
set(FlyCaptureLIB ${FlyCaptureRoot}/lib/FlyCapture2_v100.lib)
set(FlyCaptureINCLUDE ${FlyCaptureRoot}/include)
include_directories( ${FlyCaptureINCLUDE} )

include_directories(../src)

add_definitions(-std=c++11)

add_executable (camera_test ./src/main.cpp
							./src/CameraOpr.cpp
							./src/CameraOpr.h
							./src/CameraPointGreyImpl.h
							./src/CameraPointGreyImpl.cpp)

target_link_libraries (camera_test ${Boost_LIBRARIES} ${OpenCV_LIBS} ${FlyCaptureLIB})

