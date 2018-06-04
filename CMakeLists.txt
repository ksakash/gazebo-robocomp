cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(gazebo_robocomp_pkgs)

FIND_PACKAGE(Boost 1.40 COMPONENTS system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
# find_package(Ice REQUIRED COMPONENTS
    # Ice 
    # IceUtil
    # Glacier2 
    # IceBox 
    # IceGrid 
    # IcePatch2 
    # IceStorm
# )
include(FindBoost)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11 -lpcl_filters")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
    pkg_check_modules(SDF sdformat)
    pkg_check_modules(OPENCV opencv)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Paging OGRE-Paging)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE-Paging_INCLUDE_DIRS}
    ${OGRE-Terrain_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    build/gazebo_robocomp_msgs
    ${easyiceconfig_INCLUDE_DIRS}
    ice-interface
) 

link_directories(
    ${GAZEBO_LIBRARY_DIRS} 
    gazebo_robocomp_msgs 
    ${OpenCV_LIBRARIES} 
    ${PCL_LIBRARY_DIRS} 
    ${easyiceconfig_LIBRARY_DIRS} 
    # IceUtil 
    /usr/lib/x86_64-linux-gnu/c++11
    # Ice
    pthread 
    ${ZeroCIce_LIBRARIES} 
)

add_definitions(${PCL_DEFINITIONS})
add_subdirectory(gazebo_robocomp_msgs)
add_subdirectory(gazebo_plugins)
add_subdirectory(slice)
add_subdirectory(slice_cpp)

add_executable(joint_vel src/vel.cc)
target_link_libraries(joint_vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_executable(My_Robot_Vel src/diff_drive_cmd.cc)
target_link_libraries(My_Robot_Vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_library(bumperI SHARED ice-interface/bumperI.cpp ice-interface/bumperI.h)
target_link_libraries(bumperI bumper Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(CameraI SHARED ice-interface/CameraI.cpp ice-interface/CameraI.h)
target_link_libraries(CameraI Camera Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(RGBDI SHARED ice-interface/RGBDI.cpp ice-interface/RGBDI.h)
target_link_libraries(RGBDI RGBD Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(LaserI SHARED ice-interface/LaserI.cpp ice-interface/LaserI.h)
target_link_libraries(LaserI Laser Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(MotorI SHARED ice-interface/MotorI.cpp ice-interface/MotorI.h)
target_link_libraries(MotorI Motors Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(IMUI SHARED ice-interface/IMUI.cpp ice-interface/IMUI.h)
target_link_libraries(IMUI IMU Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(DifferentialRobotI ice-interface/DifferentialRobotI.cpp ice-interface/DifferentialRobotI.h)
target_link_libraries(DifferentialRobotI DifferentialRobot diffdrive_cmd diffdrive_state Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(DifferentialRobotI diffdrive_cmd diffdrive_state)

add_executable(server src/server.cpp)
target_link_libraries(server Laser LaserI Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(client src/client.cpp)
target_link_libraries(client ${GAZEBO_LIBRARIES} Laser ${Boost_LIBRARIES} Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES})
