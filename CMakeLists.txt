cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(gazebo_robocomp_pkgs)

FIND_PACKAGE(Boost 1.40 COMPONENTS system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
# find_package(Ice REQUIRED)
    # Ice 
    # IceUtil
    # Glacier2 
    # IceBox 
    # IceGrid 
    # IcePatch2 
    # IceStorm


include(FindBoost)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
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

add_library(gazebo_robocomp_laser SHARED gazebo_plugins/gazebo_robocomp_laser.cc gazebo_plugins/gazebo_robocomp_laser.hh)
target_link_libraries(gazebo_robocomp_laser Laser_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES} RayPlugin)
add_dependencies(gazebo_robocomp_laser Laser_msgs)

add_library(gazebo_robocomp_IMU SHARED gazebo_plugins/gazebo_robocomp_IMU.cc gazebo_plugins/gazebo_robocomp_IMU.hh)
target_link_libraries(gazebo_robocomp_IMU ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_library(gazebo_robocomp_joint SHARED gazebo_plugins/gazebo_robocomp_joint.cc gazebo_plugins/gazebo_robocomp_joint.hh)
target_link_libraries(gazebo_robocomp_joint ${GAZEBO_LIBRARIES} ${Boost_LIBRARIS} ${SDF_LIBRARIES} )

add_executable(joint_vel src/vel.cc)
target_link_libraries(joint_vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_library(gazebo_robocomp_DiffDrive SHARED gazebo_plugins/gazebo_robocomp_DiffDrive.cc gazebo_plugins/gazebo_robocomp_DiffDrive.hh)
target_link_libraries(gazebo_robocomp_DiffDrive ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_executable(My_Robot_Vel src/diff_drive_cmd.cc)
target_link_libraries(My_Robot_Vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_library(gazebo_robocomp_camera SHARED gazebo_plugins/gazebo_robocomp_camera.cc gazebo_plugins/gazebo_robocomp_camera.hh)
target_link_libraries(gazebo_robocomp_camera 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${SDF_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${OGRE_LIBRARIES} 
    ${OGRE-Paging_LIBRARIES} 
    ${OGRE-Terrain_LIBRARIES}
)

add_library(gazebo_robocomp_bumper SHARED gazebo_plugins/gazebo_robocomp_bumper.cc gazebo_plugins/gazebo_robocomp_bumper.hh)
target_link_libraries(gazebo_robocomp_bumper ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_library(gazebo_robocomp_RGBD SHARED gazebo_plugins/gazebo_robocomp_RGBD.cc gazebo_plugins/gazebo_robocomp_RGBD.hh)
target_link_libraries(gazebo_robocomp_RGBD ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

execute_process(COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/ice-interface/GenericBase.ice
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/ice-interface
)

message(COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/ice-interface/GenericBase.ice
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/ice-interface/
)

execute_process(COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/ice-interface/Laser.ice
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/ice-interface
)

message(COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/ice-interface/Laser.ice
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/ice-interface/
)

add_library(GenericBase SHARED ice-interface/GenericBase.cpp ice-interface/GenericBase.h)
target_link_libraries(GenericBase 
   ${Boost_LIBRARIES} 
   #Ice
   #IceUtil 
   pthread
   /usr/lib/x86_64-linux-gnu/c++11
)

add_library(Laser SHARED ice-interface/Laser.cpp ice-interface/Laser.h)
target_link_libraries(Laser
   GenericBase
   ${Boost_LIBRARIES} 
   #Ice
   #IceUtil 
   pthread
   /usr/lib/x86_64-linux-gnu/c++11
)

add_library(LaserI SHARED ice-interface/LaserI.cpp ice-interface/LaserI.h)
target_link_libraries(LaserI Laser Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(server src/server.cpp)
target_link_libraries(server Laser LaserI Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(client src/client.cpp)
target_link_libraries(client ${GAZEBO_LIBRARIES} LaserI ${Boost_LIBRARIES} Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES})