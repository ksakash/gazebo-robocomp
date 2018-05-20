cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(gazebo_robocomp_pkgs)

FIND_PACKAGE(Boost 1.40 COMPONENTS system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)

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
    ${PCL_INCLUDE_LIBRARIES}
    build/gazebo_robocomp_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS} gazebo_robocomp_msgs ${OpenCV_LIBRARIES})
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
