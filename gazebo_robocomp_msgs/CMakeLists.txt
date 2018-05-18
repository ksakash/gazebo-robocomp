find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".gazebo-[0-9.]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

set (gazebo_robocomp_msgs
    Laser_msgs.proto
    raysensor.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${gazebo_robocomp_msgs})

add_library(Laser_msgs SHARED ${PROTO_SRCS})
target_link_libraries(Laser_msgs ${PROTOBUF_LIBRARY})

