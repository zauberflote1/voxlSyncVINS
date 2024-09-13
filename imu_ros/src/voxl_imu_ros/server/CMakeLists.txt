cmake_minimum_required(VERSION 3.3)
project(server_node)


# Find Boost with the required components, e.g., thread, for the ROS node
find_package(Boost REQUIRED COMPONENTS thread)



# Include directories for ROS, Boost, and common headers
include_directories(
  include/voxl_imu_ros
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ../common  # Update this path as necessary
)

# List all the source files you want to compile, including the ROS node
set(SERVER_SOURCES
  src/config_file.c
  src/f32_ringbuf.c
  src/fft.cpp
  src/icm20948.c
  src/icm42688.c
  src/imu_interface.c
  src/misc.c
  src/mpu9250.c
  src/node.cpp
  src/timestamp.c
  ../common/cal_file.c
    # And any other source files from common
)

# Add the node.cpp source file and any additional sources from common
add_executable(${PROJECT_NAME}_node ${SERVER_SOURCES})

# Link the ROS libraries, Boost, and custom dependencies for the ROS node
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  # Custom libraries
   modal_pipe
  modal_json
  voxl_io
  rc_math
  # Standard libraries, if needed (e.g., math lib 'm', pthreads, etc.)
  m
  pthread
)

# Specify where to install the built server node
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

