cmake_minimum_required(VERSION 2.8.3)
project(voxl_imu_ros)

# Set shared library link flags
SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# Old policies for compatibility
cmake_policy(SET CMP0054 OLD)
cmake_policy(SET CMP0045 OLD)

find_library(MODAL_PIPE_LIB NAMES modal_pipe HINTS /usr/lib)
find_library(MODAL_JSON_LIB NAMES modal_json HINTS /usr/lib)
find_library(VOXL_IO_LIB NAMES voxl_io HINTS /usr/lib)
find_library(RC_MATH_LIB NAMES rc_math HINTS /usr/lib)

# Find required catkin packages and custom message dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  # Add all the other necessary components for your package
)

# Include message generation for custom messages
# add_message_files and generate_messages would be used if your package has custom messages
# ...

# Declare catkin package and dependencies
catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    # Include other dependencies
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  common
  server/include  # If you have additional includes
  # Include other necessary directories
)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -L /usr/lib64 -pthread -Wall -fpermissive")

# Recursively include all source files from server/src and common
file(GLOB_RECURSE all_src_files
  server/src/*.c*
  common/*.c*
)

# server and client binaries
add_subdirectory(server)



# also install the headers from common
install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install(FILES common/voxl_imu_server.h DESTINATION /usr/include)
