################################################################################
# * Copyright 2020 ModalAI Inc.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * 1. Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# * 2. Redistributions in binary form must reproduce the above copyright notice,
# *    this list of conditions and the following disclaimer in the documentation
# *    and/or other materials provided with the distribution.
# *
# * 3. Neither the name of the copyright holder nor the names of its contributors
# *    may be used to endorse or promote products derived from this software
# *    without specific prior written permission.
# *
# * 4. The Software is used solely in conjunction with devices provided by
# *    ModalAI Inc.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
################################################################################

cmake_minimum_required(VERSION 2.8.3)
project(voxl_mpa_to_ros)

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

cmake_policy(SET CMP0054 OLD)
cmake_policy(SET CMP0045 OLD)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  dynamic_reconfigure
  image_transport
  nodelet
  sensor_msgs
  tf2
  tf2_ros
  message_generation
  )

add_message_files(
    FILES
    AiDetection.msg
)

generate_messages()


catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    sensor_msgs
    geometry_msgs
    message_runtime
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ./include
  ./include/interfaces
  ./include/utils
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -L /usr/lib64 -pthread -Wall -fpermissive -Wno-misleading-indentation -Wno-format-truncation")

# Build lib from all source files
file(GLOB_RECURSE all_src_files src/*.c*)

add_executable(${PROJECT_NAME}_node
    ${all_src_files}
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   modal_pipe
   modal_json
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

