#******************************************************************************
# Copyright 2023 ModalAI Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# 4. The Software is used solely in conjunction with devices provided by
#    ModalAI Inc.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************/

project(QRB5165-camera-server)
cmake_minimum_required(VERSION 3.3)
set(SERVERNAME voxl-camera-server)



option(PLATFORM "platform APQ8096 or QRB5165" NONE)

if(PLATFORM MATCHES APQ8096)
    message(STATUS "Building for platform APQ8096")
    add_definitions(-DAPQ8096 )
elseif(PLATFORM MATCHES QRB5165)
    message(STATUS "Building for platform QRB5165")
    add_definitions(-DQRB5165 )
else()
    message(FATAL_ERROR "Platform not specified")
endif()



set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -fpermissive -L/usr/lib -Wl,--unresolved-symbols=ignore-in-shared-libs \
    -pthread -Wall -Wno-unused-function -Wno-format-overflow \
    -Wno-delete-non-virtual-dtor ${CMAKE_CXX_FLAGS}")

set(CMAKE_C_FLAGS "-std=gnu99 -O3 -Wall -Wextra -Wuninitialized \
    -Wunused-variable -Wdouble-promotion -Wmissing-prototypes \
    -Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")



# We know what we're doing
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format-truncation -Wno-pointer-arith")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-format-truncation -Wno-pointer-arith")


if(PLATFORM MATCHES QRB5165)
set(CMAKE_CXX_FLAGS "-L/usr/lib64 ${CMAKE_CXX_FLAGS}")
endif()



## no libexif support on voxl1
file(GLOB_RECURSE all_src_files "src/*.c*")
if(PLATFORM MATCHES APQ8096)
    list(FILTER all_src_files EXCLUDE REGEX ".*gps_pose_subscriber\\.cpp$")
endif()
add_executable(${SERVERNAME} ${all_src_files})

include_directories(
    include/
    include/hal3/
    include/config/
    include/common/
    include/omx/
    include/tof-interface/
    /usr/include/royale/
)

if(PLATFORM MATCHES QRB5165)
    target_link_libraries(${SERVERNAME} /usr/lib/libgbm.so)
endif()

target_link_libraries(${SERVERNAME}
    dl
    modal_pipe
    modal_journal
    modal_exposure
    modal_json
    voxl_cutils
    camera_metadata
    hardware
    platform
)

if(PLATFORM MATCHES APQ8096)

    target_link_libraries(${SERVERNAME}
        camera_client
        tof-interface
        spectre3
        log
    )
else()
    find_library(LIBEXIF_SO NAMES libexif.so HINTS /usr/lib/aarch64-linux-gnu)
    target_link_libraries(${SERVERNAME}
        voxl_cci_direct
        spectre
        cutils
        ${LIBEXIF_SO} 
    )
endif()

# install everything!
install(
    TARGETS ${SERVERNAME}
    LIBRARY         DESTINATION /usr/lib
    RUNTIME         DESTINATION /usr/bin
    PUBLIC_HEADER   DESTINATION /usr/include
    )


add_subdirectory(tools)
