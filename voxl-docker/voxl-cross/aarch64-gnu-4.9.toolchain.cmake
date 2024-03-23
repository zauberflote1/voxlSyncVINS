## This is the recommended toolchain for building 64-bit programs for 820/voxl1
## projects in a cross-compile environment like voxl-cross. It is the same
## gcc version 4.9 as found on the system image. A specific glibc is necessary
## for compatability with this old compiler and is included in voxl-cross.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ubuntu cross compiler installs straight to /usr/bin/
set(tools /usr)
set(CMAKE_C_COMPILER ${tools}/bin/aarch64-linux-gnu-gcc-4.9)
set(CMAKE_CXX_COMPILER ${tools}/bin/aarch64-linux-gnu-g++-4.9)


## glibc 2.23 is off in a separate location in voxl-cross to avoid conflicting
## with the new gcc7 compiler stuff which is in the default location
set(LOC /usr/aarch64-linux-gnu-2.23)
set(CMAKE_C_FLAGS   "-march=armv8-a -L ${LOC}/lib -I ${LOC}/include ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-march=armv8-a -L ${LOC}/lib -I ${LOC}/include ${CMAKE_CXX_FLAGS}")

