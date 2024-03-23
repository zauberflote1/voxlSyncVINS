## This is the recommended toolchain for building for 865/QRB5165 platforms
## in a cross-compile environment like voxl-cross

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ubuntu cross compiler installs straight to /usr/bin/
set(tools /usr)
set(CMAKE_C_COMPILER ${tools}/bin/aarch64-linux-gnu-gcc-7)
set(CMAKE_CXX_COMPILER ${tools}/bin/aarch64-linux-gnu-g++-7)

# set architecture
set(CMAKE_C_FLAGS   "-march=armv8.2-a ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-march=armv8.2-a ${CMAKE_CXX_FLAGS}")

