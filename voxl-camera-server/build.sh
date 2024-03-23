#!/bin/bash
################################################################################
# Copyright (c) 2020 ModalAI, Inc. All rights reserved.
################################################################################

## voxl-cross contains the following toolchains
## first two for apq8096, last for qrb5165
TOOLCHAIN_APQ8096_32="/opt/cross_toolchain/arm-gnueabi-4.9.toolchain.cmake"
TOOLCHAIN_APQ8096_64="/opt/cross_toolchain/aarch64-gnu-4.9.toolchain.cmake"
TOOLCHAIN_QRB5165="/opt/cross_toolchain/aarch64-gnu-7.toolchain.cmake"

## this list is just for tab-completion
AVAILABLE_PLATFORMS="qrb5165 apq8096"


print_usage(){
    echo ""
    echo " Build the current project based on platform target."
    echo ""
    echo " Usage:"
    echo ""
    echo "  ./build.sh apq8096"
    echo "        Build 32-bit binaries for apq8096"
    echo ""
    echo "  ./build.sh qrb5165"
    echo "        Build 64-bit binaries for qrb5165"
    echo ""
    echo ""
}


case "$1" in
    apq8096)
        ## single-arch library, build only 32 bit
        mkdir -p build
        cd build
        #cmake -DPLATFORM=APQ8096 ${EXTRA_OPTS} ../
        cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_APQ8096_32} -DPLATFORM=APQ8096 ${EXTRA_OPTS} ../
        make -j$(nproc)
        cd ../
        ;;
    qrb5165)
        ## single-arch library, build just 64
        mkdir -p build
        cd build
        cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165}  -DPLATFORM=QRB5165 ${EXTRA_OPTS} ../
        make -j$(nproc)
        cd ../
        ;;

    *)
        print_usage
        exit 1
        ;;
esac
