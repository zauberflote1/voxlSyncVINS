#!/bin/bash

RUN_SCRIPT=voxl-docker
HEXAGON_BIN=qualcomm_hexagon_sdk_3_1_eval.bin
GCC_BIN=gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabi.tar.xz

# check files have been downloaded
if [ ! -f cross_toolchain/downloads/$HEXAGON_BIN ]; then
	echo "Hexagon SDK not found"
	echo "please download to cross_toolchain/downloads/"
	echo "https://developer.qualcomm.com/download/hexagon/hexagon-sdk-v3-1-linux.bin"
	exit 1
fi
if [ ! -f cross_toolchain/downloads/$GCC_BIN ]; then
	echo "GCC cross compiler not found ($GCC_BIN)"
	echo "please download to cross_toolchain/downloads/"
	echo "https://releases.linaro.org/archive/14.11/components/toolchain/binaries/arm-linux-gnueabi"
	exit 1
fi

# pull in dspal submodule
git submodule update --init

# Build Docker image, UPDATE THIS VERSION NAME IF YOU REV THE DOCKERFILE
docker build -t voxl-hexagon:V1.1 -f files/voxl-hexagon.Dockerfile .
docker tag voxl-hexagon:V1.1 voxl-hexagon:latest

echo "installing ${RUN_SCRIPT}.sh to /usr/local/bin/${RUN_SCRIPT}"
sudo install -m 0755 files/${RUN_SCRIPT}.sh /usr/local/bin/${RUN_SCRIPT}

echo "DONE"
