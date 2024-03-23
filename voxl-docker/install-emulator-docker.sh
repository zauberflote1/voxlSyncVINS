#!/bin/bash
################################################################################
# Copyright 2019 ModalAI Inc.
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
# 4. The Software is licensed to be used solely in conjunction with devices
#    provided by ModalAI Inc.
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
################################################################################

IMAGE=voxl-emulator

# count files in current directory matching image anme
NUM_FILES=$(ls -1q $IMAGE* | wc -l)

if [ $NUM_FILES -eq "0" ]; then
    echo "ERROR: no voxl-emulator tar/tgz found"
    exit 1
elif [ $NUM_FILES -gt "1" ]; then
    echo "ERROR: more than 1 voxl-emulator image tar found"
    echo "make sure there is only one in the current directory"
    exit 1
fi

# now we know only one file exists
IMAGE_TAR=$(ls -1q $IMAGE*)

RUN_SCRIPT=voxl-docker

# check prerequisites
command -v docker > /dev/null
RETVAL=$?
if [ $RETVAL -ne "0" ]; then
	echo "ERROR: docker executable not found."
	echo "follow instructions in README.md"
	exit 1
fi

set -e

# make sure user has downlaoded the image file
if [ ! -f $IMAGE_TAR ]; then
	echo "Docker image tar not found"
	echo "please download from the ModalAI developer network"
	echo "and place the file in the current directory"
	echo "https://developer.modalai.com/asset/eula-download/3"
	exit 1
fi

echo "installing misc. dependencies"
# qemu-user-static must be installed AFTER binfmt-support for it to install properly
sudo apt install binfmt-support
sudo apt install qemu-user-static android-tools-adb android-tools-fastboot

echo "loading docker image"
sudo docker load -i $IMAGE_TAR

echo "tagging image as latest"
for i in $(docker images | awk '{if (NR!=1) {print $1":"$2","$3}}'); do
    ID=$(echo $i | cut -d',' -f2)
    TAG=$(echo $i | cut -d',' -f1)
    if [[ "${TAG}" == "${IMAGE}"* ]]; then
        docker tag $ID ${IMAGE}:latest
        break;
    fi
done



echo "installing ${RUN_SCRIPT}.sh to /usr/local/bin/${RUN_SCRIPT}"
sudo install -m 0755 files/${RUN_SCRIPT}.sh /usr/local/bin/${RUN_SCRIPT}

echo "DONE"
