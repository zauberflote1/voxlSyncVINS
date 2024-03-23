#!/bin/bash

echo "[INFO] Building qrb5165-emulator docker image"

IMAGE_NAME="qrb5165-emulator"
IMAGE_TAG="1.4"

if ! [[ -d qrb5165-emulator/data/rootfs ]]; then
    echo "[ERROR] Missing rootfs.  See README for instructions"
    exit 1
fi


CLEAN=""
if [ "$1" == "clean" ]; then
	CLEAN="--no-cache"
	echo "starting clean build"
fi

# Add bash utilities
cd bash_utilities
./make_package.sh
cd ..
cp ./bash_utilities/bash_utilities.tar qrb5165-emulator/

# Build Docker image
cd "qrb5165-emulator/"


docker build $CLEAN \
--platform linux/arm64/v8 \
--tag "${IMAGE_NAME}:${IMAGE_TAG}" -f qrb5165-emulator.Dockerfile .

docker tag ${IMAGE_NAME}:${IMAGE_TAG} ${IMAGE_NAME}:latest
cd ../

# Clean the bash utilities file
rm qrb5165-emulator/bash_utilities.tar

echo "[INFO] Done"
