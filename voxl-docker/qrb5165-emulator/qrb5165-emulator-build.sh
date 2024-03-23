#!/bin/bash

echo "[INFO] Building qrb5165-emulator docker image"

IMAGE_NAME="qrb5165-emulator"
IMAGE_TAG="1.3"

if ! [[ -d data/rootfs ]]; then
    echo "[ERROR] Missing rootfs.  See README for instructions"
    exit 1
fi

docker build \
--platform linux/arm64/v8 \
--tag "${IMAGE_NAME}:${IMAGE_TAG}" -f qrb5165-emulator.Dockerfile .

docker tag ${IMAGE_NAME}:${IMAGE_TAG} ${IMAGE_NAME}:latest

echo "[INFO] Done"
