# qrb5165-emulator

## Summary

The `qrb5165-emulator` is an armv8 Ubuntu 18 Docker image that runs on your Linux PC to emulate qrb5165 based platforms such as RB5 Flight.

This project takes the rootfs output of the system image build (download below) and builds it on top of an armv8 Docker 'scratch' image. This provides developers an off-target workflow for developing for the qrb5165.

For example, we use this image in our build servers for CI.

## Requirements

- Ubuntu 18.04 host (other versions may work, but have not been tested)
- Docker
- QEMU, to install:

```bash
# install packages
sudo apt-get install qemu binfmt-support qemu-user-static

# Execute the registering scripts, see https://www.stereolabs.com/docs/docker/building-arm-container-on-x86/
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## Usage

- Download the `1.1.1-M0054-14.1a-perf-rootfs.tar` image from <https://developer.modalai.com/>
- Copy the file into the `data` directory, for example, assuming the file was downloaded to `Downloads`:


```bash
cd qrb5165-emulator
cp ~/Downloads/1.1.1-M0054-14.1a-perf-rootfs.tar data
```

- Unzip into a new `rootfs` directory:

```
cd qrb5165-emulator/data
mkdir rootfs
tar -xzvf 1.1.1-M0054-14.1a-perf-rootfs.tar -C rootfs
```

The version (e.g. `1.1.1`) should match the `IMAGE_TAG` variable in the `qrb5165-docker-build.sh` script.

Now, build the image by running the following:

```bash
cd ../
./qrb5165-emulator-build.sh
```

*Note:* if you get failures, try running this again:

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```
