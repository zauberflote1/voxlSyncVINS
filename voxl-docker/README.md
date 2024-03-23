# voxl-docker

This project provides the tools needed to build the three docker images used for compiling VOXL packages.

You do not need to build these yourself, you can download the pre-built images from https://downloads.modalai.com

For overview of build environments and how to use them, go to https://docs.modalai.com/build-environments/



## voxl-docker bash script

This repo contains the "voxl-docker" bash script for easily launching docker images with the right permissions and with the local directory mounted in the docker for easy compilation. Most VOXL projects use this script in their build instructions.

Install this by running:

```
~/git/voxl-docker$ ./install-voxl-docker-script.sh
```


## voxl-emulator

voxl-emulator simulates the VOXL system image and is built alongside the base system image. voxl-emulator can only build 32-bit ARM binaries using the native ARM gcc. It is an ARM docker image that runs on x86 using qemu as an emulator, it is therefore slower than voxl-cross which is native x86, but has the advantage of emulating the VOXL sysroot.

This must be downloaded from https://downloads.modalai.com and cannot be built by itself without rebuiling the entire system image from source.

## qrb5165-emulator

qrb5165-emulator simulates the qrb5165 system image and is built alongside the base system image. qrb5165-emulator builds 64-bit ARM binaries using the native ARM gcc. It is an ARM docker image that runs on x86 using qemu as an emulator, it is therefore slower than voxl-cross which is native x86, but has the advantage of emulating the qrb5165 platform sysroot.

More details can be found [here](qrb5165-emulator/README.md)

## How to Build voxl-cross

voxl-cross is an ubuntu-based docker image containing arm 32 and 64-bit cross compilers for building arm code for VOXL. Most newer VOXL packages use this, and new ones should try to too. It includes the opkg package manager which allows voxl packages to be installed in the docker image for easy dependency satisfaction when compiling.

Note you don't need to build this yourself, you can just download it from https://downloads.modalai.com

To build this yourself:

1) download the following packages from https://downloads.modalai.com and place the voxl-cross directory.

	- qualcomm-proprietary_0.0.1.ipk
	- apq8096-proprietary_0.0.3.ipk
	- qrb5165-proprietary_0.0.2_arm64.deb
	- royale-331-spectre-4-7_1.1.0_arm64.deb

2) run the install-cross-docker.sh script

```
~/git/voxl-docker$ ./install-cross-docker.sh
```

To compress a docker image for export after it's built:

```
docker save voxl-cross:V2.3 | gzip > voxl-cross_V2.3.tgz
```

## How to build voxl-hexagon

Like voxl-cross, voxl-hexagon is an ubuntu based docker image, but contains the hexagon SDSP sdk for building SDSP projects like libvoxl_io. It also contains the opkg package manager for installing build dependencies.

Note you don't need to build this yourself, you can just download it from https://downloads.modalai.com

To build this yourself:

1) Download and place the following files into cross_toolchain/downloads

Hexagon SDK 3.1 install file: qualcomm_hexagon_sdk_3_1_eval.bin

* [downloads.modalai.com](downloads.modalai.com)

Linaro ARM compiler binaries: gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabi.tar.xz

* [https://releases.linaro.org/archive/14.11/components/toolchain/binaries/arm-linux-gnueabi](https://releases.linaro.org/archive/14.11/components/toolchain/binaries/arm-linux-gnueabi)

2) Run the install-hexagon-docker.sh script. This will also install voxl-docker.sh to usr/bin/ as did install-emulator-docker.sh in case the user wants the hexagon docker only.

```bash
./install-hexagon-docker.sh
```
