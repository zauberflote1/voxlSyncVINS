# qrb5165-imu-server

Provides voxl-imu-server virtual package.

Supports MPU9250, ICM20948, and ICM42688 IMUs.

This README covers building this package. For usage, see docs.modalai.com


## Build dependencies

* libmodal-json
* libmodal-pipe
* librc-math
* voxl-cpu-monitor
* libqrb5165-io

## Build Environment

This project builds in the voxl-cross docker image (>= V1.8)

Follow the instructions here to build and install the voxl-cross docker image:
https://gitlab.com/voxl-public/voxl-docker



## Build Instructions

1) Launch the voxl-cross docker.

```bash
~/git/qrb5165-imu-server$ voxl-docker -i voxl-cross
voxl-cross:~$
```

2) Install dependencies inside the docker. You must specify both the hardware platform and binary repo section to pull from. CI will use the `dev` binary repo for `dev` branch jobs, otherwise it will select the correct target SDK-release based on tags. When building yourself, you must decide what your intended target is, usually `dev` or `stable`

```bash
voxl-cross:~$ ./install_build_deps.sh qrb5165 dev
```


3) Build scripts should take the hardware platform as an argument: `native` or `qrb5165`. CI will pass these arguments to the build script based on the job target.

```bash
voxl-cross:~$ ./build.sh qrb5165
```


5) Make an ipk or deb package while still inside the docker.

```bash
voxl-cross:~$ ./make_package.sh ipk
```

This will make a new package file in your working directory. The name and version number came from the package control file. If you are updating the package version, edit it there.



## Deploy to VOXL

You can now push the ipk or deb package to VOXL and install with dpkg/opkg however you like. To do this over ADB, you may use the included helper script: deploy_to_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.

The deploy_to_voxl.sh script will query VOXL over adb to see if it has dpkg installed. If it does then then .deb package will be pushed an installed. Otherwise the .ipk package will be installed with opkg.

```bash
(outside of docker)
apq8096-imu-server$ ./deploy_to_voxl.sh
```

This deploy script can also push over a network given sshpass is installed and the VOXL uses the default root password.


```bash
(outside of docker)
apq8096-imu-server$ ./deploy_to_voxl.sh ssh 192.168.1.123
```
