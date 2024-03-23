## start with 18.04 bionic to match 865 image
FROM --platform=linux/amd64 ubuntu:18.04


## new primary sources list with some tweaks
COPY sources.list /etc/apt/sources.list
## add the older xenial universe and main repos for gcc 4.9
COPY xenial-sources.list /etc/apt/sources.list.d


# update base packages in noninteractive mode
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get -y update
RUN apt-get -y install apt-utils sudo
RUN apt-get -y upgrade


# basic dev tools
RUN apt-get -y install build-essential make cmake sudo curl unzip gcc wget git nano vim xterm
# 32-bit cross compiler for 820
RUN apt-get -y install libc6-armel-cross libc6-dev-armel-cross binutils-arm-linux-gnueabi
RUN apt-get -y install gcc-4.9-arm-linux-gnueabi g++-4.9-arm-linux-gnueabi
# 64-bit cross compiler for 820
RUN apt-get -y install gcc-4.9-aarch64-linux-gnu g++-4.9-aarch64-linux-gnu
# these are misc things we need for building the kernel
RUN apt-get -y install gawk gperf help2man texinfo gperf bison flex texinfo make libncurses5-dev python-dev
# these are required to build opkg
RUN apt-get -y install libtool libtool-bin autoconf automake pkg-config libcurl4-openssl-dev openssl libssl-dev libgpgme11 libgpgme-dev
# opkg needs at least v3.2 of libarchive
RUN apt-get -y install libarchive-dev

# add glibc2.23 files for cross-compiling
ADD glibc_2.23_files.tar.xz /tmp/
RUN mv /tmp/glibc_2.23_files/* /usr/

# Setup to allow multiarch for apt package installations
COPY arm-cross-compile-sources.list /etc/apt/sources.list.d
RUN dpkg --add-architecture arm64

# update with the new architectures
RUN apt-get -y update

# clean up the package cache to save space
RUN apt-get -y clean

# build and install opkg 0.4.3
RUN mkdir -p /opt/workspace/
RUN cd /opt/workspace/
RUN cd /opt/workspace/ && git clone https://git.yoctoproject.org/git/opkg
RUN cd /opt/workspace/opkg/ && git checkout tags/v0.4.3
RUN cd /opt/workspace/opkg/ && ./autogen.sh
RUN cd /opt/workspace/opkg/ && ./configure --sysconfdir=/etc
RUN cd /opt/workspace/opkg/ && make -j$(nproc)
RUN cd /opt/workspace/opkg/ && make install
RUN ldconfig

# cleanup the opkg source
RUN rm -rf /opt/workspace/opkg

# install our opkg config file
ADD opkg.conf /etc/opkg/opkg.conf

# update with the new architectures
RUN apt-get -y update

# clean up the package cache to save space
RUN apt-get -y clean

# use opkg to install the qualcomm-proprietary package
RUN apt-get -y install rsync
ADD qualcomm-proprietary_0.0.2.ipk /tmp/
RUN opkg install /tmp/qualcomm-proprietary_0.0.2.ipk


# add gcc 7 and 8. 7 is recommended for 865, 8 is for fun
RUN apt-get -y install gcc-7-aarch64-linux-gnu g++-7-aarch64-linux-gnu
RUN apt-get -y install gcc-8-aarch64-linux-gnu g++-8-aarch64-linux-gnu

# clean package archive to save space at the end
RUN apt-get -y clean

# update cmake to a newer version
ADD update_cmake.sh /tmp/
RUN /bin/bash /tmp/update_cmake.sh

# add our toolchain files
ADD arm-gnueabi-4.9.toolchain.cmake /opt/cross_toolchain/
ADD aarch64-gnu-4.9.toolchain.cmake /opt/cross_toolchain/
ADD aarch64-gnu-7.toolchain.cmake /opt/cross_toolchain/
ADD aarch64-gnu-8.toolchain.cmake /opt/cross_toolchain/

# V2.0
# Add the qrb proprietary to a local apt repo so projects can install it if they wish
ADD qrb5165-proprietary*.deb /data/offline_deb_packages/
RUN cd /data/offline_deb_packages/ && dpkg-scanpackages . /dev/null | gzip -9c > Packages.gz
RUN echo "deb [trusted=yes] file:/data/offline_deb_packages/ ./" > /etc/apt/sources.list.d/local.list
RUN apt-get update

# Add the apq proprietary to a local file so projects can install it if they wish
# We can't setup a local opkg repo like we do for apt because opkg doesn't play nice
ADD apq8096-proprietary*.ipk /data/offline_ipk_packages/

# add bash stuff
ADD bash_utilities.tar /

# add nettools
RUN apt-get -y install net-tools sshpass iputils-ping
RUN apt-get -y clean

# V2.1
# add python3
RUN apt-get update
RUN apt-get -y install python3
RUN apt-get -y clean

# V2.2
# Add the qrb royale/spectre to a local apt repo so projects can install it if they wish
# Also this release: updated apq proprietary to 0.0.3 with royale/spectre pulled from apq system image
ADD royale*.deb /data/offline_deb_packages/

# V2.3 add a meta package voxl-cross to provide things that enable
# installing build deps
ADD voxl-cross*.deb /data/offline_deb_packages/
RUN dpkg -i /data/offline_deb_packages/voxl-cross*

# V2.4 update apq proprietary to 0.0.4 with omx headers

# V2.5 update qrb proprietary to 0.0.3 with omx headers

# v2.7 add new royale lib
RUN cd /data/offline_deb_packages/ && dpkg-scanpackages . /dev/null | gzip -9c > Packages.gz
RUN cd /data/offline_deb_packages/ && dpkg-scanpackages . /dev/null > Packages
RUN echo "deb [trusted=yes] file:/data/offline_deb_packages/ ./" > /etc/apt/sources.list.d/local.list
RUN apt-get update
