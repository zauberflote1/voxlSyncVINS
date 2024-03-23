# See https://github.com/tianon/docker-brew-ubuntu-core/blob/f6c798bd7248f69db272e17677153fc119f41301/bionic/Dockerfile

# 'scratch' is an empty docker image
FROM scratch

# add the rootfs contents from the system image build output
ADD data/rootfs /

# required for ARM emulation
COPY ./bin/qemu-arm-static /usr/bin/qemu-arm-static

# fix permissions on /tmp to let apt update run
RUN chmod 1777 /tmp

# remove old apt line from rootfs
RUN rm -f /etc/apt/sources.list.d/modalai.list

# install helpers
RUN apt-get -y update
RUN apt-get -y install git cmake sudo nano

# remove opencv
RUN apt-get -y remove libopencv-dev
RUN apt-get -y autoremove

# install common dependencies
RUN apt-get -y install libusb-1.0-0 libusb-1.0-0-dev

# 32-bit cross compiler for mv-based things
RUN apt-get -y install g++-7-multilib-arm-linux-gnueabi gcc-7-multilib-arm-linux-gnueabi

# Install GStreamer RTSP server development files since they aren't included
# with the rootfs
RUN apt-get -y install libgstrtspserver-1.0-dev

# Install a file that indicates what docker image this is
# so that it can be known inside the container
RUN mkdir -p /etc/modalai
RUN touch /etc/modalai/qrb5165-emulator.id

RUN apt-get install -y git build-essential autoconf automake autopoint libtool pkg-config -y
RUN apt-get install -y gtk-doc-tools libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev -y
RUN apt-get install -y checkinstall


# # Change the prompt
# RUN echo PS1="\"\[\e[1m\]\[\e[33m\]qrb5165-emulator\[\e[0m\]:\[\e[1m\]\[\e[34m\]\w\[\e[0m\]$ \"" > /root/.bashrc
# RUN echo 'export PS1="\[\e[1m\]\[\e[33m\]qrb5165-emulator\[\e[0m\]:\[\e[1m\]\[\e[34m\]\w\[\e[0m\]$ "' >> /root/.bash_profile

# fix permissions on /usr/lib32 stuff
RUN chmod 755 /usr/lib32/*
RUN chmod 755 /lib/ld-linux.so.3
RUN chmod 755 /opt/qcom-licenses/snapdragon-flight-license.bin


# add ROS stuff for mpa-to-ros
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update
RUN apt install -y ros-melodic-ros-base
RUN apt install -y ros-melodic-camera-info-manager
RUN apt install -y ros-melodic-tf2-ros
RUN apt install -y ros-melodic-catkin
RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN rosdep init
RUN rosdep update


# add bash stuff
ADD bash_utilities.tar /
RUN echo "qrb5165-emulator" > /etc/modalai/image.name


# clean package archive to save space at the end
RUN apt -y upgrade
RUN apt-get -y clean

# start in /root
WORKDIR /root/

CMD ["/bin/bash"]
