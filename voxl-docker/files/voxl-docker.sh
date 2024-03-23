#!/bin/bash
################################################################################
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
################################################################################

function printUsage() {
	cat <<EOF
voxl-docker helper tool V1.2

Usage: $(basename $0) [ARGUMENTS]
example: voxl-docker -i voxl-emulator

This is primarily intended as an assistant for running the voxl-emulator,
qrb5165-emulator, voxl-cross, and voxl-hexagon docker images for compiling ModalAI projects.
However this can also launch any other installed docker images. It incorperates
the long list of arguments normally necessary to make 'docker run' behave to
make using docker images for compiling easier and faster.

There are a few basic arguments to retain a small amount of fexibility.

By default this mounts the current working directory as the home directory
(/home/root or /home/user) inside the docker for easy compilation of whichever project you
are currently working on. The directory that gets mounted inside the docker
can be manually specified with the -d argument.

Once inside the docker image, you will run as the root user for ease of
installing build dependencies and making packages with correct file permissions.
A typical use would be to build and make the librc_math ipk package:


me@mydesktop:~/git/librc_math$ voxl-docker -i voxl-cross
using image: voxl-cross
root@my_desktop:/home/root# ./build.sh both
root@my_desktop:/home/root# ./make_package.sh
root@my_desktop:/home/root# exit
me@mydesktop:~/git/librc_math$


By default this script will start docker images in interactive mode with
/bin/bash as the entrypoint. However, this entrypoint can be configured with
the -e option. This is most likely used to pass the docker a command to execute
before exiting automatically. For example, to build the librc_math project
in one command:

~/git/librc_math$ voxl-docker -i voxl-cross -e "/bin/bash build.sh both"


ARGUMENTS:
  -h:      : Print this help message
  -d <name>: The name of the directory to mount as ~/ inside the docker
  -i <name>: Docker image to run, usually voxl-emulator, qrb5165-emulator, or voxl-hexagon
  -l       : list installed docker images
  -e       : set the entrypoint for the docker image launch
EOF
	exit 1
}

EMULATOR="voxl-emulator"

MOUNT=`pwd`			# mount current working directory by default
IMAGE=""
USER_OPTS=""
MOUNT_OPTS=""
ENTRYPOINT="/bin/bash -l" # login shell so it loads /etc/profile


# parse arguemnts (if any)
while getopts 'phd:i:le:' opt
do
	case $opt in
	h)
		printUsage
		;;
	d)
		MOUNT=$(realpath $OPTARG)
		echo "Using ${MOUNT} as home directory inside docker"
		;;
	i)
		IMAGE="$OPTARG"
		;;
	e)
		ENTRYPOINT=$OPTARG
		;;
	l)
		docker images
		exit 0
		;;
	*)
		printUsage
		;;
		esac
done

if [[ "${IMAGE}" == "" ]]; then
	printUsage
	exit 1
fi


# run as root inside the docker
USER_OPTS="-e LOCAL_USER_ID=0 -e LOCAL_USER_NAME=root -e LOCAL_GID=0"
MOUNT_OPTS="-v ${MOUNT}:/home/root:rw -w /home/root"
PLATFORM_OPTS=""
if [[ $IMAGE = "qrb5165-emulator" ]]; then
	PLATFORM_OPTS="--platform linux/arm64/v8"
fi


# Run docker with the following options:
# --rm			automatically remove container when exiting
# -i			interactive
# -t			allocate a pseudo TTY
# --name		assign a name to the container so we can remove it later
# -e			set environment variables within docker to match host user
# -v			mount desired directory
# -w			set working directory to home

cmd=(
docker run \
	--rm -it \
	--net=host \
	--privileged \
	-w /home/$(whoami) \
	--volume="/dev/bus/usb:/dev/bus/usb" \
	$USER_OPTS \
	$MOUNT_OPTS \
	$PLATFORM_OPTS \
	${IMAGE}\
	${ENTRYPOINT})

# print what's going to run
echo "launching image: $IMAGE with the following command:"
echo ${cmd[@]}
echo ""
# run it!
"${cmd[@]}"
