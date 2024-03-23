#!/bin/bash

set -e

VERSION="2.7"
RUN_SCRIPT=voxl-docker


CLEAN=""
if [ "$1" == "clean" ]; then
	CLEAN="--no-cache"
	echo "starting clean build"
fi

# Check required files exist
REQUIRED_FILES=(
    "qualcomm-proprietary_0.0.2.ipk"
    "apq8096-proprietary_0.0.4.ipk"
    "qrb5165-proprietary_0.0.4_arm64.deb"
    "royale-331-spectre-4-7_1.1.0_arm64.deb"
    "royale-5-8-spectre-6-10_0.0.1_arm64.deb"
)
ERROR=

for FILE in ${REQUIRED_FILES[@]} ; do
    if [ ! -f "voxl-cross/$FILE" ] ; then
        echo "Missing: $FILE"
        ERROR=1
    fi
done

if [ $ERROR ] ; then
    echo
    echo "Please following the instruction in the README to download"
    echo "these files from downloads.modalai.com and place in voxl-cross/"
    exit 1
fi

# build the provides meta package
cd voxl-cross
DIR="cross_meta_pkg"
sed -i "s/Version.*/Version: ${VERSION}/g" ${DIR}/DEBIAN/control
echo -n "voxl-cross(${VERSION})" > ${DIR}/etc/modalai/image.name
PKG_NAME=$(cat $DIR/DEBIAN/control | grep "Package" | cut -d' ' -f 2)
DEB_NAME=${PKG_NAME}_${VERSION}_arm64.deb
dpkg-deb --build ${DIR} ${DEB_NAME}
cd ../


# Add bash utilities
cd bash_utilities
./make_package.sh
cd ..
cp ./bash_utilities/bash_utilities.tar voxl-cross/

# Build Docker image
cd voxl-cross
docker build $CLEAN -t voxl-cross:V${VERSION} -f voxl-cross.Dockerfile .
docker tag voxl-cross:V${VERSION} voxl-cross:latest
cd ../

# install the voxl-docker helper script
echo "installing ${RUN_SCRIPT}.sh to /usr/local/bin/${RUN_SCRIPT}"
sudo install -m 0755 files/${RUN_SCRIPT}.sh /usr/local/bin/${RUN_SCRIPT}

echo "DONE"
