# voxl_mpa_to_ros

ROSNode that takes in mpa data and published it to ROS

## Build Instructions

1. Requires the voxl-emulator (found [here](https://gitlab.com/voxl-public/support/voxl-docker)) to run docker ARM image
    * (PC) ```cd [Path To]/voxl-mpa-to-ros```
    * (PC) ```sudo voxl-docker -i voxl-emulator```
2. Build project binary:
    * (VOXL-EMULATOR) ```./install_build_deps.sh apq8096 stable```
    * (VOXL-EMULATOR) ```./clean.sh```
    * (VOXL-EMULATOR) ```./build.sh```
    * (VOXL-EMULATOR) ```./make_package.sh ipk```


### Installation
Install mpa-to-ros by running (VOXL):
```
opkg install voxl-mpa-to-ros
```
or (VOXL2/RB5):
```
apt install voxl-mpa-to-ros
```

If you're running a version of the voxl sdk <= 0.5.0 this package was found under voxl-nodes and can be installed with:
```
opkg install voxl-nodes
```

### Start Installed MPA ROS Node

Run the following commands(on voxl):

Verify your ros environment with:
```
vi ~/my_ros_env.sh
```

if you make any changes make sure to run ```exec bash``` to re-source the file

and then start the node with:

```
roslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launch
```

##### Supported Interfaces
The current supported mpa->ros translations are:  

-cameras from voxl-camera-server or any services that publish an overlay

-IMUs

-VIO data from voxl-qvio-server (the data will appear under the qvio name, but it is normal vio data)  

-Point Clouds from the TOF sensor or services providing point clouds such as voxl-dfs-server

-AI Detections from voxl-tflite-server

### Expected Behavior
```
voxl:/$ roslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launch
... logging to /home/root/.ros/log/5ef19600-cbd7-11ec-8a51-ec5c68cd23bd/roslaunch-apq8096-3468.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.1.58:46256/

SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.21

NODES
  /
    voxl_mpa_to_ros_node (voxl_mpa_to_ros/voxl_mpa_to_ros_node)

auto-starting new master
process[master]: started with pid [3487]
ROS_MASTER_URI=http://192.168.1.58:11311/

setting /run_id to 5ef19600-cbd7-11ec-8a51-ec5c68cd23bd
process[rosout-1]: started with pid [3500]
started core service [/rosout]
process[voxl_mpa_to_ros_node-2]: started with pid [3503]


MPA to ROS app is now running

Found new interface: stereo
Found new interface: tof_conf
Found new interface: tof_depth
Found new interface: tof_ir
Found new interface: tof_noise
Found new interface: tracking
Found new interface: tof_pc

```
