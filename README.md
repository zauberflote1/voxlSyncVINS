
# voxlSyncVINS
## VOXL2 Package Edits for proper external VINS usage (ROS) 
This a modified fork from the original ModalAI Gitlab. Please refer to their repo for proper documentation.
### Modifications
- Removed Subscriber Count for Publishing/ThreadManager Simplification (mpa_to_ros)
- Reduced interfaces to only cameras (mpa_to_ros)
- Ensured Timestamps were in order for both IMU and Stereo Interface (NOT Artificially)
- Reduced tolerance difference between Stereo images at the driver level
- Removed FIFO buffer from the IMU driver, read and pipe each sample (Note that ICM-42688 Interface is 24 MHz SPI)
- Changed sleep configuration post-reading a sample for 1000us (0us reaches close to theoretical max at significant CPU usage increase) 
- Enable locking a IMU operations to a CPU specified core (Jitter reduction)
- Convert Hires IMX images from YUV to BGR, so the topic can be seen in RViz directly. (This is optional but reduces processing and debugging effort.)
### Future Development
- ROS2 Mods
- Improve VOXL2 IMU driver Timestamp ordering and stabilize sampling rate (not PX4 IMU) 
- Improve IMU_ROS package to support QVIO (Add ModalPipe, FIFO patch, timestamp filtering etc...)


### Building, Running, and Considerations
Currently this repo has 2 ROS packages and 1 "VOXL" package, which are the following:
- voxl_imu_ros -> Substitutes voxl-imu-server, controlling the Linux user interface to DSP directly via ROS. Although, the some frequency jitter might be present, it is compatible with Kalibr and OpenVins. (Currently not compatible with QVIO.)
- voxl_mpa_ros_mod -> Newer versions of voxl_mpa_ros released by ModalAI are probably sufficient for most applications. The goal of this modified package is to cleanup the interfaces and change the encoding of the Hires Camera 
- voxl-camera-server-zbft -> Reduces stereo discrepancy to 5ms

### VOXL_IMU_ROS
I recommend building this natively on your QRB powered drone, but the qrb-emulator should work if you have the patience to set it up. Additionally, you cannot run voxl-imu-server and the voxl_imu_ros at the same time, so make sure to stop one before starting the other. Finally, because this has a custom priority so the kernel scheduler prioritizes the read SPI operations, you WILL need to kill the process via terminal (see below...)

````bash
#IMU_ROS IS ALREADY A CATKIN_WS-LIKE, SO DRAG IT TO A DIRECTORY SUCH AS /HOME
cd imu_ros
catkin_make ##assuming you sourced ROS already...
source devel/setup.bash
rosrun voxl_imu_ros server_node_node ##MAKE SURE ROSCORE IS RUNNING BEFORE THIS LINE
````

#### Check if it is working (another terminal):

````bash 
rostopic echo /imu_data
````


#### Killing the node:

````bash
top ##grab PID for the imu_ros, then ctrl+c
kill -9 $PID #substitute $PID by the PID in top
````


#### Locking thread to a core and IMU ODR:
- (Inside node.cpp) We current lock the IMU operations to core 7, the fastest core in VOXL2, but you can easily change the core or deactivate this. --  clear instructions on how to do this are commented in the code.
- Currently using 200 Hz as ODR but the IMU clock is a bit faster than originally intended by the manufacturer, so we get something around 204Hz. Changing the ODR is simple, make sure to visit icm42688.c and check the TDK datasheet for ICM 42688-p, I left some instructions commented.

### VOXL_MPA_ROS_MOD
Follow the same building process as the IMU_ROS package, without the hassles of killing the process via terminal

````bash 
cd voxl_mpa_ros_mod_ws
catkin_make
source devel/setup.bash
roslaunch voxl_mpa_to_ros_mod voxl_mpa_to_ros.launch 
````
### VOXL-CAMERA-SERVER
I recommend building via VOXL-Docker (VOXL-CROSS) and deploying the .deb via ssh or adb. Although, this is a single line modification from the original code. The most important modifications for VINS are in the .conf file (stereo global shutter exposure, gain, dimensions settings) -- you can check my configurations and substitute as you please in /etc/modalai/ (QRB)

#### Clone the repo from ModalAI
Make sure to pick the one that aligns with your SDK version

````cpp
##CHANGE THIS LINE IN hal3_camera_mgr.cpp

#define MAX_STEREO_DISCREPENCY_NS ((1000000000/configInfo.fps)*0.9)

##TO THIS

#define MAX_STEREO_DISCREPENCY_NS (5000000)
````



