# voxlSyncVINS
## VOXL2 Package Edits for proper external VINS usage (ROS) 
This a modified fork from the original ModalAI Gitlab. Please refer to their repo for proper documentation.
### Modifications
- Removed Subscriber Count for Publishing/ThreadManager Simplification
- Ensured Timestamps were in order for both IMU and Stereo Interface (NOT Artificially)
- Reduced tolerance difference between Stereo images at the driver level
### Future Development
- ROS2 Mods
- Improve Software Sync of Stereo Images for combo OV sensor
- Improve VOXL2 IMU driver Timestamp ordering and stabilize sampling rate (not PX4 IMU) 

