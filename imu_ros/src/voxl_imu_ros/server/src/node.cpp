#include <ros/ros.h>
#include <sensor_msgs/Imu.h>  
#include <boost/thread.hpp>

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <getopt.h>
#include <string.h>
#include <stdlib.h> // for atoi(), exit(), and system()
#include <sched.h>
#include <errno.h>
#include <inttypes.h>

#include <modal_start_stop.h>
#include <modal_pipe_server.h>

#include "config_file.h"
#include "cal_file.h"
#include "imu_interface.h"
#include "misc.h"
#include "fft.h"

#define PROCESS_NAME "voxl-imu-ros" // for PID file name





static void quit(int ret) {
	for(int i=0;i<N_IMUS;i++){
		imu_close(i);
	}
	remove_pid_file(PROCESS_NAME);
	if(ret==0) printf("Exiting Cleanly\n");
	exit(ret);
	return;
}

//ACORDING TO QVIO SERVER, FASTEST CORE IS 7 (QRB ONLY), REDUCE JITTER AND LOCK THREAD TO CORE
//FEEL FREE TO LOCK TO A DIFFERENT CORE OR NOT LOCK AT ALL -- SEE MAIN()
void _set_affinity_to_core_7() {
    cpu_set_t cpuset;
    pthread_t thread = pthread_self();

    //SET IT UP TO CORE 7
    CPU_ZERO(&cpuset);
    CPU_SET(7, &cpuset);
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) {
        perror("pthread_setaffinity_np");
    }

    //CHECK IF IT WAS SET UP
    if (pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) {
        perror("pthread_getaffinity_np");
    }

    ROS_INFO("IMU read thread is now locked to core: ");
    for (int j = 0; j < CPU_SETSIZE; j++) {
        if (CPU_ISSET(j, &cpuset)) {
            ROS_INFO(" %d", j);
        }
    }
    ROS_INFO("\n");
}

void readThreadFunc(int id) {
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

    while (ros::ok()) {
        imu_data_t data[1];  //Data structure to single read
        // int packets_read = 0;
        //COULD POTENTIALLY RECORD TIME HERE, AND HAVE THE DELAY ALREADY "INCLUDED" IN THE TIMESTAMP
        int ret = imu_basic_read(id, &data[0]);
        // packets_read = 1;

        if (ret) {
            continue;
        }
        ros::Time now = ros::Time::now();
        //Convert IMU data to a ROS message and publish it
        sensor_msgs::Imu imu;
        //GRAB READ DELAY
        int64_t delta_ns = data[0].timestamp_ns; 
        ros::Duration delta_t(delta_ns / 1e9);
        imu.header.stamp = now - delta_t;
        imu.angular_velocity.x = data[0].gyro_rad[0];
        imu.angular_velocity.y = data[0].gyro_rad[1];
        imu.angular_velocity.z = data[0].gyro_rad[2];
        imu.linear_acceleration.x = data[0].accl_ms2[0];
        imu.linear_acceleration.y = data[0].accl_ms2[1];
        imu.linear_acceleration.z = data[0].accl_ms2[2];
        imu_pub.publish(imu);

        // Real-time considerations apply --> WE SET THE IMU TO 200HZ (~204 DUE TO CLOCK SPEED)
        //BUT WE GO FASTER TO AVOID DATA LOSS DUE TO SCHDULER DELAYS, ALSO WE AVOID REPEATED DATA BY CHECKING THE INTERRUPT
        ros::Duration(0.001).sleep();  
    }
}

int main(int argc, char **argv) {
    config_file_read();
    ros::init(argc, argv, "voxl_imu_ros");
    ros::NodeHandle nh;
	int n_enabled = 0;
    int i;


	for(i=0;i<N_IMUS;i++) if(imu_enable[i]) n_enabled++;

    if(kill_existing_process(PROCESS_NAME, 2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(enable_signal_handler()==-1){
		ROS_ERROR("ERROR: failed to start signal handler");
		quit(-1);
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	make_pid_file(PROCESS_NAME);

    struct sched_param param;
	memset(&param, 0, sizeof(param));
	param.sched_priority = THREAD_PRIORITY_RT_HIGH;
	int ret = sched_setscheduler(0, SCHED_FIFO, &param);
	if(ret==-1){
		ROS_ERROR("WARNING Failed to set priority, errno = %d", errno);
	}
	// check
	ret = sched_getscheduler(0);
	if(ret!=SCHED_FIFO){
		ROS_ERROR("WARNING: failed to set scheduler");
	}

	if(!n_enabled){
		ROS_ERROR("No IMU is enabled!\n");
		ROS_ERROR("Enable at least 1 imu in the config file");
		quit(-1);
	}
    // Initialize IMU or perform board detection
    if (imu_detect_board()) {
        ROS_ERROR("Failed to detect IMU board. Exiting...");
        return -1;
    }

    // Initialize IMUs
    for (int i = 0; i < 1; ++i) {
        if (!imu_init(i)) {
            ROS_INFO("IMU %d initialized successfully.", i);
        } else {
            ROS_ERROR("Failed to initialize IMU %d.", i);
            quit(-1);
            return -1;
        }
    }

    // Start reading threads for each enabled IMU
    std::vector<boost::thread> threads;
    for (int i = 0; i < 1; ++i) {
        if (imu_enable[i]) {
            //IF YOU DONT WANT TO LOCK A SPECIFIC CORE, THEN 
            // threads.push_back(boost::thread(boost::bind(&readThreadFunc, i)));

            threads.push_back(boost::thread([i]() {
            _set_affinity_to_core_7();  
            readThreadFunc(i);
            }));

        }
    }

    // Wait for all threads to finish
    for (auto& thread : threads) {
        thread.join();
    }

    quit(0);
    return 0;
}
