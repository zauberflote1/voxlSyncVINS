/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <modal_pipe.h>
#include "imu_interface.h"

static void _helper_cb(
    __attribute__((unused))int ch, 
                           char* data, 
                           int bytes, 
                           void* context);

IMUInterface::IMUInterface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    const char *    name) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, name)
{

    m_imuMsg.header.frame_id = "map";
    m_imuMsg.orientation.x = 0;
    m_imuMsg.orientation.y = 0;
    m_imuMsg.orientation.z = 0;
    m_imuMsg.orientation.w = 0;
    m_imuMsg.orientation_covariance         = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
    m_imuMsg.angular_velocity_covariance    = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    m_imuMsg.linear_acceleration_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    pipe_client_set_simple_helper_cb(m_channel, _helper_cb, this);

    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER | CLIENT_FLAG_START_PAUSED,
                IMU_RECOMMENDED_READ_BUF_SIZE)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

}

void IMUInterface::AdvertiseTopics(){

    char topicName[64];

    snprintf(topicName, 64, "%s", m_pipeName);
    m_rosPublisher = m_rosNodeHandle.advertise<sensor_msgs::Imu>(topicName, 1);

    m_state = ST_AD;

}

void IMUInterface::StopAdvertising(){

    m_rosPublisher.shutdown();

    m_state = ST_CLEAN;

}

int IMUInterface::GetNumClients(){
    return m_rosPublisher.getNumSubscribers();
}

// called when the simple helper has data for us
static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
{
    //Check order of timestamps is correct, if not remove the packet
    // validate that the data makes sense
    int n_packets;
    imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);
    if(data_array == NULL) return;
    IMUInterface *interface = (IMUInterface *) context;
    if (interface->firstPckt){
        interface->firstPckt = false;
        interface->prevTS = _clock_monotonic_to_ros_time(data_array[0].timestamp_ns);
    }
    if(interface->GetState() != ST_RUNNING) return;
    ros::Publisher& publisher = interface->GetPublisher();
    sensor_msgs::Imu& imu = interface->GetImuMsg();

    // make a new data struct to hold the average
    imu_data_t avg;
    memset(&avg,0,sizeof(avg));

    //publish all the samples
    for(int i=0;i<n_packets;i++){
        if (_clock_monotonic_to_ros_time(data_array[i].timestamp_ns)<interface->prevTS){
             continue;
        }
        
        
        
        imu.header.stamp = (_clock_monotonic_to_ros_time(data_array[i].timestamp_ns));

        imu.angular_velocity.x = data_array[i].gyro_rad[0];
        imu.angular_velocity.y = data_array[i].gyro_rad[1];
        imu.angular_velocity.z = data_array[i].gyro_rad[2];
        imu.linear_acceleration.x = data_array[i].accl_ms2[0];
        imu.linear_acceleration.y = data_array[i].accl_ms2[1];
        imu.linear_acceleration.z = data_array[i].accl_ms2[2];

        publisher.publish(imu);
        interface->prevTS = _clock_monotonic_to_ros_time(data_array[i].timestamp_ns);

    }


    return;
}
