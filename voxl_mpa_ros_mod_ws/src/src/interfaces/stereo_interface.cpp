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
#include "stereo_interface.h"
#include "camera_helpers.h"

static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context);

StereoInterface::StereoInterface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    const char *    camName) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, camName)
{

    char frameName[64];

    snprintf(frameName, 64, "%s/left", m_pipeName);
    m_imageMsgL.header.frame_id = frameName;
    m_imageMsgL.is_bigendian    = false;

    snprintf(frameName, 64, "%s/right", m_pipeName);
    m_imageMsgR.header.frame_id = frameName;
    m_imageMsgR.is_bigendian    = false;

    pipe_client_set_camera_helper_cb(m_channel, _frame_cb, this);

    if(pipe_client_open(m_channel, camName, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_CAMERA_HELPER | CLIENT_FLAG_START_PAUSED, 0)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

}

void StereoInterface::AdvertiseTopics(){

    image_transport::ImageTransport it(m_rosNodeHandle);

    char topicName[64];

    snprintf(topicName, 64, "%s/left", m_pipeName);
    m_rosImagePublisherL = it.advertise(topicName, 1);

    snprintf(topicName, 64, "%s/right", m_pipeName);
    m_rosImagePublisherR = it.advertise(topicName, 1);

    m_state = ST_AD;

}

void StereoInterface::StopAdvertising(){

    m_rosImagePublisherL.shutdown();
    m_rosImagePublisherR.shutdown();

    m_state = ST_CLEAN;

}

int StereoInterface::GetNumClients(){
    return m_rosImagePublisherL.getNumSubscribers() + m_rosImagePublisherR.getNumSubscribers();
}

// IR helper callback whenever a frame arrives
static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context)
{

    StereoInterface *interface = (StereoInterface *) context;

    if(interface->GetState() != ST_RUNNING) return;

    if(meta.format != IMAGE_FORMAT_STEREO_RAW8 && meta.format != IMAGE_FORMAT_RAW8){
        printf("Stereo interface received non-stereo frame, exiting stereo\n");
        interface->StopPublishing();
        interface->StopAdvertising();
    }

    image_transport::Publisher& publisherL = interface->GetPublisherL();
    sensor_msgs::Image& imgL = interface->GetImageMsgL();

    image_transport::Publisher& publisherR = interface->GetPublisherR();
    sensor_msgs::Image& imgR = interface->GetImageMsgR();

    imgL.header.stamp = (_clock_monotonic_to_ros_time(meta.timestamp_ns));
    imgL.width    = meta.width;
    imgL.height   = meta.height;
    imgL.step     = meta.width;
    imgL.encoding = GetRosFormat(meta.format);

    imgR.header.stamp = imgL.header.stamp;
    imgR.width    = meta.width;
    imgR.height   = meta.height;
    imgR.step     = meta.width;
    imgR.encoding = GetRosFormat(meta.format);

    int dataSize = imgL.step * imgL.height;

    imgL.data.resize(dataSize);
    imgR.data.resize(dataSize);

    if(meta.format == IMAGE_FORMAT_STEREO_RAW8){
        memcpy(&(imgL.data[0]), frame, dataSize);
        memcpy(&(imgR.data[0]), &frame[dataSize], dataSize);
    } else {
        memcpy(&(imgL.data[0]), frame, dataSize);
        memcpy(&(imgR.data[0]), frame, dataSize);
    }

    publisherL.publish(imgL);
    publisherR.publish(imgR);
}
