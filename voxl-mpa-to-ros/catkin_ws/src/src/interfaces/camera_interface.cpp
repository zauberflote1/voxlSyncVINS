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
#include <string.h>
#include "camera_interface.h"
#include "camera_helpers.h"

static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context);

CameraInterface::CameraInterface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    const char *    camName) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, camName)
{

    m_imageMsg.header.frame_id = camName;
    m_imageMsg.is_bigendian    = false;

    pipe_client_set_camera_helper_cb(m_channel, _frame_cb, this);

    if(pipe_client_open(m_channel, camName, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_CAMERA_HELPER | CLIENT_FLAG_START_PAUSED, 0)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

}

void CameraInterface::AdvertiseTopics(){

    image_transport::ImageTransport it(m_rosNodeHandle);

    m_rosImagePublisher = it.advertise(m_pipeName, 1);

    m_state = ST_AD;

}

void CameraInterface::StopAdvertising(){

    m_rosImagePublisher.shutdown();
    m_state = ST_CLEAN;

}

int CameraInterface::GetNumClients(){
    return m_rosImagePublisher.getNumSubscribers();
}

// helper callback whenever a frame arrives
static void _frame_cb(
    __attribute__((unused)) int ch,
                            camera_image_metadata_t meta, 
                            char* frame, 
                            void* context)
{

    CameraInterface *interface = (CameraInterface *) context;

    if(interface->GetState() != ST_RUNNING) return;

    image_transport::Publisher& publisher = interface->GetPublisher();
    sensor_msgs::Image& img = interface->GetImageMsg();

    img.header.stamp = (_clock_monotonic_to_ros_time( meta.timestamp_ns));
    img.width    = meta.width;
    img.height   = meta.height;

    if(meta.format == IMAGE_FORMAT_NV21 || meta.format == IMAGE_FORMAT_NV12){

        img.step = meta.width * GetStepSize(IMAGE_FORMAT_YUV422);
        img.encoding = GetRosFormat(IMAGE_FORMAT_YUV422);

        int dataSize = img.step * img.height;
        img.data.resize(dataSize);

        char *uv = &(frame[dataSize/2]);

        if(meta.format == IMAGE_FORMAT_NV12){

            for(int i = 0; i < meta.height; i+=2)
            {
                for(int j = 0; j < meta.width*2;j+=2){

                    img.data[(i * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j];
                    img.data[(i * meta.width * 2) + (j * 2) + 1] = frame[(i * meta.width) + j];
                    img.data[(i * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j + 1];
                    img.data[(i * meta.width * 2) + (j * 2) + 3] = frame[(i * meta.width) + j + 1];

                    img.data[((i+1) * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 1] = frame[((i+1) * meta.width) + j];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j + 1];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 3] = frame[((i+1) * meta.width) + j + 1];

                }
            }
        }else {

            for(int i = 0; i < meta.height; i+=2)
            {
                for(int j = 0; j < meta.width*2;j+=2){

                    img.data[(i * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j + 1];
                    img.data[(i * meta.width * 2) + (j * 2) + 1] = frame[(i * meta.width) + j];
                    img.data[(i * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j];
                    img.data[(i * meta.width * 2) + (j * 2) + 3] = frame[(i * meta.width) + j + 1];

                    img.data[((i+1) * meta.width * 2) + (j * 2) + 0] = uv[((i/2) * meta.width) + j + 1];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 1] = frame[((i+1) * meta.width) + j];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 2] = uv[((i/2) * meta.width) + j];
                    img.data[((i+1) * meta.width * 2) + (j * 2) + 3] = frame[((i+1) * meta.width) + j + 1];

                }
            }

        }

        publisher.publish(img);

    } else {

        img.step     = meta.width * GetStepSize(meta.format);
        img.encoding = GetRosFormat(meta.format);

        int dataSize = img.step * img.height;

        img.data.resize(dataSize);

        memcpy(&(img.data[0]), frame, dataSize);

        publisher.publish(img);

    }
}
