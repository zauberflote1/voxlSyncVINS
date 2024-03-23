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
#include "ai_detection_interface.h"

// big buffer here, potentially lots of detections/frame
#define OBJ_BUF_LEN (sizeof(ai_detection_t) * 500)

static void _helper_cb(
    __attribute__((unused))int ch,
                           char* data,
                           int bytes,
                           void* context);

AiDetectionInterface::AiDetectionInterface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    const char *    name) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, name)
{
    m_objMsg.timestamp_ns = 0;
    m_objMsg.class_id = 0;
    m_objMsg.frame_id = 0;
    m_objMsg.class_name = "NULL";
    m_objMsg.cam = "NULL";
    m_objMsg.class_confidence = 0;
    m_objMsg.detection_confidence = 0;
    m_objMsg.x_min = 0;
    m_objMsg.y_min = 0;
    m_objMsg.x_max = 0;
    m_objMsg.y_max = 0;

    pipe_client_set_simple_helper_cb(m_channel, _helper_cb, this);

    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER | CLIENT_FLAG_START_PAUSED,
                OBJ_BUF_LEN)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

}

void AiDetectionInterface::AdvertiseTopics(){

    char topicName[64];

    snprintf(topicName, 64, "%s", m_pipeName);
    m_rosPublisher = m_rosNodeHandle.advertise<voxl_mpa_to_ros::AiDetection>(topicName, 1);

    m_state = ST_AD;

}

void AiDetectionInterface::StopAdvertising(){

    m_rosPublisher.shutdown();

    m_state = ST_CLEAN;

}

int AiDetectionInterface::GetNumClients(){
    return m_rosPublisher.getNumSubscribers();
}

// called when the simple helper has data for us
static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
{
    AiDetectionInterface *interface = (AiDetectionInterface *) context;
    if(interface->GetState() != ST_RUNNING) return;
    ros::Publisher& publisher = interface->GetPublisher();
    voxl_mpa_to_ros::AiDetection& obj = interface->GetObjMsg();

    int n_packets = bytes / sizeof(ai_detection_t);
    ai_detection_t* detections = (ai_detection_t*) data;

    for(int i=0;i<n_packets;i++){
        if (detections[i].magic_number != AI_DETECTION_MAGIC_NUMBER) return;

        //publish the sample
        obj.timestamp_ns = detections[i].timestamp_ns;
        obj.class_id = detections[i].class_id;
        obj.frame_id = detections[i].frame_id;
        obj.class_name = detections[i].class_name;
        obj.cam = detections[i].cam;
        obj.class_confidence = detections[i].class_confidence;
        obj.detection_confidence = detections[i].detection_confidence;
        obj.x_min = detections[i].x_min;
        obj.y_min = detections[i].y_min;
        obj.x_max = detections[i].x_max;
        obj.y_max = detections[i].y_max;
        publisher.publish(obj);
    }

    return;
}
