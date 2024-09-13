/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
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

#ifndef STEREO_MPA_INTERFACE
#define STEREO_MPA_INTERFACE

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

#include "generic_interface.h"

class StereoInterface: public GenericInterface
{
public:
    StereoInterface(ros::NodeHandle rosNodeHandle,
                    ros::NodeHandle rosNodeHandleParams,
                    const char*     camName);

    ~StereoInterface() { };

    int  GetNumClients();
    void AdvertiseTopics();
    void StopAdvertising();

    sensor_msgs::Image& GetImageMsgL(){
        return m_imageMsgL;
    }
    sensor_msgs::Image& GetImageMsgR(){
        return m_imageMsgR;
    }

    image_transport::Publisher& GetPublisherL(){
        return m_rosImagePublisherL;
    }
    image_transport::Publisher& GetPublisherR(){
        return m_rosImagePublisherR;
    }

private:

    sensor_msgs::Image                     m_imageMsgL;                   ///< Image message
    image_transport::Publisher             m_rosImagePublisherL;          ///< Image publisher

    sensor_msgs::Image                     m_imageMsgR;                   ///< Image message
    image_transport::Publisher             m_rosImagePublisherR;          ///< Image publisher

};
#endif
