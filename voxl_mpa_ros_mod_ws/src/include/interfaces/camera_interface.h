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

#ifndef CAMERA_MPA_INTERFACE
#define CAMERA_MPA_INTERFACE

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

#include "generic_interface.h"

class CameraInterface: public GenericInterface
{
public:
    CameraInterface(ros::NodeHandle rosNodeHandle,
                    ros::NodeHandle rosNodeHandleParams,
                    const char*     camName);

    ~CameraInterface() { };

    int  GetNumClients();
    void AdvertiseTopics();
    void StopAdvertising();

    sensor_msgs::Image& GetImageMsg(){
        return m_imageMsg;
    }

    sensor_msgs::CompressedImage& GetCompressedImageMsg(){
        return m_compressedImage;
    }

    sensor_msgs::CameraInfo& GetCameraInfo(){
        return m_cameraInfo;
    }

    image_transport::Publisher& GetPublisher(){
        return m_rosImagePublisher;
    }

    ros::Publisher& GetCompressedPublisher(){
        return m_rosCompressedPublisher;
    }

    ros::Publisher& GetCameraInfoPublisher(){
        return m_rosCameraInfoPublisher;
    }

    int frame_format;

private:

    sensor_msgs::Image                     m_imageMsg;                   ///< Image message
    sensor_msgs::CompressedImage           m_compressedImage;            ///< Compressed Image Message
    sensor_msgs::CameraInfo                m_cameraInfo;                 ///< Camera info metadata
    image_transport::Publisher             m_rosImagePublisher;          ///< Image publisher
    ros::Publisher                         m_rosCompressedPublisher;     ///< Compressed image handler
    ros::Publisher                         m_rosCameraInfoPublisher;     ///< Compressed image handler
    std::string pipeName;

};
#endif
