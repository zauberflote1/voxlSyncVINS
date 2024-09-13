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

#include "camera_helpers.h"
#include <modal_pipe.h>
#include <string> 
#include <sensor_msgs/image_encodings.h>

int GetStepSize(int format){
    switch (format){

        case IMAGE_FORMAT_RAW8 :
        case IMAGE_FORMAT_STEREO_RAW8 :
            return 1;

        case IMAGE_FORMAT_RAW16 :
        case IMAGE_FORMAT_YUV422 :
            return 2;

        case IMAGE_FORMAT_RGB :
            return 3;

        case IMAGE_FORMAT_FLOAT32 :
            return 4;

/*
        case IMAGE_FORMAT_NV12 :
        case IMAGE_FORMAT_H264 :
        case IMAGE_FORMAT_H265 :
        case IMAGE_FORMAT_NV21 :
        case IMAGE_FORMAT_JPG :
        case IMAGE_FORMAT_YUV420 :*/
        default:
            return -1; //unsupported
    }
}

const std::string GetRosFormat(int format){
    switch (format){

        case IMAGE_FORMAT_RAW8 :
        case IMAGE_FORMAT_STEREO_RAW8 :
            return sensor_msgs::image_encodings::MONO8;

        case IMAGE_FORMAT_RAW16 :
            return sensor_msgs::image_encodings::MONO16;

        case IMAGE_FORMAT_YUV422 :
            return sensor_msgs::image_encodings::YUV422;

        case IMAGE_FORMAT_RGB :
            return sensor_msgs::image_encodings::RGB8;

        case IMAGE_FORMAT_FLOAT32 :
            return sensor_msgs::image_encodings::TYPE_32FC1;
/*
        case IMAGE_FORMAT_NV12 :
        case IMAGE_FORMAT_H264 :
        case IMAGE_FORMAT_H265 :
        case IMAGE_FORMAT_NV21 :
        case IMAGE_FORMAT_JPG :
        case IMAGE_FORMAT_YUV420 :*/
        default:
            return std::string("UNSUPPORTED"); //unsupported
    }
}
