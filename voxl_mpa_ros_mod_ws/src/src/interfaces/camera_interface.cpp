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
#include <iostream>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "camera_interface.h"
#include "camera_helpers.h"
#include <iostream>  // For std::cout
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
    pipeName = m_pipeName;                                    ///< char* converted to string

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

    if (frame_format == IMAGE_FORMAT_H265 || frame_format == IMAGE_FORMAT_H264) {
        m_rosCompressedPublisher = m_rosNodeHandle.advertise<sensor_msgs::CompressedImage>(m_pipeName, 1);
    } else {
        m_rosImagePublisher = it.advertise(m_pipeName, 1);
    }

    // std::string pipeName = std::string(m_pipeName);
    // std::string cameraInfoTopic = pipeName + "/camera_info";
    // m_rosCameraInfoPublisher = m_rosNodeHandle.advertise<sensor_msgs::CameraInfo>(cameraInfoTopic, 1);

    // // Parsing yaml
    // std::string cv_intrinsics_path = "/data/modalai/opencv_" + pipeName + "_intrinsics.yml";
    // if(access(cv_intrinsics_path.c_str(), F_OK) == 0){
    //     YAML::Node config = YAML::LoadFile(cv_intrinsics_path);
        
    //     // Getting static values from yaml
    //     m_cameraInfo.width = config["width"].as<uint32_t>();
    //     m_cameraInfo.height = config["height"].as<uint32_t>();
    //     m_cameraInfo.distortion_model = config["distortion_model"].as<std::string>();
        
    //     // Getting rotation info
    //     m_cameraInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    //     // Getting distortion data
    //     auto distortion_data = config["D"]["data"].as<std::vector<double>>();
    //     m_cameraInfo.D.assign(distortion_data.begin(), distortion_data.end());

    //     // Load intrinsic camera matrix
    //     auto camera_matrix_data = config["M"]["data"].as<std::vector<double>>();
    //     for (size_t i = 0; i < camera_matrix_data.size(); ++i) {
    //         m_cameraInfo.K[i] = camera_matrix_data[i];
    //     }

    //     // Assuming zero for projection matrix P (3x4 zero matrix)
    //     m_cameraInfo.P = {camera_matrix_data[0], camera_matrix_data[1], camera_matrix_data[2], 0.0,
    //                 camera_matrix_data[3], camera_matrix_data[4], camera_matrix_data[5], 0.0,
    //                 camera_matrix_data[6], camera_matrix_data[7], camera_matrix_data[8], 0.0};
    // }

    m_state = ST_AD;
}

void CameraInterface::StopAdvertising(){
    
    if (frame_format == IMAGE_FORMAT_H265 || frame_format == IMAGE_FORMAT_H264) {
        m_rosCompressedPublisher.shutdown();
    } else {
        m_rosImagePublisher.shutdown();
    }

    m_rosCameraInfoPublisher.shutdown();

    m_state = ST_CLEAN;
}

int CameraInterface::GetNumClients(){

    if (frame_format == IMAGE_FORMAT_H265 || frame_format == IMAGE_FORMAT_H264) {
        return m_rosCompressedPublisher.getNumSubscribers() + m_rosCameraInfoPublisher.getNumSubscribers();
    } else {
        return m_rosImagePublisher.getNumSubscribers() + m_rosCameraInfoPublisher.getNumSubscribers();
    }
    m_state = ST_CLEAN;
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

    interface->frame_format = meta.format;
    image_transport::Publisher& publisher = interface->GetPublisher();
    ros::Publisher& compressedPublisher = interface->GetCompressedPublisher();
    sensor_msgs::Image& img = interface->GetImageMsg();
    sensor_msgs::CompressedImage& compressed_img = interface->GetCompressedImageMsg();
    // sensor_msgs::CameraInfo& camera_info = interface->GetCameraInfo();
    // ros::Publisher& camera_info_publisher = interface->GetCameraInfoPublisher();

    img.header.stamp = (_clock_monotonic_to_ros_time( meta.timestamp_ns));
    img.width    = meta.width;
    img.height   = meta.height;

    // camera_info.header.stamp = (_clock_monotonic_to_ros_time( meta.timestamp_ns));
    // camera_info.header.frame_id = std::to_string(meta.frame_id);

    // camera_info_publisher.publish(camera_info);

    if(meta.format == IMAGE_FORMAT_NV21 || meta.format == IMAGE_FORMAT_NV12){
        // Create cv::Mat from YUV image data in meta

        std::cout << "Processing NV12/NV21 format" << std::endl;
        cv::Mat yuvImage(meta.height + meta.height / 2, meta.width, CV_8UC1, frame);
        cv::Mat bgrImage;

        // Efficient color conversion
        if(meta.format == IMAGE_FORMAT_NV21) {
            std::cout << "Using NV21 to BGR conversion" << std::endl;
            cv::cvtColor(yuvImage, bgrImage, cv::COLOR_YUV2BGR_NV21);
        } else if(meta.format == IMAGE_FORMAT_NV12) {
            std::cout << "Using NV12 to BGR conversion" << std::endl;
            cv::cvtColor(yuvImage, bgrImage, cv::COLOR_YUV2BGR_NV12);
        }

        // Fill in sensor_msgs::Image manually
        sensor_msgs::Image img_msg;
        img_msg.header.stamp = (_clock_monotonic_to_ros_time(meta.timestamp_ns));
        img_msg.header.frame_id = std::to_string(meta.frame_id);
        img_msg.height = bgrImage.rows;
        img_msg.width = bgrImage.cols;
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
        img_msg.is_bigendian = false;
        img_msg.step = bgrImage.step;

        // Use data.assign for efficient and safe data copying
        img_msg.data.assign(bgrImage.datastart, bgrImage.dataend);

        // Publish the image
        publisher.publish(img_msg);
    } else if (meta.format == IMAGE_FORMAT_H265 || meta.format == IMAGE_FORMAT_H264) {
         std::cout << "Processing H264/H265 format" << std::endl;
        // Set appropriate message fields for H.265
        compressed_img.header.stamp = (_clock_monotonic_to_ros_time(meta.timestamp_ns));
        
        if(meta.format == IMAGE_FORMAT_H265){
            compressed_img.format = "h265";  // Indicate H.265 format
        } else {
            compressed_img.format = "h264";  // Indicate H.265 format
        }

        compressed_img.data.resize(meta.size_bytes); // Resize the data vector to accommodate the frame data

        // Copy frame data to the CompressedImage message
        std::memcpy(compressed_img.data.data(), frame, meta.size_bytes);

        // Publish the compressed H.265 me
        compressedPublisher.publish(compressed_img);
    
    } else if(meta.format == IMAGE_FORMAT_YUV422_UYVY) {
            std::cout << "Processing YUV422_UYVY format" << std::endl;
        // Convert UYVY to BGR8 using OpenCV
        cv::Mat uyvyImage(meta.height, meta.width, CV_8UC2, frame);
        cv::Mat bgrImage;

        cv::cvtColor(uyvyImage, bgrImage, cv::COLOR_YUV2BGR_UYVY);
        sensor_msgs::Image img_msg;
        img_msg.header.stamp = (_clock_monotonic_to_ros_time(meta.timestamp_ns));
        img_msg.header.frame_id = std::to_string(meta.frame_id);
        img_msg.height = bgrImage.rows;
        img_msg.width = bgrImage.cols;
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
        img_msg.is_bigendian = false;
        img_msg.step = bgrImage.step;
        img_msg.data.assign(bgrImage.datastart, bgrImage.dataend);

        // Publish the image
        publisher.publish(img_msg);
    } else {
         std::cout << "Processing YUV422_YUY2 format" << std::endl;
   cv::Mat yuvImage(meta.height, meta.width, CV_8UC2, frame);
    cv::Mat bgrImage;

    // Use OpenCV to convert YUV422 (YUYV) to BGR
    cv::cvtColor(yuvImage, bgrImage, cv::COLOR_YUV2BGR_YUY2);

    // Prepare ROS Image message
    img.encoding = sensor_msgs::image_encodings::BGR8;
    img.step = bgrImage.step;
    img.data.assign(bgrImage.datastart, bgrImage.dataend);

    // Publish the image
    publisher.publish(img);

    }
}
