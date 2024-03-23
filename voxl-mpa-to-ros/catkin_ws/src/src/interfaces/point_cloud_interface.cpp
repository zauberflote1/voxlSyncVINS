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
#include <modal_json.h>
#include "point_cloud_interface.h"
#include "camera_helpers.h"

static void _helper_cb(
                           int ch, 
                           point_cloud_metadata_t meta,
                           void* data, 
                           void* context);

PointCloudInterface::PointCloudInterface(
    ros::NodeHandle rosNodeHandle,
    ros::NodeHandle rosNodeHandleParams,
    const char *    name) :
    GenericInterface(rosNodeHandle, rosNodeHandleParams, name)
{

    //TODO Different frames
    m_pcMsg.header.frame_id = "world";

    m_pcMsg.is_bigendian = false;
    // Data is not "dense" since not all points are valid.
    m_pcMsg.is_dense     = false;
    m_pcMsg.row_step   = 1;

    pipe_client_set_point_cloud_helper_cb(m_channel, _helper_cb, this);

    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                CLIENT_FLAG_EN_POINT_CLOUD_HELPER | CLIENT_FLAG_START_PAUSED,
                TOF_RECOMMENDED_READ_BUF_SIZE)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }
}

void PointCloudInterface::AdvertiseTopics(){

    char topicName[64];
    snprintf(topicName, 64, "%s", m_pipeName);
    m_pcPublisher         = m_rosNodeHandle.advertise<sensor_msgs::PointCloud2>
                                (topicName, 3);

    m_state = ST_AD;

}

void PointCloudInterface::StopAdvertising(){

    m_pcPublisher.shutdown();

    m_state = ST_CLEAN;

}

int PointCloudInterface::GetNumClients(){
    return m_pcPublisher.getNumSubscribers();
}

// called when the simple helper has data for us
static void _helper_cb (int ch, point_cloud_metadata_t meta, void* data, void* context)
{

    PointCloudInterface *interface = (PointCloudInterface *) context;

    if(interface->GetState() != ST_RUNNING) return;

    sensor_msgs::PointCloud2&   pcMsg =                 interface->GetPCMsg();
    ros::Publisher&             pcPublisher =           interface->GetPCPublisher();

    if(interface->m_inputPCType != meta.format){

        switch (meta.format) {

            case POINT_CLOUD_FORMAT_FLOAT_XYZ:{
                // The total size of the channels of the point cloud for a single point are:
                // sizeof(float) * 3 = x, y, and z
                pcMsg.point_step = (sizeof(float) * 3);

                // Describe the fields (channels) associated with each point.
                pcMsg.fields.resize(3);
                pcMsg.fields[0].name = "x";
                pcMsg.fields[1].name = "y";
                pcMsg.fields[2].name = "z";

                // Defines the format of channels.
                int index;

                for (index = 0; index < 3; index++)
                {
                    pcMsg.fields[index].offset = sizeof(float) * index;
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
                    pcMsg.fields[index].count = 1;
                }

                break;
            }

            case POINT_CLOUD_FORMAT_FLOAT_XYZC:{

                // The total size of the channels of the point cloud for a single point are:
                // sizeof(float) * 4 = x, y, z, confidence
                pcMsg.point_step = (sizeof(float) * 4);

                // Describe the fields (channels) associated with each point.
                pcMsg.fields.resize(4);
                pcMsg.fields[0].name = "x";
                pcMsg.fields[1].name = "y";
                pcMsg.fields[2].name = "z";
                pcMsg.fields[3].name = "confidence";

                // Defines the format of channels.
                int index;

                for (index = 0; index < 4; index++)
                {
                    pcMsg.fields[index].offset = sizeof(float) * index;
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
                    pcMsg.fields[index].count = 1;
                }
                break;
            }

            case POINT_CLOUD_FORMAT_FLOAT_XYZRGB:{

                // The total size of the channels of the point cloud for a single point are:
                // sizeof(float) * 3 = x, y, z + sizeof(uint8_t) * 3 = r,g,b
                pcMsg.point_step = (sizeof(float) * 3) + (sizeof(uint8_t) * 3);

                // Describe the fields (channels) associated with each point.
                pcMsg.fields.resize(6);
                pcMsg.fields[0].name = "x";
                pcMsg.fields[1].name = "y";
                pcMsg.fields[2].name = "z";
                pcMsg.fields[3].name = "r";
                pcMsg.fields[4].name = "g";
                pcMsg.fields[5].name = "b";

                // Defines the format of channels.
                int index;

                for (index = 0; index < 3; index++)
                {
                    pcMsg.fields[index].offset = sizeof(float) * index;
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
                    pcMsg.fields[index].count = 1;
                }

                for (index = 3; index < 6; index++)
                {
                    pcMsg.fields[index].offset = (sizeof(float) * 3) + (sizeof(uint8_t) * (index - 3));
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::UINT8;
                    pcMsg.fields[index].count = 1;
                }

                break;
            }

            case POINT_CLOUD_FORMAT_FLOAT_XYZCRGB:{

                // The total size of the channels of the point cloud for a single point are:
                // sizeof(float) * 3 = x, y, z, conf + sizeof(uint8_t) * 3 = r,g,b
                pcMsg.point_step = (sizeof(float) * 4) + (sizeof(uint8_t) * 3);

                // Describe the fields (channels) associated with each point.
                pcMsg.fields.resize(7);
                pcMsg.fields[0].name = "x";
                pcMsg.fields[1].name = "y";
                pcMsg.fields[2].name = "z";
                pcMsg.fields[3].name = "confidence";
                pcMsg.fields[4].name = "r";
                pcMsg.fields[5].name = "g";
                pcMsg.fields[6].name = "b";

                // Defines the format of channels.
                int index;

                for (index = 0; index < 4; index++)
                {
                    pcMsg.fields[index].offset = sizeof(float) * index;
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
                    pcMsg.fields[index].count = 1;
                }

                for (index = 4; index < 7; index++)
                {
                    pcMsg.fields[index].offset = (sizeof(float) * 4) + (sizeof(uint8_t) * (index - 4));
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::UINT8;
                    pcMsg.fields[index].count = 1;
                }

                break;
            }

            case POINT_CLOUD_FORMAT_FLOAT_XY:{

                // The total size of the channels of the point cloud for a single point are:
                // sizeof(float) * 2 = x, y
                pcMsg.point_step = (sizeof(float) * 2);

                // Describe the fields (channels) associated with each point.
                pcMsg.fields.resize(2);
                pcMsg.fields[0].name = "x";
                pcMsg.fields[1].name = "y";

                // Defines the format of channels.
                int index;

                for (index = 0; index < 2; index++)
                {
                    pcMsg.fields[index].offset = sizeof(float) * index;
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
                    pcMsg.fields[index].count = 1;
                }

                break;
            }

            case POINT_CLOUD_FORMAT_FLOAT_XYC:{

                // The total size of the channels of the point cloud for a single point are:
                // sizeof(float) * 3 = x, y, confidence
                pcMsg.point_step = (sizeof(float) * 3);

                // Describe the fields (channels) associated with each point.
                pcMsg.fields.resize(3);
                pcMsg.fields[0].name = "x";
                pcMsg.fields[1].name = "y";
                pcMsg.fields[2].name = "confidence";

                // Defines the format of channels.
                int index;

                for (index = 0; index < 3; index++)
                {
                    pcMsg.fields[index].offset = sizeof(float) * index;
                    pcMsg.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
                    pcMsg.fields[index].count = 1;
                }

                break;
            }

            default:
                printf("Unknown pointcloud format: %d, closing interface\n", meta.format);
                pipe_client_close(ch); //This will not return
                return;

        }

        pcMsg.height = 1;
        pcMsg.width  = meta.n_points;

        pcMsg.row_step = pcMsg.point_step * pcMsg.width;
        pcMsg.data.resize(pcMsg.height * pcMsg.row_step);

    }

    pcMsg.header.stamp = (_clock_monotonic_to_ros_time( meta.timestamp_ns));

    switch (meta.format) {

        case POINT_CLOUD_FORMAT_FLOAT_XYZ:{

            float* dataPoints = (float*)data;

            for(uint32_t i = 0; i < meta.n_points; i++){
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = dataPoints[i*3 + 0];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = dataPoints[i*3 + 1];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[2].offset])) = dataPoints[i*3 + 2];
            }
            break;
        }

        case POINT_CLOUD_FORMAT_FLOAT_XYZC:{

            float* dataPoints = (float*)data;

            for(uint32_t i = 0; i < meta.n_points; i++){
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = dataPoints[i*4 + 0];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = dataPoints[i*4 + 1];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[2].offset])) = dataPoints[i*4 + 2];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[3].offset])) = dataPoints[i*4 + 3];
            }
            break;
        }

        case POINT_CLOUD_FORMAT_FLOAT_XY:{

            float* dataPoints = (float*)data;

            for(uint32_t i = 0; i < meta.n_points; i++){
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = dataPoints[i*2 + 0];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = dataPoints[i*2 + 1];
            }
            break;
        }

        case POINT_CLOUD_FORMAT_FLOAT_XYC:{

            float* dataPoints = (float*)data;

            for(uint32_t i = 0; i < meta.n_points; i++){
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = dataPoints[i*3 + 0];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = dataPoints[i*3 + 1];
                *((float *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[2].offset])) = dataPoints[i*3 + 2];
            }
            break;
        }

        case POINT_CLOUD_FORMAT_FLOAT_XYZRGB:{

            typedef struct datatype {
                float floats[3];
                uint8_t ints[3];
            } __attribute__((packed)) datatype;

            datatype* dataPoints = (datatype*)data;

            for(uint32_t i = 0; i < meta.n_points; i++){

                datatype dataPoint = dataPoints[i];

                *((float *)  &(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = dataPoint.floats[0];
                *((float *)  &(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = dataPoint.floats[1];
                *((float *)  &(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[2].offset])) = dataPoint.floats[2];
                *((uint8_t *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[3].offset])) = dataPoint.ints[0];
                *((uint8_t *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[4].offset])) = dataPoint.ints[1];
                *((uint8_t *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[5].offset])) = dataPoint.ints[2];
            }
            break;
        }

        case POINT_CLOUD_FORMAT_FLOAT_XYZCRGB:{

            typedef struct datatype {
                float floats[4];
                uint8_t ints[3];
            } __attribute__((packed)) datatype;

            datatype* dataPoints = (datatype*)data;

            for(uint32_t i = 0; i < meta.n_points; i++){

                datatype dataPoint = dataPoints[i];

                *((float *)  &(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[0].offset])) = dataPoint.floats[0];
                *((float *)  &(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[1].offset])) = dataPoint.floats[1];
                *((float *)  &(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[2].offset])) = dataPoint.floats[2];
                *((float *)  &(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[3].offset])) = dataPoint.floats[3];
                *((uint8_t *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[4].offset])) = dataPoint.ints[0];
                *((uint8_t *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[5].offset])) = dataPoint.ints[1];
                *((uint8_t *)&(pcMsg.data[(i * pcMsg.point_step) + pcMsg.fields[6].offset])) = dataPoint.ints[2];
            }
            break;
        }

    }
    pcPublisher.publish(pcMsg);

    return;
}
