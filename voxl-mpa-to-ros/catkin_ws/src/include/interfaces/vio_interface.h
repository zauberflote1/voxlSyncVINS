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

#ifndef VIO_MPA_INTERFACE
#define VIO_MPA_INTERFACE

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "generic_interface.h"

class VIOInterface: public GenericInterface
{
public:
    VIOInterface(ros::NodeHandle rosNodeHandle,
                 ros::NodeHandle rosNodeHandleParams,
                 const char*     name);

    ~VIOInterface() { };

    int  GetNumClients();
    void AdvertiseTopics();
    void StopAdvertising();

    geometry_msgs::PoseStamped& GetPoseMsg(){
        return m_poseMsg;
    }
    nav_msgs::Odometry& GetOdometryMsg(){
        return m_odomMsg;
    }

    ros::Publisher& GetPosePublisher(){
        return m_posePublisher;
    }
    ros::Publisher& GetOdometryPublisher(){
        return m_odomPublisher;
    }

private:

    geometry_msgs::PoseStamped           m_poseMsg;                    ///< Image message
    nav_msgs::Odometry                   m_odomMsg;                    ///< Image message

    ros::Publisher                       m_posePublisher;              ///< Image publisher
    ros::Publisher                       m_odomPublisher;              ///< Image publisher

};
#endif
