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

#ifndef GENERIC_MPA_INTERFACE
#define GENERIC_MPA_INTERFACE

#include <string.h>
#include <ros/ros.h>
#include <modal_pipe.h>
#include "common_utils.h"

#define PIPE_CLIENT_NAME "mpa-to-ros"

enum InterfaceState {
    ST_NULL,
    ST_READY,
    ST_AD,
    ST_RUNNING,
    ST_CLEAN
};

/**
 * This class describes the functionality for a generic mpa interface, all interfaces
 * should follow this structure, see camera_interface for a basic example
 * 
 * All children should call the 4-parameter constructor of this class
 * 
 * All children should use the protected disconnect callback function in this class
 *    as the dc callback for their pipes, which will close them cleanly should
 *    their server disconnect
 * 
 * rosNodeHandle and basechannel should be supplied by main, numchannels should 
 * be supplied by the child class, and pipename should be supplied by main via a rosparam
 */
class GenericInterface
{

public:

    GenericInterface(ros::NodeHandle  rosNodeHandle,
                     ros::NodeHandle  rosNodeHandleParams,
                     const char *     pipeName):
    m_rosNodeHandle(rosNodeHandle),
    m_rosNodeHandleParams(rosNodeHandleParams),
    m_channel(pipe_client_get_next_available_channel())
    {

        if(m_channel == -1) throw -1;
        pipe_client_set_disconnect_cb(m_channel, _interface_dc_cb, this);
        //printf("%s on channel: %d\n", pipeName, m_channel);
        strcpy(m_pipeName, pipeName);
        m_state = ST_READY;
    }

    virtual ~GenericInterface(){
        printf("Closing %s on channel: %d\n", m_pipeName, m_channel);
        pipe_client_close(m_channel);
    };


    /**
     * The interface should inspect all of its output messages and determine
     * how many subscribers it has
     *
     * @return     Numbr of subscribers this interface has
     */
    virtual int GetNumClients()        = 0;

    /**
     * The interface should use this function to inspect which of
     * its potential pipes are available and advertise them to ros
     *
     * state should be set to advertising at the end of a successful call
     */
    virtual void AdvertiseTopics()     = 0;

    /**
     * The interface should use this function to subscribe to its pipes
     * and begin actually publishing data to ros
     *
     * This may be called multiple times during execution
     *
     * state should be set to running at the end of a successful call
     *
     * Simple version implemented here, can be overridden if necessary
     *
     */
    virtual void StartPublishing() {

        pipe_client_resume(m_channel);
        m_state = ST_RUNNING;

    }

    /**
     * The interface should use this function to unsubscribe from its
     * pipes and cease publishing data to ros
     *
     * This may be called multiple times during execution
     *
     * state should be set to advertising at the end of a successful call
     *
     * Simple version implemented here, can be overridden if necessary
     *
     */
    virtual void StopPublishing() {

        pipe_client_pause(m_channel);
        m_state = ST_AD;

    }

    /**
     * The interface should use this function to close all ros outputs
     * 
     * state should be set to clean at the end of a successful call
     */
    virtual void StopAdvertising()   = 0;

    InterfaceState GetState(){
        return m_state;
    }

    void SetState(InterfaceState state){
        m_state = state;
    }

    const char *GetPipeName(){
        return (char *)m_pipeName;
    }

protected:

    static void _interface_dc_cb(int ch, void* context){

        GenericInterface *interface = (GenericInterface *)context;

        printf("Interface: %s's data pipe disconnected, closing until it returns\n", interface->GetPipeName());

        interface->StopPublishing();
        interface->StopAdvertising();
        interface->SetState(ST_READY);

    }

    ros::NodeHandle         m_rosNodeHandle, m_rosNodeHandleParams;
    char                    m_pipeName[MODAL_PIPE_MAX_PATH_LEN];
    InterfaceState          m_state               = ST_NULL;
    const int               m_channel;

};

#endif
