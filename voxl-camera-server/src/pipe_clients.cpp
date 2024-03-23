/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
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

#include <stdio.h>
#include <pthread.h>

#include <c_library_v2/common/mavlink.h>
#include <c_library_v2/development/development.h>
#include <modal_pipe_client.h>
#include <cpu_monitor_interface.h>
#include <modal_journal.h>

#include "pipe_clients.h"
#include "voxl_camera_server.h" // for #define PROCESS_NAME "voxl-camera-server"



#define GPS_PIPE_NAME    "mavlink_gps_raw_int"
#define CPU_PIPE_NAME    "cpu_monitor"

#define GPS_CH  0
#define CPU_CH  1


static double lat_deg = 0.0;
static double lon_deg = 0.0;
static double alt_m = 0.0;
static int standby_active = 0;


// protect multi-byte states such as the attitude struct with this mutex
static pthread_mutex_t gps_mtx = PTHREAD_MUTEX_INITIALIZER;

static void _gps_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
    M_PRINT("connected to GPS mavlink pipe\n");
}

static void _gps_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
    M_PRINT("disconnected from GPS mavlink pipe\n");
}

static void _gps_helper_cb(__attribute__((unused))int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
    pthread_mutex_lock(&gps_mtx);
    // validate that the data makes sense
    int n_packets;
    mavlink_message_t* msg_array = pipe_validate_mavlink_message_t(data, bytes, &n_packets);
    if(msg_array == NULL){
        return;
    }

    // grab the first one
    mavlink_message_t* msg = &msg_array[0];

    // fetch integer values from the unpacked mavlink message
    int32_t  lat  = mavlink_msg_gps_raw_int_get_lat(msg);
    int32_t  lon  = mavlink_msg_gps_raw_int_get_lon(msg);
    int32_t  alt  = mavlink_msg_gps_raw_int_get_alt(msg);

    // convert integer values to more useful units
    lat_deg = (double)lat/10000000.0;
    lon_deg = (double)lon/10000000.0;
    alt_m   = (double)alt/1000.0;
    pthread_mutex_unlock(&gps_mtx);

    return;
}


static void _cpu_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
    M_PRINT("Connected to cpu-monitor\n");
    return;
}

static void _cpu_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
    M_PRINT("Disconnected from cpu-monitor\n");
    return;
}


// called whenever the simple helper has data for us to process
static void _cpu_helper_cb(__attribute__((unused))int ch, char* raw_data, int bytes, __attribute__((unused)) void* context)
{
    int n_packets;
    cpu_stats_t *data_array = modal_cpu_validate_pipe_data(raw_data, bytes, &n_packets);
    if (data_array == NULL){
        M_DEBUG("Data array is null");
        return;
    }
    // only use most recent packet
    cpu_stats_t data = data_array[n_packets-1];

    if(data.flags&CPU_STATS_FLAG_STANDBY_ACTIVE){
        if(!standby_active){
            M_DEBUG("Entering standby mode\n");
            standby_active = 1;
        }
    }
    else{
        if(standby_active){
            M_DEBUG("Exiting standby mode\n");
            standby_active = 0;
        }
    }
    M_DEBUG("Value of standby_active is: %i \n", standby_active);
    return;
}




int pipe_clients_init(void)
{
    // Setup necessary pipes for GPS and CPU
    pipe_client_set_connect_cb(GPS_CH, _gps_connect_cb, NULL);
    pipe_client_set_disconnect_cb(GPS_CH, _gps_disconnect_cb, NULL);
    pipe_client_set_simple_helper_cb(GPS_CH, _gps_helper_cb, NULL);
    int ret = pipe_client_open(GPS_CH, GPS_PIPE_NAME, PROCESS_NAME, \
                    EN_PIPE_CLIENT_SIMPLE_HELPER, \
                    MAVLINK_MESSAGE_T_RECOMMENDED_READ_BUF_SIZE);


    pipe_client_set_connect_cb(CPU_CH, _cpu_connect_cb, NULL);
    pipe_client_set_disconnect_cb(CPU_CH, _cpu_disconnect_cb, NULL);
    pipe_client_set_simple_helper_cb(CPU_CH, _cpu_helper_cb, NULL);
    ret |= pipe_client_open(CPU_CH, CPU_PIPE_NAME, PROCESS_NAME, \
                    EN_PIPE_CLIENT_SIMPLE_HELPER, \
                    CPU_STATS_RECOMMENDED_READ_BUF_SIZE);

    return ret;
}

gps_data_t grab_gps_info(void)
{
    gps_data_t gps_info;
    pthread_mutex_lock(&gps_mtx);
    gps_info.latitude = lat_deg;
    gps_info.longitude = lon_deg;
    gps_info.altitude = alt_m;
    pthread_mutex_unlock(&gps_mtx);
    return gps_info;
}

int grab_cpu_standby_active(void)
{
    return standby_active;
}

int pipe_clients_close(void)
{
    pipe_client_close(GPS_CH);
    pipe_client_close(CPU_CH);
    return 0;
}

