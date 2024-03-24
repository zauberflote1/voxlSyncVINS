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

/**
 * @file voxl_imu_server_common.h
 *
 * This file contains common things shared between the server and client.
 * This is installed to /usr/include for use in your project
 */


#ifndef VOXL_IMU_SERVER_COMMON_H
#define VOXL_IMU_SERVER_COMMON_H


// The main structure containing accel, gyro, and temp data is imu_data_t
// which is defined in modal_imu_server_interface.h
// This structure is shared with the CX3 imu server and any potential new imu
// servers to ensure interoperability.
#include <modal_pipe_interfaces.h>


// config file exists here if you want to access it
#define VOXL_IMU_SERVER_CONF_FILE	"/etc/modalai/voxl-imu-server.conf"

// voxl-imu-server supports the two IMUs built into VOXL (0 and 1)
// plus up to two auxilliary IMUs that can be attached through expantion headers
// enumerated as imus 2 and 3. You must configure and enable auxiliary imus in
// the config file before they appear in the file system.
#define N_IMUS	(4)
#define MAX_IMU	(N_IMUS-1)

// These are the paths of the named pipe interfaces for each imu
// MODAL_PIPE_DEFAULT_BASE_DIR is defined in modal_pipe_common.h
#include <modal_pipe_common.h>

// Pipe names and locations. A Modal Pipe client only needs either name or
// location to connect. Both are here for completeness
#define VOXL_IMU0_NAME		"imu_apps"
#define VOXL_IMU0_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR VOXL_IMU0_NAME "/")
#define VOXL_IMU1_NAME		"imu1"
#define VOXL_IMU1_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR VOXL_IMU1_NAME "/")
#define VOXL_IMU2_NAME		"imu2"
#define VOXL_IMU2_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR VOXL_IMU2_NAME "/")
#define VOXL_IMU3_NAME		"imu3"
#define VOXL_IMU3_LOCATION	(MODAL_PIPE_DEFAULT_BASE_DIR VOXL_IMU3_NAME "/")

// helper arrays for accessing name strings by index
#define VOXL_IMU_NAME_ARRAY		{VOXL_IMU0_NAME, \
								 VOXL_IMU1_NAME, \
								 VOXL_IMU2_NAME, \
								 VOXL_IMU3_NAME}

#define VOXL_IMU_LOCATION_ARRAY	{VOXL_IMU0_LOCATION, \
								 VOXL_IMU1_LOCATION, \
								 VOXL_IMU2_LOCATION, \
								 VOXL_IMU3_LOCATION}


// standard gravity is used with the voxl-imu calibration tool
// e.g., once calibrated, the accelerometer should read an acceleration vector
// of length 9.80665 in any orientation assuming no noise and ideal sensor.
#ifndef G_TO_MS2
#define G_TO_MS2	(9.80665)
#endif

// shortcuts for converting between radians and degrees
#ifndef DEG_TO_RAD
#define DEG_TO_RAD	(3.14159265358979323846/180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG	(180.0/3.14159265358979323846)
#endif


#endif // VOXL_IMU_SERVER_COMMON_H
