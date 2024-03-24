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


#ifndef CAL_FILE_H
#define CAL_FILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "voxl_imu_server.h"

// available commnds for client to send to server over command pipe
#define COMMAND_START_CAL	"start_calibration"
#define COMMAND_STOP_CAL	"stop_calibration"

// This is here just for the voxl-imu-calibration tool
#define CALIBRATION_FILE "/data/modalai/voxl-imu-server.cal"


////////////////////////////////////////////////////////////////////////////////
// declare all config file fields here. define them in config_file.c
////////////////////////////////////////////////////////////////////////////////
extern int has_static_cal;
extern float gyro_offset[N_IMUS][3];
extern float accl_offset[N_IMUS][3];
extern float accl_scale[N_IMUS][3];

extern int has_baseline_temp[N_IMUS];
extern float gyro_baseline_temp[N_IMUS];
extern float accl_baseline_temp[N_IMUS];

extern int has_temp_cal[N_IMUS];
extern float gyro_drift_coeff[N_IMUS][3][3];
extern float accl_drift_coeff[N_IMUS][3][3];

// these are the temperature calibration coefficients corrected for the static
// offset in the normal calibration. imu-server uses only these!
extern float corrected_gyro_drift_coeff[N_IMUS][3][3];
extern float corrected_accl_drift_coeff[N_IMUS][3][3];

/**
 * load the config file and populate the above extern variables
 *
 * @return     0 on success, -1 on failure
 */
int cal_file_read(void);


/**
 * @brief      prints the current configuration values to the screen
 *
 *             this includes all of the extern variables listed above. If this
 *             is called before config_file_load then it will print the default
 *             values.
 *
 * @return     0 on success, -1 on failure
 */
int cal_file_print(void);

/**
 * writes the contents of the above extern variables to disk
 *
 * @return     0 on success, -1 on failure
 */
int cal_file_write(void);



/**
 * @brief      returns 1 is a valid static cal has been read
 */
int cal_file_has_static_cal(void);



#ifdef __cplusplus
}
#endif

#endif // end #define CAL_FILE_H
