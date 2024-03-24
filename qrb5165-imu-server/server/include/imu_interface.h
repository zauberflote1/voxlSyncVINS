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

#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <voxl_imu_server.h>


/**
 * 0 means pass, -1 means fail
 */
#define TEST_PASS		0
#define TEST_FAIL		-1
#define TEST_FAIL_COM	-2 // indicates couldn't communicate with imu

typedef struct imu_self_test_result_t{
	int overall;	// overall result, TEST_PASS or TEST_FAIL
	int gyro[3];	// gyro results for each axis TEST_PASS or TEST_FAIL
	int accl[3];	// accl results for each axis TEST_PASS or TEST_FAIL
} imu_self_test_result_t;

#define SELF_TEST_INITIALIZER {TEST_FAIL_COM, {TEST_FAIL,TEST_FAIL,TEST_FAIL}, {TEST_FAIL,TEST_FAIL,TEST_FAIL}}

#define FIFO_READ_BUF_LEN	4096

// VOXL board revisions
#define BOARD_M0104 3 // voxl2 mini
#define BOARD_M0053 2 // rb5-flight, irrelevant since m0053 doesn't have an apps proc imu
#define BOARD_M0054 1 // voxl2
#define BOARD_UNKNOWN 0

////////////////////////////////////////////////////////////////////////////////
// helper functions for printing data
////////////////////////////////////////////////////////////////////////////////
int imu_self_test_print_results(int id, imu_self_test_result_t result);
int imu_print_data(int id, imu_data_t data);


////////////////////////////////////////////////////////////////////////////////
// helper functions for manipulating imu data
////////////////////////////////////////////////////////////////////////////////
int imu_load_calibration_file(void);
int imu_apply_calibration(int id, imu_data_t* data);
int imu_detect_board(void);
int imu_rotate_to_common_frame(int id, imu_data_t* data);

////////////////////////////////////////////////////////////////////////////////
// everything below here must have a device-specific implementation
// eg, mpu9250_init() icm20948_init(), and icm42688_init()
////////////////////////////////////////////////////////////////////////////////

int imu_detect(int id);
int imu_init(int id);
int imu_close(int id);
int imu_basic_read(int id, imu_data_t* data);
int imu_fifo_reset(int id);
int imu_fifo_start(int id);
int imu_fifo_stop(int id);
int imu_is_fifo_running(int id);
int imu_fifo_read(int id, imu_data_t* data, int* packets);
int imu_self_test(int id, imu_self_test_result_t* result);

#ifdef __cplusplus
}
#endif


#endif // IMU_INTERFACE_H
