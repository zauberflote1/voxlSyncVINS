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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR bus[id]INESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/


#include <stdio.h>
#include <voxl_io.h>
#include <string.h>

#include "config_file.h"
#include "cal_file.h"
#include "imu_interface.h"
#include "mpu9250.h"
#include "icm20948.h"
#include "icm42688.h"

#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif

#define SPI_SPEED_INIT 1000000  // 1mbit, need to see if we can go any slower

// colors for printing self-test results
#define COLOR_RED "\x1b[31m"
#define COLOR_GRN "\x1b[32m"
#define COLOR_RST "\x1b[0m"
#define LINE "=================================================================\n"


////////////////////////////////////////////////////////////////////////////////
// Local vars
////////////////////////////////////////////////////////////////////////////////
typedef enum imu_ic_t{
	IMU_IC_UNKNOWN	= 0,
	IMU_IC_MPU9250	= 1,
	IMU_IC_ICM20948	= 2,
	IMU_IC_ICM42688	= 3
} imu_ic_t;

// set on call to imu_detect
static imu_ic_t ic[N_IMUS];
static int board_id = BOARD_UNKNOWN;

// each imu gets its own shared memory for fifo reads
static uint8_t* fifo_buffer[N_IMUS];

// flag to indicate if the imu has been initialized and if the fifo is running
static int initialized[N_IMUS];
static int fifo_running[N_IMUS];
static int spi_running[N_IMUS];


static int imu_open_spi_bus(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}

	// allow this function to be called after init, just return
	if(spi_running[id]) return 0;

	if(voxl_spi_init(bus[id], SPI_MODE_3, SPI_SPEED_INIT)){
		fprintf(stderr, "ERROR: in imu_open_spi_bus[%d]\n", id);
		return -1;
	}

	fifo_buffer[id] = malloc(FIFO_READ_BUF_LEN);
	if(fifo_buffer[id]==NULL){
		fprintf(stderr, "ERROR: failed to allocate FIFO memory\n");
		return -1;
	}
	spi_running[id]=1;
	return 0;
}


static int imu_close_spi_bus(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}

	// allow this function to be called after cleanup or before init, just return
	if(!spi_running[id]) return 0;

	if(voxl_spi_close(bus[id])){
		fprintf(stderr, "ERROR: in imu_close_spi_bus[id]\n");
		return -1;
	}
	if(fifo_buffer[id]){
		free(fifo_buffer[id]);
		fifo_buffer[id]=0;
	}
	spi_running[id]=0;
	return 0;
}



int imu_self_test_print_results(int id, imu_self_test_result_t result)
{
	if(result.overall==TEST_PASS)	printf(COLOR_GRN LINE);
	else							printf(COLOR_RED LINE);

	if(result.overall==TEST_PASS)	printf(COLOR_GRN "RESULT FOR IMU%d on SPI BUS %d\n", id, bus[id]);
	else							printf(COLOR_RED "RESULT FOR IMU%d on SPI BUS %d\n", id, bus[id]);

	if(result.gyro[0]==TEST_PASS)	printf(COLOR_GRN "GYRO X: PASS\n");
	else							printf(COLOR_RED "GYRO X: FAIL\n");

	if(result.gyro[1]==TEST_PASS)	printf(COLOR_GRN "GYRO Y: PASS\n");
	else							printf(COLOR_RED "GYRO Y: FAIL\n");

	if(result.gyro[2]==TEST_PASS)	printf(COLOR_GRN "GYRO Z: PASS\n");
	else							printf(COLOR_RED "GYRO Z: FAIL\n");

	if(result.accl[0]==TEST_PASS)	printf(COLOR_GRN "ACCL X: PASS\n");
	else							printf(COLOR_RED "ACCL X: FAIL\n");

	if(result.accl[1]==TEST_PASS)	printf(COLOR_GRN "ACCL Y: PASS\n");
	else							printf(COLOR_RED "ACCL Y: FAIL\n");

	if(result.accl[2]==TEST_PASS)	printf(COLOR_GRN "ACCL Z: PASS\n");
	else							printf(COLOR_RED "ACCL Z: FAIL\n");

	if(result.overall==TEST_PASS)	printf(COLOR_GRN "OVERALL PASS\n");
	else if(result.overall==TEST_FAIL_COM){
		printf(COLOR_RED "OVERALL FAIL: COULD NOT COMMUNICATE WITH IMU\n");
	}
	else							printf(COLOR_RED "OVERALL FAIL\n");

	if(result.overall==TEST_PASS)	printf(COLOR_GRN LINE);
	else							printf(COLOR_RED LINE);

	// make sure color is back to normal before exiting
	printf(COLOR_RST);
	return 0;
}


int imu_print_data(int id, imu_data_t data)
{
	printf("imu%d: A:%6.2f %6.2f %6.2f G:%6.2f %6.2f %6.2f T: %6.2f ts: %lu\n",\
		id,
		(double)data.accl_ms2[0],(double)data.accl_ms2[1],(double)data.accl_ms2[2],\
		(double)data.gyro_rad[0],(double)data.gyro_rad[1],(double)data.gyro_rad[2],\
		(double)data.temp_c, data.timestamp_ns);
	return 0;
}



int imu_apply_calibration(int id, imu_data_t* data)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	// apply the base cal offsets
	for(int i=0;i<3;i++){
		data->gyro_rad[i] -= gyro_offset[id][i];
		data->accl_ms2[i] -= accl_offset[id][i];
	}

	// apply temp cal offset if it exists
	if(has_baseline_temp[id] && has_temp_cal[id]){
		float t  = data->temp_c;
		float tt = t*t;
		for(int i=0;i<3;i++){
			data->gyro_rad[i] -= corrected_gyro_drift_coeff[id][i][0] + \
								(corrected_gyro_drift_coeff[id][i][1]*t) +\
								(corrected_gyro_drift_coeff[id][i][2]*tt);
			data->accl_ms2[i] -= corrected_accl_drift_coeff[id][i][0] + \
								(corrected_accl_drift_coeff[id][i][1]*t) +\
								(corrected_accl_drift_coeff[id][i][2]*tt);
			/*
			printf(" gyro %d %f\n", i, corrected_gyro_drift_coeff[id][i][0] + \
								(corrected_gyro_drift_coeff[id][i][1]*t) +\
								(corrected_gyro_drift_coeff[id][i][2]*tt));
			printf(" accl %d %f\n", i, corrected_accl_drift_coeff[id][i][0] + \
								(corrected_accl_drift_coeff[id][i][1]*t) +\
								(corrected_accl_drift_coeff[id][i][2]*tt));
			*/
		}
	}

	// apply scale AFTER offset
	for(int i=0;i<3;i++){
		data->accl_ms2[i] /=  accl_scale[id][i];
	}

	return 0;
}


int imu_rotate_to_common_frame(__attribute__((unused)) int id, imu_data_t* data)
{
	// BOARD_M0054 and MOO53 both have 1 appsproc 42688 imu
	if(board_id == BOARD_M0054 || board_id == BOARD_M0053){
		data->gyro_rad[1] = -data->gyro_rad[1];
		data->accl_ms2[1] = -data->accl_ms2[1];
		data->gyro_rad[2] = -data->gyro_rad[2];
		data->accl_ms2[2] = -data->accl_ms2[2];
	}
	else if(board_id == BOARD_M0104){
		data->gyro_rad[0] = -data->gyro_rad[0];
		data->accl_ms2[0] = -data->accl_ms2[0];
		data->gyro_rad[1] = -data->gyro_rad[1];
		data->accl_ms2[1] = -data->accl_ms2[1];
	}
	return 0;
}


int imu_detect_board()
{
	// default to VOXL2 (M0054)
	board_id = BOARD_UNKNOWN;

	FILE *file = fopen("/etc/version", "r");

	if (file == NULL) {
		perror("Error opening /etc/modalai");
		return -1;
	}

	char line[512];

	if(fgets(line, sizeof(line), file) == NULL) {
		fprintf(stderr, "ERROR failed to read line from /etc/version, assuming VOXL2\n");
		return -1;
	}

	// Check if the substring is present in the line
	if(strstr(line, "M0054") != NULL){
		printf("Detected M0054 VOXL2\n");
		board_id = BOARD_M0054;
	}
	else if(strstr(line, "M0104") != NULL){
		printf("Detected M0104 VOXL2 mini\n");
		board_id = BOARD_M0104;
	}
	else{
		fprintf(stderr, "ERROR failed to pick board from /etc/version\n");
		board_id = BOARD_UNKNOWN;
	}

	fclose(file);

	return 0;
}


int imu_detect(int id)
{
	if(imu_open_spi_bus(id)) return -1;

	// check if imu has already been identified, no need to do it again
	if(ic[id]!=IMU_IC_UNKNOWN) return 0;

	// Check for ICM20948
	if(icm20948_detect(bus[id])==0){
		ic[id] = IMU_IC_ICM20948;
		printf("detected ICM20948 on spi bus[id] %d\n", bus[id]);
		return 0;
	}

	// Check for ICM42688
	if(icm42688_detect(bus[id])==0){
		ic[id] = IMU_IC_ICM42688;
		printf("detected ICM42688 on spi bus[id] %d\n", bus[id]);
		return 0;
	}

	// Check for MPU9250
	if(mpu9250_detect(bus[id])==0){
		ic[id] = IMU_IC_MPU9250;
		printf("detected MPU9250 on spi bus[id] %d\n", bus[id]);
		return 0;
	}

	fprintf(stderr, "ERROR: imu_detect failed to find a valid part\n");
	return -1;
}


int imu_init(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}

	// already initialized, nothing to do
	if(initialized[id]) return 0;

	if(imu_open_spi_bus(id)) return -1;
	if(imu_detect(id)) return -1;

	switch(ic[id]){
	case IMU_IC_MPU9250:
		if(mpu9250_init(bus[id], imu_sample_rate_hz[id], imu_lp_cutoff_freq_hz[id])) return -1;
		break;
	case IMU_IC_ICM20948:
		if(icm20948_init(bus[id], imu_sample_rate_hz[id], imu_lp_cutoff_freq_hz[id])) return -1;
		break;
	case IMU_IC_ICM42688:
		if(icm42688_init(bus[id], imu_sample_rate_hz[id], imu_lp_cutoff_freq_hz[id])) return -1;
		break;
	default:
		fprintf(stderr, "ERROR: in imu_init() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}
	initialized[id]=1;	// all initialized now
	fifo_running[id]=0;	// but we haven't started the fifo yet!!!
	return 0;
}


int imu_close(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	if(!initialized[id]) return 0;

	// slow spi speed down for register config
	voxl_spi_set_freq(bus[id], SPI_SPEED_INIT);

	switch(ic[id]){
	case IMU_IC_MPU9250:
		if(mpu9250_close(bus[id])) return -1;
		break;
	case IMU_IC_ICM20948:
		if(icm20948_close(bus[id])) return -1;
		break;
	case IMU_IC_ICM42688:
		if(icm42688_close(bus[id])) return -1;
		break;
	default:
		fprintf(stderr, "ERROR: in imu_close() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}
	initialized[id]=0;
	fifo_running[id]=0;

	if(imu_close_spi_bus(id)) return -1;

	return 0;
}



int imu_basic_read(int id, imu_data_t* data)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!initialized[id])){
		fprintf(stderr, "ERROR: tried to call %s without initializing IMU first\n", __FUNCTION__);
		return -1;
	}

	switch(ic[id]){
	case IMU_IC_MPU9250:
		return mpu9250_basic_read(bus[id], data);
	case IMU_IC_ICM20948:
		return icm20948_basic_read(bus[id], data);
	case IMU_IC_ICM42688:
		return icm42688_basic_read(bus[id], data);
	default:
		fprintf(stderr, "ERROR: in imu_basic_read() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}
	return 0;
}


int imu_fifo_reset(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!initialized[id])){
		fprintf(stderr, "ERROR: tried to call %s without initializing IMU first\n", __FUNCTION__);
		return -1;
	}

	// slow spi speed down for register config
	voxl_spi_set_freq(bus[id], SPI_SPEED_INIT);

	switch(ic[id]){
	case IMU_IC_MPU9250:
		if(mpu9250_fifo_reset(bus[id])) return -1;
		break;
	case IMU_IC_ICM20948:
		if(icm20948_fifo_reset(bus[id])) return -1;
		break;
	case IMU_IC_ICM42688:
		if(icm42688_fifo_reset(bus[id])) return -1;
		break;
	default:
		fprintf(stderr, "ERROR: in imu_fifo_reset() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}
	fifo_running[id]=1;
	return 0;
}


int imu_fifo_start(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!initialized[id])){
		fprintf(stderr, "ERROR: tried to call %s without initializing IMU first\n", __FUNCTION__);
		return -1;
	}

	// already running, nothing to do
	if(fifo_running[id]) return 0;

	// slow spi speed down for register config
	voxl_spi_set_freq(bus[id], SPI_SPEED_INIT);

	switch(ic[id]){
	case IMU_IC_MPU9250:
		if(mpu9250_fifo_start(bus[id])) return -1;
		break;
	case IMU_IC_ICM20948:
		if(icm20948_fifo_start(bus[id])) return -1;
		break;
	case IMU_IC_ICM42688:
		if(icm42688_fifo_start(bus[id])) return -1;
		break;
	default:
		fprintf(stderr, "ERROR: in imu_fifo_start() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}
	fifo_running[id]=1;
	printf("started fifo on imu%d\n", id);
	return 0;
}


int imu_fifo_stop(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!initialized[id])){
		fprintf(stderr, "ERROR: tried to call %s without initializing IMU first\n", __FUNCTION__);
		return -1;
	}
	if(!fifo_running[id]) return 0;

	// slow spi speed down for register config
	voxl_spi_set_freq(bus[id], SPI_SPEED_INIT);

	switch(ic[id]){
	case IMU_IC_MPU9250:
		if(mpu9250_fifo_stop(bus[id])) return -1;
		break;
	case IMU_IC_ICM20948:
		if(icm20948_fifo_stop(bus[id])) return -1;
		break;
	case IMU_IC_ICM42688:
		if(icm42688_fifo_stop(bus[id])) return -1;
		break;
	default:
		fprintf(stderr, "ERROR: in imu_fifo_stop() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}
	fifo_running[id]=0;
	return 0;
}

int imu_is_fifo_running(int id)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	return fifo_running[id];
}


int imu_fifo_read(int id, imu_data_t* data, int* packets)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}
	if(unlikely(!initialized[id])){
		fprintf(stderr, "ERROR: tried to call %s without initializing IMU first\n", __FUNCTION__);
		return -1;
	}
	if(!fifo_running[id]) return 0;

	int ret;

	switch(ic[id]){
	case IMU_IC_MPU9250:
		ret = mpu9250_fifo_read(bus[id], data, packets, fifo_buffer[id]);
		break;
	case IMU_IC_ICM20948:
		ret = icm20948_fifo_read(bus[id], data, packets, fifo_buffer[id]);
		break;
	case IMU_IC_ICM42688:
		ret = icm42688_fifo_read(bus[id], data, packets, fifo_buffer[id]);
		break;
	default:
		fprintf(stderr, "ERROR: in imu_fifo_read() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}

	if(ret) return ret;

	/*
	// debug some scales and offsets to test calibration
	for(int i=0; i<*packets; i++){
		float t;

		t = data[i].accl_ms2[0];
		data[i].accl_ms2[0] = (t * 1.5f) + 0.0f;
		t = data[i].accl_ms2[1];
		data[i].accl_ms2[1] = (t * 1.0f) + 5.0f;
		t = data[i].accl_ms2[2];
		data[i].accl_ms2[2] = (t * 1.5f) + 5.0f;
	}
	*/

/*
	// debug some scales and offsets to test temp calibration
	for(int i=0; i<*packets; i++){

		// scaled so x should be about -1 to +1 through the test
		float x = (data[i].temp_c - 45.0f)/15.0f;

		data[i].gyro_rad[0] = data[i].gyro_rad[0] + (10*x) + (10*x*x);
		data[i].gyro_rad[1] = data[i].gyro_rad[1] - (10*x) - (10*x*x);
		data[i].gyro_rad[2] = data[i].gyro_rad[2] + (10*x) - (10*x*x);
		data[i].accl_ms2[0] = data[i].accl_ms2[0] + (10*x) + (10*x*x);
		data[i].accl_ms2[1] = data[i].accl_ms2[1] - (10*x) - (10*x*x);
		data[i].accl_ms2[2] = data[i].accl_ms2[2] + (10*x) - (10*x*x);
	}
*/
	
	return 0;
}


int imu_self_test(int id, imu_self_test_result_t* result)
{
	// sanity checks
	if(unlikely(id<0||id>MAX_IMU)){
		fprintf(stderr, "ERROR: in %s, id out of bounds\n", __FUNCTION__);
		return -1;
	}

	switch(ic[id]){
	case IMU_IC_MPU9250:
		return mpu9250_self_test(bus[id], result);
	case IMU_IC_ICM20948:
		return icm20948_self_test(bus[id], result);
	case IMU_IC_ICM42688:
		return icm42688_self_test(bus[id], result);
	default:
		fprintf(stderr, "ERROR: in imu_self_test() invalid ic part number\n");
		fprintf(stderr, "call imu_detect first!\n");
		return -1;
	}
	return 0;
}
