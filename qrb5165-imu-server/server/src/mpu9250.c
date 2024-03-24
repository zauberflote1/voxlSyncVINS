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


#include <stdio.h>
#include <unistd.h> // for usleep
#include <string.h> // for memset
#include <math.h>

#include <voxl_io.h>
#include "mpu9250.h"
#include "mpu9250_defs.h"
#include "global_debug_flags.h"
#include "timestamp.h"

// we are locked into used 2000dps/16g FSRs with the HIRES FIFO mode!!
static const float gyro_scale_16 = (2000.0*DEG_TO_RAD)/32767.0;
static const float accl_scale_16 = (16.0*G_TO_MS2)/32767.0;

#define SPI_SPEED_BASE 1000000  // 1mbit, that's as slow as DSPAL will let us go
#define SPI_SPEED_DATA 5000000  // limit is 10mbit, go at half that

static double configured_odr_hz[16];

// ratio of imu clock speed to apps proc
// imu runs considerably slower. this value is estimated in realtime
#define STARTING_CLOCK_RATIO	1.0
static double clock_ratio[16];
// keep track of timestamp so we can pick up next read
static int last_read_was_good[16];
static int64_t last_ts_ns[16];





int mpu9250_detect(int bus)
{
	uint8_t c;
	if(voxl_spi_read_reg_byte(bus, WHO_AM_I_MPU9250, &c)){
		fprintf(stderr, "ERROR in mpu9250_fifo_detect() reading WHOAMI reg\n");
		return -1;
	}

	// 0x71 for mpu9250, ox73 or 0x75 for mpu9255, or 0x68 for mpu9150
	// 0x70 for mpu6500,  0x68 or 0x69 for mpu6050
	if(c!=0x68 && c!=0x69 && c!=0x70 && c!=0x71 && c!=0x73 && c!=75) return -1;

	return 0;
}


int mpu9250_init(int bus, double sample_rate_hz, int lp_cutoff_freq_hz)
{
	uint8_t val, DIV, UI_FILT_BW;

	if(clock_ratio[bus]==0.0) clock_ratio[bus] = STARTING_CLOCK_RATIO;

	// find which output data rate divider most closely matches desired sample rate
	DIV=255; // start with slowest in case a faster match isn't found
	for(int i=0;i<=255;i++){
		// here i is the value that will go into GYRO_SMPLRT_DIV
		// ODR = 1125/(1+i)
		// middle is the sample rate halfway between i and i+1
		double middle = ((1000.0/(i+1))+(1000.0/(i+2)))/2.0;
		if(sample_rate_hz>middle){
			DIV=i;
			break;
		}
	}
	// save the real ODR as configured for later
	configured_odr_hz[bus] = 1000.0/(DIV+1);
	printf("using ODR=%6.2fhz which is the nearest the mpu9250 can get to %4.1f\n",\
										configured_odr_hz[bus], sample_rate_hz);

	// round to nearest available filter setting
	if     (lp_cutoff_freq_hz >= 322) UI_FILT_BW = 0; // nominally 460 hz
	else if(lp_cutoff_freq_hz >= 138) UI_FILT_BW = 1; // nominally 184 hz
	else if(lp_cutoff_freq_hz >= 67)  UI_FILT_BW = 2; // nominally 92 hz
	else if(lp_cutoff_freq_hz >= 30)  UI_FILT_BW = 3; // nominally 41 hz
	else if(lp_cutoff_freq_hz >= 15)  UI_FILT_BW = 4; // nominally 20 hz
	else if(lp_cutoff_freq_hz >= 7)   UI_FILT_BW = 5; // nominally 10 hz
	else                              UI_FILT_BW = 6; // nominally 5 hz


	////////////////////////////////////////////////////////////////////////////
	// now set up registers
	////////////////////////////////////////////////////////////////////////////

	// reset the device
	val = H_RESET;
	voxl_spi_write_reg_byte(bus, PWR_MGMT_1, val);
	usleep(10000); // wait for reset

	// set up FSR
	val = GYRO_FSR_CFG_2000 | FCHOICE_B_DLPF_EN;
	voxl_spi_write_reg_byte(bus, GYRO_CONFIG, val);

	val = ACCEL_FSR_CFG_16G;
	voxl_spi_write_reg_byte(bus, ACCEL_CONFIG, val);

	// set up lowpass filter
	val = FIFO_MODE_REPLACE_OLD | UI_FILT_BW;
	voxl_spi_write_reg_byte(bus, CONFIG, val);

	// enable secret 4k fifo mode
	val = ACCEL_FCHOICE_1KHZ | BIT_FIFO_SIZE_4096 | UI_FILT_BW;
	voxl_spi_write_reg_byte(bus, ACCEL_CONFIG_2 , val);

	// set sample rate
	val = DIV;
	voxl_spi_write_reg_byte(bus, SMPLRT_DIV , val);

	// next step is for the user to enable the fifo
	usleep(10000);
	return 0;
}


int mpu9250_close(int bus)
{
	mpu9250_fifo_stop(bus);
	return 0;
}


int mpu9250_basic_read(int bus, imu_data_t* data)
{
	uint8_t raw[14];
	int64_t time_read_start = voxl_apps_time_monotonic_ns();

	if(voxl_spi_read_reg(bus, ACCEL_XOUT_H, raw, 14)){
		fprintf(stderr, "ERROR in mpu9250_basic_read, failed to read register\n");
		return -1;
	}

	// construct 16-bit values
	int16_t ax16,ay16,az16,gx16,gy16,gz16,tr16;
	ax16 = (raw[0] << 8) | raw[1];
	ay16 = (raw[2] << 8) | raw[3];
	az16 = (raw[4] << 8) | raw[5];
	tr16 = (raw[6] << 8) | raw[7];
	gx16 = (raw[8] << 8) | raw[9];
	gy16 = (raw[10]<< 8) | raw[11];
	gz16 = (raw[12]<< 8) | raw[13];

	if(ax16==0 && ay16==0 && az16==0 && gx16==0 && gy16==0 && gz16==0 && tr16==0){
		fprintf(stderr, "ERROR: in %s read all 0's\n", __FUNCTION__);
		return -1;
	}

	data->accl_ms2[0]  = ax16 * accl_scale_16;
	data->accl_ms2[1]  = ay16 * accl_scale_16;
	data->accl_ms2[2]  = az16 * accl_scale_16;
	data->gyro_rad[0]  = gx16 * gyro_scale_16;
	data->gyro_rad[1]  = gy16 * gyro_scale_16;
	data->gyro_rad[2]  = gz16 * gyro_scale_16;
	data->temp_c       = (tr16 / TEMP_SENSITIVITY) + 21.0f;
	data->timestamp_ns = time_read_start;
	return 0;
}


int mpu9250_fifo_reset(int bus)
{
	uint8_t val;

	if(voxl_spi_write_reg_byte(bus, FIFO_EN, 0)){
		fprintf(stderr, "ERROR in mpu9250_fifo_start()) writing to SPI bus\n");
		return -1;
	}

	if(voxl_spi_write_reg_byte(bus,USER_CTRL, BIT_FIFO_RST)){
		fprintf(stderr, "ERROR in mpu9250_fifo_reset() writing to SPI bus\n");
		return -1;
	}
	usleep(50000); // invensense recommended wait

	if(voxl_spi_write_reg_byte(bus, USER_CTRL, BIT_FIFO_EN)){
		fprintf(stderr, "ERROR in mpu9250_fifo_start()) writing to SPI bus\n");
		return -1;
	}

	val = FIFO_TEMP_EN | FIFO_GYRO_X_EN | FIFO_GYRO_Y_EN | FIFO_GYRO_Z_EN | FIFO_ACCEL_EN;
	if(voxl_spi_write_reg_byte(bus, FIFO_EN, val)){
		fprintf(stderr, "ERROR in mpu9250_fifo_start() writing to SPI bus\n");
		return -1;
	}

	return 0;
}


int mpu9250_fifo_start(int bus)
{
	// mpu9250 is a bit finicky starting fifo, best to just do a reset
	// which will leave the fifo running
	return mpu9250_fifo_reset(bus);
}


int mpu9250_fifo_stop(int bus)
{
	uint8_t val = 0;
	if(voxl_spi_write_reg_byte(bus, FIFO_EN, val)){
		fprintf(stderr, "ERROR in mpu9250_fifo_stop() writing to SPI bus\n");
		return -1;
	}
	if(voxl_spi_write_reg_byte(bus, USER_CTRL, val)){
		fprintf(stderr, "ERROR in mpu9250_fifo_stp[()) writing to SPI bus\n");
		return -1;
	}
	return 0;
}


int mpu9250_fifo_read(int bus, imu_data_t* data, int* packets, uint8_t* fifo_buffer)
{
	static int n_empty_reads = 0;

	int records = 0; // num packets read as reported by dsp
	*packets = 0; // make sure we indicate nothing has been read if we return early

	const int p_len = 14; // accel plus gyro plus temp
	const int p_warn = 260;
	const int p_max = 292;

	// read time both before and after the SPI transfer
	int64_t time_before_read = voxl_apps_time_monotonic_ns();

	// read as much as we can from the fifo into our own buffer
	int ret = voxl_spi_read_imu_fifo(bus, FIFO_COUNTH, FIFO_R_W, p_len, \
								&records, fifo_buffer, FIFO_READ_BUF_LEN, \
								SPI_SPEED_BASE, SPI_SPEED_DATA);
	if(ret){
		last_read_was_good[bus]=0;
		fprintf(stderr, "ERROR mpu9250_fifo_read: DSP reader had an issue\n");
		return -1;
	}
	if(records==0){
		// should never pass mmore than a few empty reads in a row
		n_empty_reads++;
		if (n_empty_reads >= 10){
			fprintf(stderr, "ERROR mpu9250_fifo_read: buffer was empty %d times in a row\n", n_empty_reads);
			return -1;
		}
		return 0;
	}
	if(records>p_max){
		fprintf(stderr, "ERROR mpu9250_fifo_read: impossible value in FIFO_COUNT: %d\n",\
																		records);
		return 0;
	}

	// fifo can actually hold p_max but if we are reading >=p_warn then
	// something has gone wrong or CPU is stalled
	if(en_print_fifo_count && records>=p_warn && records<(p_max-1)){
		fprintf(stderr, "WARNING FIFO ALMOST FULL\n");
	}
	// fifo rarely reports p_max even when full but p_max-1 shows up frequently
	if(records>=(p_max-1)){
		if(en_print_fifo_count){
			fprintf(stderr, "WARNING FIFO COMPLETELY FULL\n");
		}
		last_read_was_good[bus]=0; // assume packets were lost since the last read
	}


	// calculate the best-guess filtered timestamp
	int64_t filtered_ts_ns = calc_filtered_ts_ns(time_before_read, records, &clock_ratio[bus],\
							last_ts_ns[bus], configured_odr_hz[bus], last_read_was_good[bus]);

	// find the timestep based on either the last fifo sample (smooth) or
	// based on the sample rate (jumpy)
	int64_t new_dt;
	if(last_read_was_good[bus]){
		new_dt = (filtered_ts_ns - last_ts_ns[bus])/records;
	}
	else{
		new_dt = 1000000000.0/(configured_odr_hz[bus]/clock_ratio[bus]);
	}

	// done using this flag now, set it for the next loop
	// it may go back to 0 later if there was a parsing problem
	last_read_was_good[bus] = 1;
	last_ts_ns[bus] = filtered_ts_ns;

	// now loop through and decode each packet
	for(int i=0;i<records;i++){
		uint8_t* base = fifo_buffer + (p_len*i); // start of current packet

		// grab 16-bit accel and gyro data
		int16_t ax16,ay16,az16,gx16,gy16,gz16,tr16;
		ax16 = (base[0] << 8) | base[1];
		ay16 = (base[2] << 8) | base[3];
		az16 = (base[4] << 8) | base[5];
		tr16 = (base[6] << 8) | base[7];
		gx16 = (base[8] << 8) | base[9];
		gy16 = (base[10]<< 8) | base[11];
		gz16 = (base[12]<< 8) | base[13];

		if(ax16==0 && ay16==0 && az16==0 && gx16==0 && gy16==0 && gz16==0 && tr16==0){
			fprintf(stderr, "ERROR: in mpu9250_fifo_read, read all 0's\n");
			return -1;
		}

		data[i].accl_ms2[0]  = ax16 * accl_scale_16;
		data[i].accl_ms2[1]  = ay16 * accl_scale_16;
		data[i].accl_ms2[2]  = az16 * accl_scale_16;
		data[i].gyro_rad[0]  = gx16 * gyro_scale_16;
		data[i].gyro_rad[1]  = gy16 * gyro_scale_16;
		data[i].gyro_rad[2]  = gz16 * gyro_scale_16;
		data[i].temp_c = (tr16 / TEMP_SENSITIVITY) + 21.0f;

		// place timestamp based on our previous filtering
		data[i].timestamp_ns = filtered_ts_ns - ((records-i-1)*new_dt);
	}

	*packets = records;
	n_empty_reads = 0;

	return 0;
}


static int __self_test_sample(int bus, int32_t gyro_avg[3], int32_t accl_avg[3])
{
	int32_t gyro_sum[3] = {0,0,0};
	int32_t accl_sum[3] = {0,0,0};
	int i;
	const int samples = 200;

	for(i=0;i<samples;i++){
		uint8_t raw[14];

		if(voxl_spi_read_reg(bus, ACCEL_XOUT_H, raw, 14)){
			fprintf(stderr, "ERROR in mpu9250_basic_read, failed to read register\n");
			return -1;
		}

		// construct 16-bit values
		int16_t axr,ayr,azr,gxr,gyr,gzr;
		axr = (raw[0] << 8) | raw[1];
		ayr = (raw[2] << 8) | raw[3];
		azr = (raw[4] << 8) | raw[5];
		gxr = (raw[8] << 8) | raw[9];
		gyr = (raw[10]<< 8) | raw[11];
		gzr = (raw[12]<< 8) | raw[13];

		gyro_sum[0]+=gxr;
		gyro_sum[1]+=gyr;
		gyro_sum[2]+=gzr;
		accl_sum[0]+=axr;
		accl_sum[1]+=ayr;
		accl_sum[2]+=azr;
		usleep(1000);
	}

	gyro_avg[0] = (gyro_sum[0] / samples);
	gyro_avg[1] = (gyro_sum[1] / samples);
	gyro_avg[2] = (gyro_sum[2] / samples);
	accl_avg[0] = (accl_sum[0] / samples);
	accl_avg[1] = (accl_sum[1] / samples);
	accl_avg[2] = (accl_sum[2] / samples);

	return 0;
}



int mpu9250_self_test(int bus, imu_self_test_result_t* result)
{
	uint8_t val;
	int i;

	// store the outputs of sampling here
	int32_t gyro_avg_normal[3];
	int32_t gyro_avg_test_on[3];
	int32_t accl_avg_normal[3];
	int32_t accl_avg_test_on[3];
	uint32_t gyro_response[3];		// gyro response as abs(test_on - test_off)
	uint32_t accl_response[3];		// accl response as abs(test_on - test_off)
	uint8_t  gyro_factory_data[3];	// factory gyro test data stored in chip
	uint8_t  accl_factory_data[3];	// factory accl test data stored in chip
	float gyro_factory_response[3];	// output from response equation
	float accl_factory_response[3];	// output from response equation

	printf("Starting self-test on MPU9250 on SPI bus %d\n", bus);

	// indicate all failures so if we quit early it indicate a failure
	memset(result,1,sizeof(imu_self_test_result_t));

	////////////////////////////////////////////////////////////////////////////
	// reset and set up registers for self test
	////////////////////////////////////////////////////////////////////////////

	val = H_RESET;
	voxl_spi_write_reg_byte(bus, PWR_MGMT_1, val);
	usleep(10000); // wait for reset

	// set sample rate to 1khz
	val = 0;
	voxl_spi_write_reg_byte(bus, SMPLRT_DIV , val);

	// set up FSR to 250dps/2g
	val = GYRO_FSR_CFG_250 | FCHOICE_B_DLPF_EN;
	voxl_spi_write_reg_byte(bus, GYRO_CONFIG, val);
	val = ACCEL_FSR_CFG_2G;
	voxl_spi_write_reg_byte(bus, ACCEL_CONFIG, val);

	// set up lowpass filter to 92hz
	val = 0x02;
	voxl_spi_write_reg_byte(bus, CONFIG, val);
	voxl_spi_write_reg_byte(bus, ACCEL_CONFIG_2 , val);

	// wait 100ms for outputs to settle after configuration
	usleep(100000);

	////////////////////////////////////////////////////////////////////////////
	// sample data before and after enabling self test
	////////////////////////////////////////////////////////////////////////////

	// before
	if(__self_test_sample(bus, gyro_avg_normal, accl_avg_normal)){
		fprintf(stderr, "self-test failed trying to read normal data\n");
		return -1;
	}

	// turn on self-test and wait 200ms for oscillation to stabilize
	voxl_spi_write_reg_byte(bus, ACCEL_CONFIG, 0xE0);
	voxl_spi_write_reg_byte(bus, GYRO_CONFIG, 0xE0);
	usleep(200000);

	// sample data with self-test enabled
	if(__self_test_sample(bus, gyro_avg_test_on, accl_avg_test_on)){
		fprintf(stderr, "self-test failed trying to read after turning on ST\n");
		return -1;
	}

	////////////////////////////////////////////////////////////////////////////
	// calculate delta with factory register data
	////////////////////////////////////////////////////////////////////////////

	// grab factory self-test data
	if(voxl_spi_read_reg(bus, SELF_TEST_X_GYRO, gyro_factory_data, 3)){
		fprintf(stderr, "self-test failed trying to read factory gyro results\n");
		return -1;
	}
	if(voxl_spi_read_reg(bus, SELF_TEST_X_ACCEL, accl_factory_data, 3)){
		fprintf(stderr, "self-test failed trying to read factory accel results\n");
		return -1;
	}

	// all done with registers, reset
	val = H_RESET;
	voxl_spi_write_reg_byte(bus, PWR_MGMT_1, val);

	for(i=0; i<3; i++){
		gyro_response[i] = INV_ABS(gyro_avg_test_on[i] - gyro_avg_normal[i]);
		accl_response[i] = INV_ABS(accl_avg_test_on[i] - accl_avg_normal[i]);
	}

	// convert self test register data to real response at 250dps/2g FSR
	gyro_factory_response[0] = (2620.0)*(pow( 1.01 , ((double)gyro_factory_data[0] - 1.0) ));
	gyro_factory_response[1] = (2620.0)*(pow( 1.01 , ((double)gyro_factory_data[1] - 1.0) ));
	gyro_factory_response[2] = (2620.0)*(pow( 1.01 , ((double)gyro_factory_data[2] - 1.0) ));
	accl_factory_response[0] = (2620.0)*(pow( 1.01 , ((double)accl_factory_data[0] - 1.0) ));
	accl_factory_response[1] = (2620.0)*(pow( 1.01 , ((double)accl_factory_data[1] - 1.0) ));
	accl_factory_response[2] = (2620.0)*(pow( 1.01 , ((double)accl_factory_data[2] - 1.0) ));


	////////////////////////////////////////////////////////////////////////////
	// Gyro calculation
	////////////////////////////////////////////////////////////////////////////

	// zero out results indicating all pass before checking for failures
	memset(result,0,sizeof(imu_self_test_result_t));

	uint32_t gyro_sensitivity_1dps = 32768 / 250;
	for(i=0; i<3; i++){
		// normal test if factory results are present in registers
		if(gyro_factory_response[i]){
			// first check ratio of our test to factory test
			float ratio = ((float)gyro_response[i]) / ((float)gyro_factory_response[i]);
			if((ratio <= MIN_RATIO_GYRO) || (ratio >= MAX_RATIO_GYRO)){
				result->gyro[i] = TEST_FAIL;
				result->overall = TEST_FAIL;
			}
			// also check that steady offset is in bounds
			if((INV_ABS(gyro_avg_normal[i]) > (int32_t)(MAX_ST_GYRO_OFFSET_DPS * gyro_sensitivity_1dps))){
				result->gyro[i] = TEST_FAIL;
				result->overall = TEST_FAIL;
			}
			printf("gyro axis: %d ratio: %5.3f response: %5.0f factory: %5.0f offset_dps: %5.2f\n",\
				i, (double)ratio, (double)gyro_response[i], (double)gyro_factory_response[i],\
				(double)gyro_avg_normal[i]/(double)gyro_sensitivity_1dps);
		}

		// aux test if factory data is not present
		else{
			fprintf(stderr, "WARNING no factory data present, falling back to aux test\n");
			if(gyro_response[i] < (MIN_ST_GYRO_DPS * gyro_sensitivity_1dps)){
				result->gyro[i] = TEST_FAIL;
				result->overall = TEST_FAIL;
			}
		}
	}


	////////////////////////////////////////////////////////////////////////////
	// Accel calculation
	////////////////////////////////////////////////////////////////////////////

	// Check Accel self-test result
	uint32_t accel_sensitivity_1g = 32768/2;
	for(i=0;i<3;i++){
		// normal test if factory results are present in registers
		if(accl_factory_response[i]){
			float ratio = ((float)accl_response[i]) / ((float)accl_factory_response[i]);
			printf("accl axis: %d ratio: %5.3f response: %5.0f factory: %5.0f\n",\
					i, (double)ratio, (double)accl_response[i], (double)accl_factory_response[i]);
			if((ratio >= MAX_RATIO_ACCL) || (ratio <= MIN_RATIO_ACCL)){
				result->accl[i] = TEST_FAIL;
				result->overall = TEST_FAIL;
			}
		}

		// aux test if factory data is not present
		else{
			fprintf(stderr, "WARNING no factory data present, falling back to aux test\n");
			if((accl_response[i] < ((MIN_ST_ACCEL_MG * accel_sensitivity_1g) / 1000)) \
			|| (accl_response[i] > ((MAX_ST_ACCEL_MG * accel_sensitivity_1g) / 1000))){
				result->accl[i] = TEST_FAIL;
				result->overall = TEST_FAIL;
			}
		}
	}

	return 0;
}

