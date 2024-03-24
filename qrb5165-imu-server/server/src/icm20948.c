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
#include "icm20948.h"
#include "icm20948_defs.h"
#include "global_debug_flags.h"
#include "timestamp.h"

// we are locked into used 2000dps/16g FSRs with the HIRES FIFO mode!!
static const float gyro_scale_16 = (2000.0*DEG_TO_RAD)/32767.0;
static const float accl_scale_16 = (16.0*G_TO_MS2)/32767.0;

#define SPI_SPEED_BASE 1000000  // limit is 1mbit for regular registers, and that's as slow as DSPAL will let us go
#define SPI_SPEED_DATA 5000000  // limit is 7mbit, go less than that


static int current_bank[16];
static double configured_odr_hz[16];

// ratio of imu clock speed to apps proc
// starting point comes from register
static double clock_ratio[16];
// keep track of timestamp so we can pick up next read
static int last_read_was_good[16];
static int64_t last_ts_ns[16];


static int __switch_to_bank(int bus, uint8_t bank)
{
	if(bank>3){
		fprintf(stderr, "ERROR in __switch_to_bank, bank must be in 0 to 3\n");
		return -1;
	}
	if(voxl_spi_write_reg_byte(bus, REG_BANK_SEL, bank<<4)){
		fprintf(stderr, "ERROR in __switch_to_bank, writing to SPI bus\n");
		return -1;
	}
	current_bank[bus] = bank;
	usleep(1000);
	return 0;
}


int icm20948_detect(int bus)
{
	uint8_t val;
	if(__switch_to_bank(bus, 0)){
		fprintf(stderr, "ERROR in icm20948_fifo_detect() switching bank\n");
		return -1;
	}
	if(voxl_spi_read_reg_byte(bus, REG_WHO_AM_I, &val)){
		fprintf(stderr, "ERROR in icm20948_fifo_detect() reading WHOAMI reg\n");
		return -1;
	}
	if(val == 0xEA) return 0;

	return -1;
}


int icm20948_init(int bus, double sample_rate_hz, int lp_cutoff_freq_hz)
{
	uint8_t val, DIV, UI_FILT_BW;

	// find which output data rate divider most closely matches desired sample rate
	DIV=255; // start with slowest in case a faster match isn't found
	for(int i=0;i<=255;i++){
		// here i is the value that will go into GYRO_SMPLRT_DIV
		// ODR = 1125/(1+i)
		// middle is the sample rate halfway between i and i+1
		double middle = ((1125.0/(i+1))+(1125.0/(i+2)))/2.0;
		if(sample_rate_hz>middle){
			DIV=i;
			break;
		}
	}
	// save the real ODR as configured for later
	configured_odr_hz[bus] = 1125.0/(DIV+1);
	printf("using ODR=%6.2fhz which is the nearest the icm20948 can get to %4.1f\n",\
										configured_odr_hz[bus], sample_rate_hz);

	// round to nearest available filter setting
	if     (lp_cutoff_freq_hz >= 279) UI_FILT_BW = 7; // nominally 361 hz
	else if(lp_cutoff_freq_hz >= 174) UI_FILT_BW = 0; // nominally 197 hz
	else if(lp_cutoff_freq_hz >= 134) UI_FILT_BW = 1; // nominally 152 hz
	else if(lp_cutoff_freq_hz >= 85)  UI_FILT_BW = 2; // nominally 120 hz
	else if(lp_cutoff_freq_hz >= 38)  UI_FILT_BW = 3; // nominally 51 hz
	else if(lp_cutoff_freq_hz >= 18)  UI_FILT_BW = 4; // nominally 24 hz
	else if(lp_cutoff_freq_hz >= 9)   UI_FILT_BW = 5; // nominally 12 hz
	else                              UI_FILT_BW = 6; // nominally 6 hz


	////////////////////////////////////////////////////////////////////////////
	// start with bank 0 registers
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 0)) return -1;

	// reset the device, wait >1ms after this write
	voxl_spi_write_reg_byte(bus, REG_PWR_MGMT_1, BIT_H_RESET);
	usleep(100000); // 100ms fom tdk example code

	// select PLL clock
	voxl_spi_write_reg_byte(bus, REG_PWR_MGMT_1, BIT_CLK_PLL);

	// turn off low-power mode
	val = 0;
	voxl_spi_write_reg_byte(bus, REG_LP_CONFIG, val);
	usleep(1000);

	////////////////////////////////////////////////////////////////////////////
	// now bank 1
	////////////////////////////////////////////////////////////////////////////

	// clock error in measured at the factory and written to chip
	// read it out and use as a starting point for clock estimator
	if(clock_ratio[bus]==0.0){
		int8_t PLL;
		if(__switch_to_bank(bus, 1)) return -1;
		if(voxl_spi_read_reg_byte(bus, REG_TIMEBASE_CORRECTION_PLL, (uint8_t*)&PLL)){
			fprintf(stderr, "ERROR in icm20948_fifo_detect() reading WHOAMI reg\n");
			return -1;
		}
		clock_ratio[bus] = 1.0 - (PLL*0.00079); // CORRECT
	}

	////////////////////////////////////////////////////////////////////////////
	// now bank 2
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 2)) return -1;

	// set ODR
	val = DIV;
	voxl_spi_write_reg_byte(bus, REG_GYRO_SMPLRT_DIV, val);
	voxl_spi_write_reg_byte(bus, REG_ACCEL_SMPLRT_DIV_2, val);

	// set DLPF and FSR
	val = GYRO_FSR_CFG_2000 | GYRO_FCHOICE_EN_DLPF | (UI_FILT_BW<<3);
	voxl_spi_write_reg_byte(bus, REG_GYRO_CONFIG_1, val);
	val = ACCEL_FSR_CFG_16G | ACCEL_FCHOICE_EN_DLPF | (UI_FILT_BW<<3);
	voxl_spi_write_reg_byte(bus, REG_ACCEL_CONFIG, val);

	// set temp DLPF
	val = 5; // 17hz BW
	voxl_spi_write_reg_byte(bus, REG_TEMP_CONFIG, val);
	usleep(1000);


	////////////////////////////////////////////////////////////////////////////
	// back to bank 0
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 0)) return -1;

	// next step is for the user to enable the fifo
	usleep(10000);
	return 0;
}


int icm20948_close(int bus)
{
	icm20948_fifo_stop(bus);
	return 0;
}


int icm20948_basic_read(int bus, imu_data_t* data)
{
	uint8_t raw[14];
	int64_t time_read_start = voxl_apps_time_monotonic_ns();

	if(voxl_spi_read_reg(bus, REG_ACCEL_XOUT_H_SH, raw, 14)){
		fprintf(stderr, "ERROR in icm20948_basic_read, failed to read register\n");
		return -1;
	}

	// construct 16-bit values
	int16_t ax16,ay16,az16,gx16,gy16,gz16,tr16;
	ax16 = (raw[0] << 8) | raw[1];
	ay16 = (raw[2] << 8) | raw[3];
	az16 = (raw[4] << 8) | raw[5];
	gx16 = (raw[6] << 8) | raw[7];
	gy16 = (raw[8] << 8) | raw[9];
	gz16 = (raw[10]<< 8) | raw[11];
	tr16 = (raw[12]<< 8) | raw[13];

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


int icm20948_fifo_reset(int bus)
{
	uint8_t val;

	// make sure we are in the right register bank
	if(current_bank[bus]!=0) __switch_to_bank(bus, 0);

	// halt writes to fifo during reset
	val = 0;
	voxl_spi_write_reg_byte(bus, REG_USER_CTRL, val);
	val = 0;
	voxl_spi_write_reg_byte(bus, REG_FIFO_EN_2, val);

	// set reset flag and wait for it to take effect
	val = 0xe0;
	voxl_spi_write_reg_byte(bus, REG_FIFO_RST, val);
	usleep(1000);

	// turn on fifo writes again
	val = BIT_FIFO_EN;
	voxl_spi_write_reg_byte(bus, REG_USER_CTRL, val);
	val = BIT_ACCEL_FIFO_EN | BITS_GYRO_FIFO_EN | BIT_TEMP_FIFO_EN;
	voxl_spi_write_reg_byte(bus, REG_FIFO_EN_2, val);

	return 0;
}


int icm20948_fifo_start(int bus)
{
	icm20948_fifo_reset(bus);
	return 0;
}


int icm20948_fifo_stop(int bus)
{
	// make sure we are in the right register bank
	if(current_bank[bus]!=0) __switch_to_bank(bus, 0);

	// fifo config 0
	uint8_t val = 0;
	if(voxl_spi_write_reg_byte(bus, REG_FIFO_EN_2, val)){
		fprintf(stderr, "ERROR in icm20948_fifo_start() writing to SPI bus\n");
		return -1;
	}

	return 0;
}


int icm20948_fifo_read(int bus, imu_data_t* data, int* packets, uint8_t* fifo_buffer)
{
	static int n_empty_reads = 0;

	int records = 0;
	*packets = 0; // make sure we indicate nothing has been read if we return early

	const int p_len = 14; // accel plus gyro plus temp
	const int p_warn = 260;
	const int p_max = 292;

	// make sure we are in the right register bank
	if(current_bank[bus]!=0) __switch_to_bank(bus, 0);

	// read time both before and after the SPI transfer
	int64_t time_before_read = voxl_apps_time_monotonic_ns();

	// read as much as we can from the fifo into our own buffer
	int ret = voxl_spi_read_imu_fifo(bus, REG_FIFO_COUNT_H, REG_FIFO_R_W, p_len, \
						&records, fifo_buffer, p_max*p_len, \
						SPI_SPEED_BASE, SPI_SPEED_DATA);
	if(ret){
		last_read_was_good[bus]=0;
		fprintf(stderr, "ERROR icm20948_fifo_read: reader found an issue, resetting fifo\n");
		// usually this happens because the 20948 filled its fifo and put a partial
		// packet in it. At that point we can no longer trust the fifo, reset it.
		icm20948_fifo_reset(bus);
		return -1;
	}
	if(records==0){
		// should never pass mmore than a few empty reads in a row
		n_empty_reads++;
		if (n_empty_reads >= 10){
			fprintf(stderr, "ERROR icm20948_fifo_read: buffer was empty %d times in a row\n", n_empty_reads);
			return -1;
		}
		return 0;
	}
	if(records>p_max){
		fprintf(stderr, "ERROR icm20948_fifo_read: impossible value in FIFO_COUNT: %d\n",\
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

		// construct 16-bit values
		int16_t ax16,ay16,az16,gx16,gy16,gz16,tr16;
		ax16 = (base[0] << 8) | base[1];
		ay16 = (base[2] << 8) | base[3];
		az16 = (base[4] << 8) | base[5];
		gx16 = (base[6] << 8) | base[7];
		gy16 = (base[8] << 8) | base[9];
		gz16 = (base[10]<< 8) | base[11];
		tr16 = (base[12]<< 8) | base[13];

		if(ax16==0 && ay16==0 && az16==0 && gx16==0 && gy16==0 && gz16==0 && tr16==0){
			fprintf(stderr, "ERROR: in %s read all 0's\n", __FUNCTION__);
			return -1;
		}

		data[i].accl_ms2[0]  = ax16 * accl_scale_16;
		data[i].accl_ms2[1]  = ay16 * accl_scale_16;
		data[i].accl_ms2[2]  = az16 * accl_scale_16;
		data[i].gyro_rad[0]  = gx16 * gyro_scale_16;
		data[i].gyro_rad[1]  = gy16 * gyro_scale_16;
		data[i].gyro_rad[2]  = gz16 * gyro_scale_16;
		data[i].temp_c       = (tr16 / TEMP_SENSITIVITY) + 21.0f;

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
	int bad_readings = 10; // allow up to 10 bad readings

	for(i=0;i<samples;i++){
		uint8_t raw[12];

		if(voxl_spi_read_reg(bus, REG_ACCEL_XOUT_H_SH, raw, 12)){
			fprintf(stderr, "ERROR in __self_test_sample, failed to read register\n");
			return -1;
		}

		// construct 16-bit values
		int16_t ax16,ay16,az16,gx16,gy16,gz16,tr16;
		ax16 = (raw[0] << 8) | raw[1];
		ay16 = (raw[2] << 8) | raw[3];
		az16 = (raw[4] << 8) | raw[5];
		gx16 = (raw[6] << 8) | raw[7];
		gy16 = (raw[8] << 8) | raw[9];
		gz16 = (raw[10]<< 8) | raw[11];
		tr16 = (raw[12]<< 8) | raw[13];

		if(ax16==0 && ay16==0 && az16==0 && gx16==0 && gy16==0 && gz16==0 && tr16==0){
			if(bad_readings==0){
				fprintf(stderr, "ERROR in __self_test_sample, too many bad readings\n");
				return -1;
			}
			bad_readings--;
			i--;
			usleep(10000); // wait a bit longer in case IMU is still waking up
			continue;
		}

		gyro_sum[0]+=gx16;
		gyro_sum[1]+=gy16;
		gyro_sum[2]+=gz16;
		accl_sum[0]+=ax16;
		accl_sum[1]+=ay16;
		accl_sum[2]+=az16;
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



int icm20948_self_test(int bus, imu_self_test_result_t* result)
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

	printf("Starting self-test on ICM20948 on SPI bus %d\n", bus);

	// indicate all failures so if we quit early it indicate a failure
	memset(result,1,sizeof(imu_self_test_result_t));

	if(__switch_to_bank(bus, 0)){
		fprintf(stderr, "self-test failed trying to change register bank\n");
		return -1;
	}

	////////////////////////////////////////////////////////////////////////////
	// start with bank 0 registers
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 0)) return -1;

	// reset the device, wait >1ms after this write
	voxl_spi_write_reg_byte(bus, REG_PWR_MGMT_1, BIT_H_RESET);
	usleep(100000); // 100ms fom tdk example code

	// select PLL clock
	voxl_spi_write_reg_byte(bus, REG_PWR_MGMT_1, BIT_CLK_PLL);

	// turn off low-power mode
	val = 0;
	voxl_spi_write_reg_byte(bus, REG_LP_CONFIG, val);


	////////////////////////////////////////////////////////////////////////////
	// now bank 2
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 2)) return -1;

	// set ODR
	voxl_spi_write_reg_byte(bus, REG_GYRO_SMPLRT_DIV, 0);
	voxl_spi_write_reg_byte(bus, REG_ACCEL_SMPLRT_DIV_2, 0);

	// set DLPF and FSR
	voxl_spi_write_reg_byte(bus, REG_GYRO_CONFIG_1, SELFTEST_GYRO_FS);
	voxl_spi_write_reg_byte(bus, REG_ACCEL_CONFIG, SELFTEST_ACCEL_FS);


	////////////////////////////////////////////////////////////////////////////
	// back to bank 0
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 0)) return -1;

	// wait 100ms for outputs to settle after configuration
	usleep(100000);

	// sample data before enabling self test
	if(__self_test_sample(bus, gyro_avg_normal, accl_avg_normal)){
		fprintf(stderr, "self-test failed trying to read normal data\n");
		return -1;
	}

	// turn on self-test and wait 200ms for oscillation to stabilize
	__switch_to_bank(bus, 2);
	voxl_spi_write_reg_byte(bus, REG_GYRO_CONFIG_2, GYRO_SELF_TEST_EN);
	voxl_spi_write_reg_byte(bus, REG_ACCEL_CONFIG_2, ACCEL_SELF_TEST_EN);
	__switch_to_bank(bus, 0);
	usleep(200000);

	// sample data with self-test enabled
	if(__self_test_sample(bus, gyro_avg_test_on, accl_avg_test_on)){
		fprintf(stderr, "self-test failed trying to read after turning on ST\n");
		return -1;
	}

	// turn off self-test
	__switch_to_bank(bus, 2);
	voxl_spi_write_reg_byte(bus, REG_GYRO_CONFIG_2, 0);
	voxl_spi_write_reg_byte(bus, REG_ACCEL_CONFIG_2, 0);
	__switch_to_bank(bus, 0);

	// calculate deltas
	for(i=0; i<3; i++){
		gyro_response[i] = INV_ABS(gyro_avg_test_on[i] - gyro_avg_normal[i]);
		accl_response[i] = INV_ABS(accl_avg_test_on[i] - accl_avg_normal[i]);
	}

	// grab factory self-test data
	__switch_to_bank(bus,1);
	if(voxl_spi_read_reg(bus, REG_SELF_TEST_X_GYRO, gyro_factory_data, 3)){
		fprintf(stderr, "self-test failed trying to read factory gyro results\n");
		return -1;
	}
	if(voxl_spi_read_reg(bus, REG_SELF_TEST_X_ACCEL, accl_factory_data, 3)){
		fprintf(stderr, "self-test failed trying to read factory accel results\n");
		return -1;
	}
	__switch_to_bank(bus,0);

	// zero out results indicating all pass before checking for failures
	memset(result,0,sizeof(imu_self_test_result_t));

	////////////////////////////////////////////////////////////////////////////
	// Gyro calculation
	////////////////////////////////////////////////////////////////////////////

	// calculate ST results OTP based on the equation
	for(i=0;i<3;i++){
		gyro_factory_response[i] = sSelfTestEquation[gyro_factory_data[i]-1];
	}

	// Check Gyro self-test results
	uint32_t gyro_sensitivity_1dps = 32768 / 250;
	for(i=0; i<3; i++){

		// normal test if factory results are present in registers
		if(gyro_factory_data[i]){
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
			if((gyro_response[i] < MIN_ST_GYRO_RESPONSE) || \
				(gyro_response[i] > MIN_ST_GYRO_RESPONSE)){
				result->gyro[i] = TEST_FAIL;
				result->overall = TEST_FAIL;
			}
		}
	}


	////////////////////////////////////////////////////////////////////////////
	// Accel calculation
	////////////////////////////////////////////////////////////////////////////

	// calculate ST results OTP based on the equation
	for(i=0;i<3;i++){
		accl_factory_response[i] = sSelfTestEquation[accl_factory_data[i]-1];
	}

	// Check Accel self-test result
	for(i=0;i<3;i++){
		// normal test if factory results are present in registers
		if(accl_factory_data[i]){
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
			if((accl_response[i] < MIN_ST_ACCEL_RESPONSE) \
			|| (accl_response[i] > MAX_ST_ACCEL_RESPONSE)){
				result->accl[i] = TEST_FAIL;
				result->overall = TEST_FAIL;
			}
		}
	}

	return 0;
}
