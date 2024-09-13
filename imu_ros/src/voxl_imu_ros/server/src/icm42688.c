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
#include <unistd.h> // for usleep
#include <string.h> // for memset
#include <math.h>

#include <modal_start_stop.h>
#include <voxl_io.h>
#include <rc_math/timestamp_filter.h>
#include "icm42688.h"
#include "icm42688_defs.h"
#include "global_debug_flags.h"

// enable hires fifo
#define HIRES_FIFO

// enable real time clock
#define EN_RTC

// we are locked into used 2000dps/16g FSRs with the HIRES FIFO mode!!
static const float gyro_scale_16 = (2000.0*DEG_TO_RAD)/32767.0;
static const float accl_scale_16 = (16.0*G_TO_MS2)/32767.0;
static const float gyro_scale_20 = (2000.0*DEG_TO_RAD)/524287.0;
static const float accl_scale_20 = (16.0*G_TO_MS2)/524287.0;

#define MAX_BUS 32
static int current_bank[MAX_BUS];
static double configured_odr_hz[MAX_BUS];

// ratio of imu clock speed to apps proc
// imu runs faster than requested due to 32.768khz clock instead of 32khz, start with this known correction factor, this value is estimated in realtime
#define READ_DELAY_NS			6300000 // tuned so qvio reports 0 timeshift
#define STARTING_CLOCK_RATIO	0.9765625
static rc_ts_filter_t f[16];


#define SPI_SPEED_BASE 1000000  // 1mbit, that's as slow as DSPAL will let us go
#define SPI_SPEED_DATA 5000000  // limit is 10mbit, go at half that



static int __switch_to_bank(int bus, uint8_t bank)
{
	if(bank>4){
		fprintf(stderr, "ERROR in __switch_to_bank, bank must be in 0 to 4\n");
		return -1;
	}
	if(voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_BANK_SEL, bank)){
		fprintf(stderr, "ERROR in __switch_to_bank, writing to SPI bus\n");
		return -1;
	}
	current_bank[bus] = bank;
	usleep(1000);
	return 0;
}


int icm42688_detect(int bus)
{
	uint8_t val;
	if(__switch_to_bank(bus, 0)){
		fprintf(stderr, "ERROR in icm42688_fifo_detect() switching bank\n");
		return -1;
	}
	if(voxl_spi_read_reg_byte(bus, ICM42688_UB0_REG_WHOAMI, &val)){
		fprintf(stderr, "ERROR in icm42688_fifo_detect() reading WHOAMI reg\n");
		return -1;
	}
	if(val == 0x47) return 0;

	// fprintf(stderr, "NOT ICM42688, WHOAMI returned %d, expected %d\n", val, 0x47);
	return -1;
}


int icm42688_init(int bus, double sample_rate_hz, int lp_cutoff_freq_hz)
{
	uint8_t val, ODR, UI_FILT_BW;
	/*
		// check sample rate option, pick nearest rate
		if     (sample_rate_hz>=24000){	ODR=1;	configured_odr_hz[bus]=32000;}
		else if(sample_rate_hz>=12000){	ODR=2;	configured_odr_hz[bus]=16000;}
		else if(sample_rate_hz>=6000){	ODR=3;	configured_odr_hz[bus]=8000;}
		else if(sample_rate_hz>=3000){	ODR=4;	configured_odr_hz[bus]=4000;}
		else if(sample_rate_hz>=1500){	ODR=5;	configured_odr_hz[bus]=2000;}
		else if(sample_rate_hz>=750){	ODR=6;	configured_odr_hz[bus]=1000;}
		else if(sample_rate_hz>=350){	ODR=15;	configured_odr_hz[bus]=500;}
		else if(sample_rate_hz>=150){	ODR=7;	configured_odr_hz[bus]=200;}
		else if(sample_rate_hz>=75){	ODR=8;	configured_odr_hz[bus]=100;}
		else if(sample_rate_hz>=37){	ODR=9;	configured_odr_hz[bus]=50;}
		else if(sample_rate_hz>=19){	ODR=10;	configured_odr_hz[bus]=25;}
		else{							ODR=11;	configured_odr_hz[bus]=12.5;}

		printf("using ODR=%6.2fhz which is the nearest the icm42688 can get to %4.1f\n",\
												configured_odr_hz[bus], sample_rate_hz);
	*/


	// for icm42688, cutoff frequency varies based on sample rate
	// round to nearest available BW setting
	// if     (lp_cutoff_freq_hz >= sample_rate_hz/1.5) UI_FILT_BW = 15;// filter off
	// else if(lp_cutoff_freq_hz >= sample_rate_hz/3)   UI_FILT_BW = 0; // nominally sample_rate/2
	// else if(lp_cutoff_freq_hz >= sample_rate_hz/4.5) UI_FILT_BW = 1; // nominally sample_rate/4
	// else if(lp_cutoff_freq_hz >= sample_rate_hz/6.5) UI_FILT_BW = 2; // nominally sample_rate/5
	// else if(lp_cutoff_freq_hz >= sample_rate_hz/9)   UI_FILT_BW = 3; // nominally sample_rate/8
	// else if(lp_cutoff_freq_hz >= sample_rate_hz/13)  UI_FILT_BW = 4; // nominally sample_rate/10
	// else if(lp_cutoff_freq_hz >= sample_rate_hz/18)  UI_FILT_BW = 5; // nominally sample_rate/16
	// else if(lp_cutoff_freq_hz >= sample_rate_hz/30)  UI_FILT_BW = 6; // nominally sample_rate/20
	// else                                             UI_FILT_BW = 7; // nominally sample_rate/40

	UI_FILT_BW = 14;//15
	////////////////////////////////////////////////////////////////////////////
	// start with bank 0 registers
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 0)) return -1;

	// reset the device, wait >1ms after this write
	val = SOFT_RESET_CONFIG;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_DEVICE_CONFIG, val);
	usleep(2000); // 2ms to be safe

	#ifdef EN_RTC

	// configure INTF_CONFIG1 for real time clock
	val = 0x95; // default plus enable RTC_MODE
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_INTF_CONFIG1, val);

	// now jump to back 1 for INTF_CONFIG5
	if(__switch_to_bank(bus, 1)) return -1;

	// configure INTF_CONFIG5 for real time clock
	val = 0x04; // set pin 9 function to CLKIN
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_INTF_CONFIG5, val);

	#endif



	////////////////////////////////////////////////////////////////////////////
	// now bank 1
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 1)) return -1;

	// AA filter and notch filter are on by default
	val = 0xA0; // AAF ON, Notch ON
	//val = 0xA1; // AAF ON, Notch OFF
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC2, val);


	// // 42hz strongest possible for testing only
	// static const uint8_t  AAF_DELT		= 1;
	// static const uint16_t AAF_DELTSQR	= 1;
	// static const uint8_t  AAF_BITSHIFT	= 15;

	// 126hz AAF
	// this is what we actually use for 1khz sample rate
	static const uint8_t  AAF_DELT		= 3;
	static const uint16_t AAF_DELTSQR	= 9;
	static const uint8_t  AAF_BITSHIFT	= 12;

	// // 213hz old value, for testing now
	// static const uint8_t  AAF_DELT		= 5;
	// static const uint16_t AAF_DELTSQR	= 25;
	// static const uint8_t  AAF_BITSHIFT	= 10;

	// write these AA filter settings to gyro in bank 1
	val = AAF_DELT;
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC3, val);
	val = (AAF_DELTSQR & 0xFF); // lower 8 bits of DELTSQR
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC4, val);
	val = (AAF_BITSHIFT<<4) | ((AAF_DELTSQR & 0xF00)>>8); // upper 4 bits of DELTSQR
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC5, val);
	usleep(1000);

	////////////////////////////////////////////////////////////////////////////
	// now bank 2
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 2)) return -1;

	// write same AA filter params to Accel. Note the 3 values above are the same
	// but the registers are constructed differently.
	val = AAF_DELT<<1;
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_ACCEL_CONFIG_STATIC2, val);
	val = (AAF_DELTSQR & 0xFF); // lower 8 bits of DELTSQR
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_ACCEL_CONFIG_STATIC3, val);
	val = (AAF_BITSHIFT<<4) | ((AAF_DELTSQR & 0xF00)>>8); // upper 4 bits of DELTSQR
	voxl_spi_write_reg_byte(bus, ICM42688_UB1_REG_ACCEL_CONFIG_STATIC4, val);
	usleep(1000);

	////////////////////////////////////////////////////////////////////////////
	// back to bank 0
	////////////////////////////////////////////////////////////////////////////
	if(__switch_to_bank(bus, 0)) return -1;

	// configure INTF_CONFIG0
	val =	FIFO_HOLD_LAST_DATA_EN	| FIFO_COUNT_REC_BYTES | \
			FIFO_COUNT_ENDIAN_BIG	| SENSOR_DATA_ENDIAN_BIG | \
			UI_SIFS_CFG_DISABLE_I2C;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_INTF_CONFIG0, val);

	// GYRO_CONFIG0 Gyro FSR fixed at 2000 dps for now, also set ODR
	val = GYRO_FS_SEL_2000_DPS | 0x07; // 200HZ
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_GYRO_CONFIG0, val);

	// GYRO_CONFIG1 set gyro UI filter order
	val = TEMP_FILT_BW_4000 | GYRO_UI_FILT_ORD_3 | GYRO_DEC2_M2_ORD_3;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_GYRO_CONFIG1, val);

	// ACCEL_CONFIG0 Accel FSR fixed at 16g for now, also set ODR
	val = ACCEL_FS_SEL_16_G | 0x07;//200HZ
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_ACCEL_CONFIG0 , val);

	// ACCEL_CONFIG1 set accel UI filter order
	val = ACCEL_UI_FILT_ORD_3 | ACCEL_DEC2_M2_ORD_3;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_ACCEL_CONFIG1, val);

	// timestamp config
	val  = TMST_DEFAULT;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_TMST_CONFIG, val);

	// set UI filter bandwidth the same for accel and gyro
	val = (UI_FILT_BW << 4) | (UI_FILT_BW << 0);
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_GYRO_ACCEL_CONFIG0, val);

	// fifo config 1, temp always enabled even if we turn off that bit
	val = FIFO_RESUME_PARTIAL_RD_EN | FIFO_GYRO_EN | FIFO_ACCEL_EN | FIFO_TEMP_EN;
	#ifdef HIRES_FIFO
		val |= FIFO_HIRES_EN;
	#endif
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_FIFO_CONFIG1, val);

	// LAST thing is to finally turn on the sensors AFTER all the registers have been configured
	// turn on temp, accel and gyro in low noise mode, requires >200us wait after
	val = TEMP_EN | GYRO_MODE_LOW_NOISE | ACCEL_MODE_LOW_NOISE;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_PWR_MGMT0, val);

	/*
	// read out notch filter register values for R&D only
	if(__switch_to_bank(bus, 1)) return -1;
	voxl_spi_read_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC6, &val);
	printf("X: %d\n", val);
	voxl_spi_read_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC7, &val);
	printf("Y: %d\n", val);
	voxl_spi_read_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC8, &val);
	printf("Z: %d\n", val);
	voxl_spi_read_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC9, &val);
	printf("%x\n", val);
	voxl_spi_read_reg_byte(bus, ICM42688_UB1_REG_GYRO_CONFIG_STATIC10, &val);
	printf("%x\n", val);
	*/

	// wait for imu to wakeup, otherwise we get -32768 out of the data regs
	usleep(100000);

	// set up the timestamp filter
	//rc_ts_filter_init(&f[bus], configured_odr_hz[bus]/STARTING_CLOCK_RATIO);

	return 0;
}


int icm42688_close(int bus)
{
	icm42688_fifo_stop(bus);
	return 0;
}


int icm42688_basic_read(int bus, imu_data_t* data)
{
	uint8_t status;
	uint8_t raw[14];
	int64_t time_read_start = voxl_apps_time_monotonic_ns();

	if (!voxl_spi_read_reg(bus, ICM42688_UB0_REG_INT_STATUS, &status, 1)){
		if(status & DATA_RDY_INT){
			// read all 14 bytes in one go -->ACTUALLY 12 BYTES NO TEMP DATA
			if(voxl_spi_read_reg(bus, ICM42688_UB0_REG_ACCEL_DATA_X1, raw, 14)){
				fprintf(stderr, "ERROR in icm42688_basic_read, failed to read register\n");
				return -1;
			}
			// construct 16-bit values
			int16_t ax16,ay16,az16,gx16,gy16,gz16;
			ax16 = (raw[0] << 8) | raw[1];
			ay16 = (raw[2] << 8) | raw[3];
			az16 = (raw[4] << 8) | raw[5];
			gx16 = (raw[6] << 8) | raw[7];
			gy16 = (raw[8] << 8) | raw[9];
			gz16 = (raw[10]<< 8) | raw[11];

			if( ax16 == -32768 || ay16 == -32768 || az16 == -32768 ||\
				gx16 == -32768 || gy16 == -32768 || gz16 == -32768){
				fprintf(stderr, "ERROR in %s, bad reading!\n", __FUNCTION__);
				return -1;
			}

			if(	ax16==0 && ay16==0 && az16==0 && \
				gx16==0 && gy16==0 && gz16==0){
				fprintf(stderr, "ERROR: in %s read all 0's\n", __FUNCTION__);
				return -1;
			}

			data->accl_ms2[0]  = ax16 * accl_scale_16;
			data->accl_ms2[1]  = ay16 * accl_scale_16;
			data->accl_ms2[2]  = az16 * accl_scale_16;
			data->gyro_rad[0]  = gx16 * gyro_scale_16;
			data->gyro_rad[1]  = gy16 * gyro_scale_16;
			data->gyro_rad[2]  = gz16 * gyro_scale_16;
			data->timestamp_ns = voxl_apps_time_monotonic_ns()-time_read_start;
			return 0;
		}
		else {
			//fprintf(stderr, "ERROR in %s, data not ready\n", __FUNCTION__);
			return -1;
		}
	}
	else {
		fprintf(stderr, "ERROR in %s, failed to read status register\n", __FUNCTION__);
		return -1;
	}
		
}


int icm42688_fifo_reset(int bus)
{
	// make sure we are in the right register bank
	if(current_bank[bus]!=0) __switch_to_bank(bus, 0);

	uint8_t val = FIFO_FLUSH;
	if(voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_SIGNAL_PATH_RESET, val)){
		fprintf(stderr, "ERROR in icm42688_fifo_reset() writing to SPI bus\n");
		return -1;
	}
	return 0;
}


int icm42688_fifo_start(int bus)
{
	// make sure we are in the right register bank
	if(current_bank[bus]!=0) __switch_to_bank(bus, 0);

	// fifo config 0
	uint8_t val = FIFO_MODE_STREAM;
	if(voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_FIFO_CONFIG, val)){
		fprintf(stderr, "ERROR in icm42688_fifo_start() writing to SPI bus\n");
		return -1;
	}
	icm42688_fifo_reset(bus);
	return 0;
}


int icm42688_fifo_stop(int bus)
{
	// make sure we are in the right register bank
	if(current_bank[bus]!=0) __switch_to_bank(bus, 0);

	// fifo config 0
	uint8_t val = FIFO_MODE_BYPASS;
	if(voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_FIFO_CONFIG, val)){
		fprintf(stderr, "ERROR in icm42688_fifo_stop() writing to SPI bus\n");
		return -1;
	}
	icm42688_fifo_reset(bus);
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

		if(voxl_spi_read_reg(bus, ICM42688_UB0_REG_ACCEL_DATA_X1, raw, 12)){
			fprintf(stderr, "ERROR in __self_test_sample, failed to read register\n");
			return -1;
		}

		// construct 16-bit values
		int16_t ax16,ay16,az16,gx16,gy16,gz16;
		ax16 = (raw[0] << 8) | raw[1];
		ay16 = (raw[2] << 8) | raw[3];
		az16 = (raw[4] << 8) | raw[5];
		gx16 = (raw[6] << 8) | raw[7];
		gy16 = (raw[8] << 8) | raw[9];
		gz16 = (raw[10]<< 8) | raw[11];

		if( ax16 == -32768 || ay16 == -32768 || az16 == -32768 ||\
			gx16 == -32768 || gy16 == -32768 || gz16== -32768){
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



int icm42688_self_test(int bus, imu_self_test_result_t* result)
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

	printf("Starting self-test on ICM42688 on SPI bus %d\n", bus);

	// indicate all failures so if we quit early it indicate a failure
	memset(result,1,sizeof(imu_self_test_result_t));

	if(__switch_to_bank(bus, 0)){
		fprintf(stderr, "self-test failed trying to change register bank\n");
		return -1;
	}

	// reset the device, wait >1ms after this write
	val = SOFT_RESET_CONFIG;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_DEVICE_CONFIG, val);
	usleep(2000); // 2ms to be safe

	// turn on temp, accel and gyro in low noise mode, requires >200us wait after
	val = TEMP_EN | GYRO_MODE_LOW_NOISE | ACCEL_MODE_LOW_NOISE;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_PWR_MGMT0, val);
	usleep(1000);

	// GYRO_CONFIG0 for self-test, use 250DPS and 1kz
	val = GYRO_FS_SEL_250_DPS | 5;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_GYRO_CONFIG0, val);

	// GYRO_CONFIG1 set gyro UI filter order
	val = TEMP_FILT_BW_5 | GYRO_UI_FILT_ORD_3 | GYRO_DEC2_M2_ORD_3;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_GYRO_CONFIG1, val);

	// ACCEL_CONFIG0 for self-test, use 4g and 1kz
	val = ACCEL_FS_SEL_4_G | 5;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_ACCEL_CONFIG0, val);

	// ACCEL_CONFIG1 set accel UI filter order
	val = ACCEL_UI_FILT_ORD_3 | ACCEL_DEC2_M2_ORD_3;
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_ACCEL_CONFIG1, val);

	// GYRO_ACCEL_CONFIG0 set UI filter bandwidth for self-test at 1/10 ODR
	val = (4 << 4) | (4 << 0);
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_GYRO_ACCEL_CONFIG0, val);

	// wait 100ms for outputs to settle after configuration
	usleep(100000);

	// sample data before enabling self test
	if(__self_test_sample(bus, gyro_avg_normal, accl_avg_normal)){
		fprintf(stderr, "self-test failed trying to read normal data\n");
		return -1;
	}

	// turn on self-test and wait 200ms for oscillation to stabilize
	if(voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_SELF_TEST_CONFIG, 0x7F)){
		fprintf(stderr, "self-test failed trying to turning on ST reg\n");
		return -1;
	}
	usleep(200000);

	// sample data with self-test enabled
	if(__self_test_sample(bus, gyro_avg_test_on, accl_avg_test_on)){
		fprintf(stderr, "self-test failed trying to read after turning on ST\n");
		return -1;
	}

	// turn off self-test
	voxl_spi_write_reg_byte(bus, ICM42688_UB0_REG_SELF_TEST_CONFIG, 0x00);

	// calculate deltas
	for(i=0; i<3; i++){
		gyro_response[i] = INV_ABS(gyro_avg_test_on[i] - gyro_avg_normal[i]);
		accl_response[i] = INV_ABS(accl_avg_test_on[i] - accl_avg_normal[i]);
	}

	// grab factory self-test data
	__switch_to_bank(bus,1);
	if(voxl_spi_read_reg(bus, ICM42688_UB1_REG_ICXG_ST_DATA, gyro_factory_data, 3)){
		fprintf(stderr, "self-test failed trying to read factory gyro results\n");
		return -1;
	}
	__switch_to_bank(bus,2);
	if(voxl_spi_read_reg(bus, ICM42688_UB1_REG_XA_ST_DATA, accl_factory_data, 3)){
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
		int fs_sel = 3; // 250DPS for self-test
		gyro_factory_response[i] = INV_ST_OTP_EQUATION(fs_sel, gyro_factory_data[i]);
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
	for(i=0;i<3;i++) {
		int fs_sel = 2; // 4G for self-test
		accl_factory_response[i] = INV_ST_OTP_EQUATION(fs_sel, accl_factory_data[i]);
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
