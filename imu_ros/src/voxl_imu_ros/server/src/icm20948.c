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




