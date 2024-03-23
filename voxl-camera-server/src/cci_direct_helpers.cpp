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

#ifndef APQ8096

#include <stdio.h>
#include <stdint.h>

#include <cci_direct.h>
#include "cci_direct_helpers.h"



static int _ov7251_tweaks(int cam_id)
{
	printf("setting ov7251 BLC register for cam id %d\n", cam_id);
	int ret = 0;

	uint16_t slave_addr = voxl_cci_get_slave_addr(cam_id);
	if((int16_t)slave_addr == -1){
		fprintf(stderr, "failed to fetch slave address for cam_id #%d\n", cam_id);
		return -1;
	}

	uint8_t val = 0x02; // force BLC to run every frame, this prevents random brightness shifts
	ret |= voxl_cci_write(cam_id, slave_addr, 0x4005, CCI_16BIT, &val, CCI_8BIT);

	return ret;
}


int cci_direct_apply_register_tweaks(int cam_id, enum sensor_t sensor)
{
	voxl_cci_init();

	switch(sensor){

		case SENSOR_OV7251:
			return _ov7251_tweaks(cam_id);
			break;

		default:
			return 0;
	}

	return 0;
}


// same registers for both!
static int _ov7251_ov9782_rotate(int cam_id, int en_rotate)
{
	int ret = 0;
	uint8_t val;

	uint16_t slave_addr = voxl_cci_get_slave_addr(cam_id);
	if((int16_t)slave_addr == -1){
		fprintf(stderr, "failed to fetch slave address for cam_id #%d\n", cam_id);
		return -1;
	}

	// normal, upright
	if(!en_rotate){
		val = 0;
		ret |= voxl_cci_write(cam_id, slave_addr, 0x3820, CCI_16BIT, &val, CCI_8BIT);
		ret |= voxl_cci_write(cam_id, slave_addr, 0x3821, CCI_16BIT, &val, CCI_8BIT);
	}
	else{
		val = 4;
		ret |= voxl_cci_write(cam_id, slave_addr, 0x3820, CCI_16BIT, &val, CCI_8BIT);
		ret |= voxl_cci_write(cam_id, slave_addr, 0x3821, CCI_16BIT, &val, CCI_8BIT);
	}

	return ret;
}



static int _ar0144_rotate(int cam_id, int en_rotate)
{
	int ret = 0;
	uint8_t val;

	uint16_t slave_addr = voxl_cci_get_slave_addr(cam_id);
	if((int16_t)slave_addr == -1){
		fprintf(stderr, "failed to fetch slave address for cam_id #%d\n", cam_id);
		return -1;
	}

	// normal, upright
	if(!en_rotate){
		val = 0;
		ret |= voxl_cci_write(cam_id, slave_addr, 0x301D, CCI_16BIT, &val, CCI_8BIT);
	}
	else{
		val = 3;
		ret |= voxl_cci_write(cam_id, slave_addr, 0x301D, CCI_16BIT, &val, CCI_8BIT);
	}

	return ret;
}



int cci_direct_set_rotation(int cam_id, sensor_t sensor, int en_rotate)
{
	voxl_cci_init();

	switch(sensor){

		case SENSOR_OV9782:
		case SENSOR_OV7251:
			return _ov7251_ov9782_rotate(cam_id, en_rotate);
			break;

		case SENSOR_AR0144:
			return _ar0144_rotate(cam_id, en_rotate);
			break;

		default:
			return 0;
	}

	return 0;
}


static int _ov9782_set_wb_registers(int cam_id)
{
	int ret = 0;

	// for 9782 WITH IR filter
	// uint16_t reg_red = 0x3400;
	// uint16_t val_red = 0x0800; // doubled from default 0x0400
	// uint16_t reg_grn = 0x3402;
	// uint16_t val_grn = 0x0400; // OV default
	// uint16_t reg_blu = 0x3404;
	// uint16_t val_blu = 0x0800; // doubled from default 0x0400


	// for M0113 with no IR filter
	uint16_t reg_red = 0x3400;
	uint16_t val_red = 0x0580; // default 0x0400
	uint16_t reg_grn = 0x3402;
	uint16_t val_grn = 0x0400; // OV default
	uint16_t reg_blu = 0x3404;
	uint16_t val_blu = 0x0900; // default 0x0400

	// multiplier to crank up overall gain closer to the limit
	float mult = 1.5;
	val_red *= mult;
	val_grn *= mult;
	val_blu *= mult;

	uint16_t slave_addr = voxl_cci_get_slave_addr(cam_id);
	if((int16_t)slave_addr == -1){
		fprintf(stderr, "failed to fetch slave address for cam_id #%d\n", cam_id);
		return -1;
	}

	ret |= voxl_cci_write(cam_id, slave_addr, reg_red, CCI_16BIT, (uint8_t*)&val_red, CCI_16BIT);
	ret |= voxl_cci_write(cam_id, slave_addr, reg_grn, CCI_16BIT, (uint8_t*)&val_grn, CCI_16BIT);
	ret |= voxl_cci_write(cam_id, slave_addr, reg_blu, CCI_16BIT, (uint8_t*)&val_blu, CCI_16BIT);

	return ret;
}


int cci_direct_set_white_balance(int cam_id, enum sensor_t sensor)
{
	// currently this is the only Color CV camera we have
	if(sensor == SENSOR_OV9782){
		return _ov9782_set_wb_registers(cam_id);
	}
	return 0;
}


#endif // APQ8096
