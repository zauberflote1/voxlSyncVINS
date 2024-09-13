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
#include <string.h>
#include <stdlib.h> // for malloc free
#include <unistd.h> // for access()

#include <modal_json.h>
#include "config_file.h"


// define all the "Extern" variables from config_file.h and their defaults
int		imu_enable[N_IMUS]					= {1,		0,		0,		0};
int		bus[N_IMUS]							= {3,		1,		14,		5};
double	imu_sample_rate_hz[N_IMUS]			= {1000,	1000,	1000,	1000};
double	imu_lp_cutoff_freq_hz[N_IMUS]		= {92,		92,		92,		92};
int		imu_rotate_common_frame[N_IMUS]		= {1,		0,		0,		0};
double	imu_fifo_poll_rate_hz[N_IMUS]		= {100,	    100,	100,	100};


////////////////////////////////////////////////////////////////////////////////
// Don't forget to add print statements for each value too!
////////////////////////////////////////////////////////////////////////////////
int config_file_print(void)
{
	printf("=================================================================\n");
	printf("imu_apps_enable:                 %d\n",		imu_enable[0]);
	printf("imu_apps_bus:                    %d\n",		bus[0]);
	printf("imu_apps_sample_rate_hz:         %4.1f\n",	imu_sample_rate_hz[0]);
	printf("imu_apps_lp_cutoff_freq_hz:      %4.1f\n",	imu_lp_cutoff_freq_hz[0]);
	printf("imu_apps_rotate_common_frame:    %d\n",		imu_rotate_common_frame[0]);
	printf("imu_apps_fifo_poll_rate_hz:      %4.1f\n",	imu_fifo_poll_rate_hz[0]);
	printf("\n");
	printf("aux_imu1_enable:                 %d\n",		imu_enable[1]);
	printf("aux_imu1_bus:                    %d\n",		bus[1]);
	printf("aux_imu1_sample_rate_hz:         %4.1f\n",	imu_sample_rate_hz[1]);
	printf("aux_imu1_lp_cutoff_freq_hz:      %4.1f\n",	imu_lp_cutoff_freq_hz[1]);
	printf("aux_imu1_fifo_poll_rate_hz:      %4.1f\n",	imu_fifo_poll_rate_hz[1]);
	printf("\n");
	printf("aux_imu2_enable:                 %d\n",		imu_enable[2]);
	printf("aux_imu2_spi_bus:                %d\n",		bus[2]);
	printf("aux_imu2_sample_rate_hz:         %4.1f\n",	imu_sample_rate_hz[2]);
	printf("aux_imu2_lp_cutoff_freq_hz:      %4.1f\n",	imu_lp_cutoff_freq_hz[2]);
	printf("aux_imu2_fifo_poll_rate_hz:      %4.1f\n",	imu_fifo_poll_rate_hz[2]);
	printf("\n");
	printf("aux_imu3_enable:                 %d\n",		imu_enable[3]);
	printf("aux_imu3_spi_bus:                %d\n",		bus[3]);
	printf("aux_imu3_sample_rate_hz:         %4.1f\n",	imu_sample_rate_hz[3]);
	printf("aux_imu3_lp_cutoff_freq_hz:      %4.1f\n",	imu_lp_cutoff_freq_hz[3]);
	printf("aux_imu3_fifo_poll_rate_hz:      %4.1f\n",	imu_fifo_poll_rate_hz[3]);
	printf("=================================================================\n");
	return 0;
}



int config_file_read(void)
{
	int ret = json_make_empty_file_if_missing(VOXL_IMU_SERVER_CONF_FILE);
	if(ret < 0) return -1;
	else if(ret>0) fprintf(stderr, "Creating new config file: %s\n", VOXL_IMU_SERVER_CONF_FILE);

	cJSON* parent = json_read_file(VOXL_IMU_SERVER_CONF_FILE);
	if(parent==NULL) return -1;

	// don't read in bus from imu 0, this is hard-wired on qrb platforms
	json_fetch_bool_with_default(	parent, "imu0_enable",				&imu_enable[0],					imu_enable[0]);
	json_fetch_double_with_default(	parent, "imu0_sample_rate_hz",		&imu_sample_rate_hz[0],			imu_sample_rate_hz[0]);
	json_fetch_double_with_default(	parent, "imu0_lp_cutoff_freq_hz",	&imu_lp_cutoff_freq_hz[0],		imu_lp_cutoff_freq_hz[0]);
	json_fetch_bool_with_default(	parent, "imu0_rotate_common_frame",	&imu_rotate_common_frame[0],	imu_rotate_common_frame[0]);
	json_fetch_double_with_default(	parent, "imu0_fifo_poll_rate_hz",	&imu_fifo_poll_rate_hz[0],		imu_fifo_poll_rate_hz[0]);

	
	json_fetch_bool_with_default(	parent, "aux_imu1_enable",				&imu_enable[1],					imu_enable[1]);
	json_fetch_int_with_default(	parent, "aux_imu1_bus",					&bus[1],						bus[1]);
	json_fetch_double_with_default(	parent, "aux_imu1_sample_rate_hz",		&imu_sample_rate_hz[1],			imu_sample_rate_hz[1]);
	json_fetch_double_with_default(	parent, "aux_imu1_lp_cutoff_freq_hz",	&imu_lp_cutoff_freq_hz[1],		imu_lp_cutoff_freq_hz[1]);
	json_fetch_double_with_default(	parent, "aux_imu1_fifo_poll_rate_hz",	&imu_fifo_poll_rate_hz[1],		imu_fifo_poll_rate_hz[1]);

	// read in bus for aux channels since user needs to set those up
	// don't read in rotate_common_frame since the user's imu could be in any
	// orientation and this does not apply.
	json_fetch_bool_with_default(	parent, "aux_imu2_enable",				&imu_enable[2],					imu_enable[2]);
	json_fetch_int_with_default(	parent, "aux_imu2_spi_bus",				&bus[2],						bus[2]);
	json_fetch_double_with_default(	parent, "aux_imu2_sample_rate_hz",		&imu_sample_rate_hz[2],			imu_sample_rate_hz[2]);
	json_fetch_double_with_default(	parent, "aux_imu2_lp_cutoff_freq_hz",	&imu_lp_cutoff_freq_hz[2],		imu_lp_cutoff_freq_hz[2]);
	json_fetch_double_with_default(	parent, "aux_imu2_fifo_poll_rate_hz",	&imu_fifo_poll_rate_hz[2],		imu_fifo_poll_rate_hz[2]);

	json_fetch_bool_with_default(	parent, "aux_imu3_enable",				&imu_enable[3],					imu_enable[3]);
	json_fetch_int_with_default(	parent, "aux_imu3_spi_bus",				&bus[3],						bus[3]);
	json_fetch_double_with_default(	parent, "aux_imu3_sample_rate_hz",		&imu_sample_rate_hz[3],			imu_sample_rate_hz[3]);
	json_fetch_double_with_default(	parent, "aux_imu3_lp_cutoff_freq_hz",	&imu_lp_cutoff_freq_hz[3],		imu_lp_cutoff_freq_hz[3]);
	json_fetch_double_with_default(	parent, "aux_imu3_fifo_poll_rate_hz",	&imu_fifo_poll_rate_hz[3],		imu_fifo_poll_rate_hz[3]);

	if(json_get_parse_error_flag()){
		fprintf(stderr, "failed to parse config file\n");
		cJSON_Delete(parent);
		return -1;
	}

	// write modified data to disk if neccessary
	if(json_get_modified_flag()){
		// printf("The config file was modified during parsing, saving the changes to disk\n");
		json_write_to_file(VOXL_IMU_SERVER_CONF_FILE, parent);
	}
	cJSON_Delete(parent);
	return 0;
}

