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
#include <string.h>
#include <stdlib.h> // for malloc free
#include <unistd.h> // for access()

#include <modal_json.h>
#include "cal_file.h"


////////////////////////////////////////////////////////////////////////////////
// define all the "Extern" variables from cal_file.h
// these all match the defaults in the cal file
////////////////////////////////////////////////////////////////////////////////

static float default_offset[3] = {0.0f, 0.0f, 0.0f};
static float default_scale[3]  = {1.0f, 1.0f, 1.0f};

static float default_gyro_drift[3] = {0.0f, 0.0f, 0.0f};
static float default_accl_drift[3] = {0.0f, 0.0f, 0.0f};


int has_static_cal;

float gyro_offset[N_IMUS][3];
float accl_offset[N_IMUS][3];
float accl_scale[N_IMUS][3];

int has_baseline_temp[N_IMUS];
float gyro_baseline_temp[N_IMUS];
float accl_baseline_temp[N_IMUS];

int has_temp_cal[N_IMUS];
float gyro_drift_coeff[N_IMUS][3][3];
float accl_drift_coeff[N_IMUS][3][3];

float corrected_gyro_drift_coeff[N_IMUS][3][3];
float corrected_accl_drift_coeff[N_IMUS][3][3];


////////////////////////////////////////////////////////////////////////////////
// Don't forget to add print statements for each value too!
////////////////////////////////////////////////////////////////////////////////
int cal_file_print(void)
{
	printf("=================================================================\n");
	for(int i=0;i<N_IMUS;i++){
		printf("calibration for IMU%d:\n",i);
		printf("Gyro Offsets (rad/s): X: %7.3f Y: %7.3f Z: %7.3f\n", (double)gyro_offset[i][0], (double)gyro_offset[i][1], (double)gyro_offset[i][2]);
		printf("Accl Offsets (m/s^2): X: %7.3f Y: %7.3f Z: %7.3f\n", (double)accl_offset[i][0], (double)accl_offset[i][1], (double)accl_offset[i][2]);
		printf("Accl Scale          : X: %7.3f Y: %7.3f Z: %7.3f\n", (double)accl_scale[i][0],  (double)accl_scale[i][1],  (double)accl_scale[i][2]);
		printf("\n");
		printf("has_baseline_temp:      %d\n", has_baseline_temp[i]);
		printf("gyro_baseline_temp (C): %7.3f\n", (double)gyro_baseline_temp[i]);
		printf("accl_baseline_temp (C): %7.3f\n", (double)accl_baseline_temp[i]);
		printf("\n");
		printf("has_temp_cal:          %d\n", has_temp_cal[i]);
		printf("gyro_drift_coeff: X: %10.6f %10.6f %10.6f\n", (double)gyro_drift_coeff[i][0][0], (double)gyro_drift_coeff[i][0][1], (double)gyro_drift_coeff[i][0][2]);
		printf("gyro_drift_coeff: Y: %10.6f %10.6f %10.6f\n", (double)gyro_drift_coeff[i][1][0], (double)gyro_drift_coeff[i][1][1], (double)gyro_drift_coeff[i][1][2]);
		printf("gyro_drift_coeff: Z: %10.6f %10.6f %10.6f\n", (double)gyro_drift_coeff[i][2][0], (double)gyro_drift_coeff[i][2][1], (double)gyro_drift_coeff[i][2][2]);
		printf("accl_drift_coeff: X: %10.6f %10.6f %10.6f\n", (double)accl_drift_coeff[i][0][0], (double)accl_drift_coeff[i][0][1], (double)accl_drift_coeff[i][0][2]);
		printf("accl_drift_coeff: Y: %10.6f %10.6f %10.6f\n", (double)accl_drift_coeff[i][1][0], (double)accl_drift_coeff[i][1][1], (double)accl_drift_coeff[i][1][2]);
		printf("accl_drift_coeff: Z: %10.6f %10.6f %10.6f\n", (double)accl_drift_coeff[i][2][0], (double)accl_drift_coeff[i][2][1], (double)accl_drift_coeff[i][2][2]);
		printf("\n");
	}
	printf("\r=================================================================\n");
	return 0;
}


/**
 * used by the server to read cal file out from disk
 */
int cal_file_read(void)
{
	// assume we are calibrated until indicaged otherwise
	has_static_cal = 1;
	cJSON* parent = NULL;

	if(access(CALIBRATION_FILE, F_OK) != 0){
		printf("voxl-imu-server currently has no calibration file\n");
		has_static_cal = 0;

	}
	else{
		// check for old empty file from an old install and wipe it if there
		int ret = system("grep -q -e \"gyro0_offset 0.0 0.0\" /data/modalai/voxl-imu-server.cal");
		if(ret==0){
			fprintf(stderr, "removing old empty file\n");
			remove(CALIBRATION_FILE);
			has_static_cal = 0;
		}
		else{
			// something made the file unparsable, user should make a new calibration file
			parent = json_read_file(CALIBRATION_FILE);
			if(parent==NULL){
				fprintf(stderr, "\nERROR: Malformed calibration file: %s\n", CALIBRATION_FILE);
				fprintf(stderr, "Please make a new calibration file with voxl-calibrate-imu\n\n");
				parent = cJSON_CreateObject();
				has_static_cal = 0;
			}
			else{
				// check that we didn't just read in an empty file
				cJSON* tmp = cJSON_GetObjectItem(parent, "gyro0_offset");
				if(tmp==NULL){
					fprintf(stderr, "missing gyro0_offset, removing old empty file\n");
					remove(CALIBRATION_FILE);
					has_static_cal = 0;
				}
			}
		}
	}

	if(!has_static_cal){
		parent = cJSON_CreateObject();
	}


	// now actually load everything and place defaults where cal file was missing things
	// note, unlike a config file we don't write this back to disk!!
	json_fetch_fixed_vector_float_with_default(	parent, "gyro0_offset",			&gyro_offset[0][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl0_offset",			&accl_offset[0][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl0_scale",			&accl_scale[0][0],  3, default_scale);
	json_fetch_bool_with_default(				parent, "has_baseline_temp0",	&has_baseline_temp[0],	0);
	json_fetch_float_with_default(				parent, "gyro_baseline_temp0",	&gyro_baseline_temp[0],	30.0f);
	json_fetch_float_with_default(				parent, "accl_baseline_temp0",	&accl_baseline_temp[0],	30.0f);
	json_fetch_bool_with_default(				parent, "has_temp_cal0",		&has_temp_cal[0],		0);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_x_0",	&gyro_drift_coeff[0][0][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_y_0",	&gyro_drift_coeff[0][1][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_z_0",	&gyro_drift_coeff[0][2][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_x_0",	&accl_drift_coeff[0][0][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_y_0",	&accl_drift_coeff[0][1][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_z_0",	&accl_drift_coeff[0][2][0], 3, default_accl_drift);


	json_fetch_fixed_vector_float_with_default(	parent, "gyro1_offset",			&gyro_offset[1][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl1_offset",			&accl_offset[1][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl1_scale",			&accl_scale[1][0],  3, default_scale);
	json_fetch_bool_with_default(				parent, "has_baseline_temp1",	&has_baseline_temp[1],	0);
	json_fetch_float_with_default(				parent, "gyro_baseline_temp1",	&gyro_baseline_temp[1],	30.0f);
	json_fetch_float_with_default(				parent, "accl_baseline_temp1",	&accl_baseline_temp[1],	30.0f);
	json_fetch_bool_with_default(				parent, "has_temp_cal1",		&has_temp_cal[1],		0);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_x_1",	&gyro_drift_coeff[1][0][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_y_1",	&gyro_drift_coeff[1][1][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_z_1",	&gyro_drift_coeff[1][2][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_x_1",	&accl_drift_coeff[1][0][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_y_1",	&accl_drift_coeff[1][1][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_z_1",	&accl_drift_coeff[1][2][0], 3, default_accl_drift);


	json_fetch_fixed_vector_float_with_default(	parent, "gyro2_offset",			&gyro_offset[2][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl2_offset",			&accl_offset[2][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl2_scale",			&accl_scale[2][0],  3, default_scale);
	json_fetch_bool_with_default(				parent, "has_baseline_temp2",	&has_baseline_temp[2],	0);
	json_fetch_float_with_default(				parent, "gyro_baseline_temp2",	&gyro_baseline_temp[2],	30.0f);
	json_fetch_float_with_default(				parent, "accl_baseline_temp2",	&accl_baseline_temp[2],	30.0f);
	json_fetch_bool_with_default(				parent, "has_temp_cal2",		&has_temp_cal[2],		0);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_x_2",	&gyro_drift_coeff[2][0][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_y_2",	&gyro_drift_coeff[2][1][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_z_2",	&gyro_drift_coeff[2][2][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_x_2",	&accl_drift_coeff[2][0][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_y_2",	&accl_drift_coeff[2][1][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_z_2",	&accl_drift_coeff[2][2][0], 3, default_accl_drift);


	json_fetch_fixed_vector_float_with_default(	parent, "gyro3_offset",			&gyro_offset[3][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl3_offset",			&accl_offset[3][0], 3, default_offset);
	json_fetch_fixed_vector_float_with_default(	parent, "accl3_scale",			&accl_scale[3][0],  3, default_scale);
	json_fetch_bool_with_default(				parent, "has_baseline_temp3",	&has_baseline_temp[3],	0);
	json_fetch_float_with_default(				parent, "gyro_baseline_temp3",	&gyro_baseline_temp[3],	30.0f);
	json_fetch_float_with_default(				parent, "accl_baseline_temp3",	&accl_baseline_temp[3],	30.0f);
	json_fetch_bool_with_default(				parent, "has_temp_cal3",		&has_temp_cal[3],		0);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_x_3",	&gyro_drift_coeff[3][0][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_y_3",	&gyro_drift_coeff[3][1][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "gyro_drift_coeff_z_3",	&gyro_drift_coeff[3][2][0], 3, default_gyro_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_x_3",	&accl_drift_coeff[3][0][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_y_3",	&accl_drift_coeff[3][1][0], 3, default_accl_drift);
	json_fetch_fixed_vector_float_with_default(	parent, "accl_drift_coeff_z_3",	&accl_drift_coeff[3][2][0], 3, default_accl_drift);


	// calculate the corrected temp drift coefficients
	for(int i=0;i<N_IMUS;i++){
		// skip imus without temp cal
		if(!has_baseline_temp[i] || !has_temp_cal[i]) continue;

		float bg = gyro_baseline_temp[i];
		float ba = accl_baseline_temp[i];

		for(int j=0;j<3;j++){
			// correct the first coefficient so that the temp offset is 0 at the baseline temperature
			corrected_gyro_drift_coeff[i][j][0] = -((gyro_drift_coeff[i][j][1]*bg) + (gyro_drift_coeff[i][j][2]*bg*bg));
			corrected_gyro_drift_coeff[i][j][1] = gyro_drift_coeff[i][j][1];
			corrected_gyro_drift_coeff[i][j][2] = gyro_drift_coeff[i][j][2];
			corrected_accl_drift_coeff[i][j][0] = -((accl_drift_coeff[i][j][1]*ba) + (accl_drift_coeff[i][j][2]*ba*ba));
			corrected_accl_drift_coeff[i][j][1] = accl_drift_coeff[i][j][1];
			corrected_accl_drift_coeff[i][j][2] = accl_drift_coeff[i][j][2];
		}
	}
	// don't bother writing back to disk, if something was missing just use the
	// defaults. A new calibration will write in new values
	cJSON_Delete(parent);
	return 0;
}


/**
 * @brief      used by the calibrator to write new data to disk
 *
 * @return     0 on success, -1 on failure
 */
int cal_file_write(void)
{
	// new json object to construct from raw data
	cJSON *parent = cJSON_CreateObject();

	// construct the json object
	cJSON_AddItemToObject(parent,	"gyro0_offset",	cJSON_CreateFloatArray(gyro_offset[0], 3));
	cJSON_AddItemToObject(parent,	"accl0_offset",	cJSON_CreateFloatArray(accl_offset[0], 3));
	cJSON_AddItemToObject(parent,	"accl0_scale",	cJSON_CreateFloatArray(accl_scale[0],  3));
	cJSON_AddBoolToObject(parent,	"has_baseline_temp0",	has_baseline_temp[0]);
	cJSON_AddNumberToObject(parent,	"gyro_baseline_temp0",	gyro_baseline_temp[0]);
	cJSON_AddNumberToObject(parent,	"accl_baseline_temp0",	accl_baseline_temp[0]);
	cJSON_AddBoolToObject(parent,	"has_temp_cal0",		has_temp_cal[0]);
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_x_0",	cJSON_CreateFloatArray(gyro_drift_coeff[0][0], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_y_0",	cJSON_CreateFloatArray(gyro_drift_coeff[0][1], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_z_0",	cJSON_CreateFloatArray(gyro_drift_coeff[0][2], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_x_0",	cJSON_CreateFloatArray(accl_drift_coeff[0][0], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_y_0",	cJSON_CreateFloatArray(accl_drift_coeff[0][1], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_z_0",	cJSON_CreateFloatArray(accl_drift_coeff[0][2], 3));


	cJSON_AddItemToObject(parent,	"gyro1_offset",	cJSON_CreateFloatArray(gyro_offset[1], 3));
	cJSON_AddItemToObject(parent,	"accl1_offset",	cJSON_CreateFloatArray(accl_offset[1], 3));
	cJSON_AddItemToObject(parent,	"accl1_scale",	cJSON_CreateFloatArray(accl_scale[1],  3));
	cJSON_AddBoolToObject(parent,	"has_baseline_temp1",	has_baseline_temp[1]);
	cJSON_AddNumberToObject(parent,	"gyro_baseline_temp1",	gyro_baseline_temp[1]);
	cJSON_AddNumberToObject(parent,	"accl_baseline_temp1",	accl_baseline_temp[1]);
	cJSON_AddBoolToObject(parent,	"has_temp_cal1",		has_temp_cal[1]);
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_x_1",	cJSON_CreateFloatArray(gyro_drift_coeff[1][0], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_y_1",	cJSON_CreateFloatArray(gyro_drift_coeff[1][1], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_z_1",	cJSON_CreateFloatArray(gyro_drift_coeff[1][2], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_x_1",	cJSON_CreateFloatArray(accl_drift_coeff[1][0], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_y_1",	cJSON_CreateFloatArray(accl_drift_coeff[1][1], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_z_1",	cJSON_CreateFloatArray(accl_drift_coeff[1][2], 3));


	cJSON_AddItemToObject(parent,	"gyro2_offset",	cJSON_CreateFloatArray(gyro_offset[2], 3));
	cJSON_AddItemToObject(parent,	"accl2_offset",	cJSON_CreateFloatArray(accl_offset[2], 3));
	cJSON_AddItemToObject(parent,	"accl2_scale",	cJSON_CreateFloatArray(accl_scale[2],  3));
	cJSON_AddBoolToObject(parent,	"has_baseline_temp2",	has_baseline_temp[2]);
	cJSON_AddNumberToObject(parent,	"gyro_baseline_temp2",	gyro_baseline_temp[2]);
	cJSON_AddNumberToObject(parent,	"accl_baseline_temp2",	accl_baseline_temp[2]);
	cJSON_AddBoolToObject(parent,	"has_temp_cal2",		has_temp_cal[2]);
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_x_2",	cJSON_CreateFloatArray(gyro_drift_coeff[2][0], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_y_2",	cJSON_CreateFloatArray(gyro_drift_coeff[2][1], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_z_2",	cJSON_CreateFloatArray(gyro_drift_coeff[2][2], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_x_2",	cJSON_CreateFloatArray(accl_drift_coeff[2][0], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_y_2",	cJSON_CreateFloatArray(accl_drift_coeff[2][1], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_z_2",	cJSON_CreateFloatArray(accl_drift_coeff[2][2], 3));


	cJSON_AddItemToObject(parent,	"gyro3_offset",	cJSON_CreateFloatArray(gyro_offset[3], 3));
	cJSON_AddItemToObject(parent,	"accl3_offset",	cJSON_CreateFloatArray(accl_offset[3], 3));
	cJSON_AddItemToObject(parent,	"accl3_scale",	cJSON_CreateFloatArray(accl_scale[3],  3));
	cJSON_AddBoolToObject(parent,	"has_baseline_temp3",	has_baseline_temp[3]);
	cJSON_AddNumberToObject(parent,	"gyro_baseline_temp3",	gyro_baseline_temp[3]);
	cJSON_AddNumberToObject(parent,	"accl_baseline_temp3",	accl_baseline_temp[3]);
	cJSON_AddBoolToObject(parent,	"has_temp_cal3",		has_temp_cal[3]);
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_x_3",	cJSON_CreateFloatArray(gyro_drift_coeff[3][0], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_y_3",	cJSON_CreateFloatArray(gyro_drift_coeff[3][1], 3));
	cJSON_AddItemToObject(parent,	"gyro_drift_coeff_z_3",	cJSON_CreateFloatArray(gyro_drift_coeff[3][2], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_x_3",	cJSON_CreateFloatArray(accl_drift_coeff[3][0], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_y_3",	cJSON_CreateFloatArray(accl_drift_coeff[3][1], 3));
	cJSON_AddItemToObject(parent,	"accl_drift_coeff_z_3",	cJSON_CreateFloatArray(accl_drift_coeff[3][2], 3));


	if(json_write_to_file(CALIBRATION_FILE, parent)){
		fprintf(stderr, "ERROR failed to write calibration file %s to disk\n", CALIBRATION_FILE);
		return -1;
	}

	return 0;
}