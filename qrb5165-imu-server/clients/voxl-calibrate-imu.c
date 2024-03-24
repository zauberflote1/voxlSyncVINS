/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
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
#include <getopt.h>
#include <unistd.h>	// for usleep()
#include <string.h>
#include <stdlib.h>	// for system()
#include <math.h>	// for fabs()

#include <rc_math.h>
#include <modal_start_stop.h>
#include <modal_pipe_client.h>
#include <voxl_imu_server.h>
#include "cal_file.h"


#define MAX_SAMPLES			1000

#define GYRO_MAX_NOISE		0.05
#define ACCL_MAX_NOISE		0.25

// flags indicating if one or both IMUs are actively being calibrated
static int en[N_IMUS];

// 0: not sampling, otherwise sampling
static int sampling_active;
static volatile int n_samples[N_IMUS];
static imu_data_t samples[N_IMUS][MAX_SAMPLES];

// keep track of sensors average during sampling
static double gyro_avg[N_IMUS][3];
static double accl_avg[N_IMUS][3];
static double temp_avg[N_IMUS][7]; // 7 orientations, take temp at each


static int _quit(int ret)
{
	main_running = 0;
	sampling_active = 0;

	// turn off all the data callbacks
	for(int i=0; i<N_IMUS; i++){
		if(en[i]) pipe_client_set_simple_helper_cb(i, NULL, NULL);
	}

	// turn off calibration mode, only need to send once
	for(int i=0; i<N_IMUS; i++){
		if(en[i]){
			pipe_client_send_control_cmd(i, COMMAND_STOP_CAL);
			break;
		}
	}

	pipe_client_close_all();

	// turn fan back on if voxl-fan is available
	if(access("/usr/bin/voxl-fan", F_OK )==0){
		system("/usr/bin/voxl-fan on");
	}

	exit(ret);
	return 0;
}

/**
 * This is a blocking function which returns 0 if the user presses ENTER.
 * If ctrl-C is pressed it will quit the program
 */
static int continue_or_quit(void)
{
	// set stdin to non-canonical raw mode to capture all button presses
	fflush(stdin);
	if(system("stty raw")!=0){
		fprintf(stderr,"ERROR in continue_or_quit setting stty raw\n");
		return -1;
	}

	int ret;

	while(1){
		int c = getchar();
		// break if we read ctrl-c
		if(c==3 || c==24 || c==26){
			ret=-1;
			break;
		}
		if(c=='\r' || c=='\n'){
			ret = 0;
			break;
		}
	}

	fflush(stdin);

	// put stdin back to normal canonical mode
	if(system("stty cooked")!=0){
		fprintf(stderr,"ERROR in continue_or_quit setting stty cooked\n");
		return -1;
	}
	if(ret) _quit(ret);

	printf("\n");
	return ret;
}


static void _data_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	// reading from a pipe will return 0 if the other end (server) closes
	// check for this (and negative  numbers indicating error) and quit.
	// your program doesn't need to quit, you can handle the server quitting
	// however you like to suite your needs. We just quit for this example.
	if(bytes<=0){
		fprintf(stderr, "Server Disconnected from channel %d\n", ch);
		fprintf(stderr, "quitting\n");
		pipe_client_set_simple_helper_cb(ch, NULL, NULL);
		main_running = 0;
		return;
	}

	// if there was an error OR no packets received, just return;
	int n_packets = 0;
	imu_data_t* data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);
	if(data_array == NULL) return;
	if(n_packets<=0) return;

	// copy new data into the growing list of samples. quit once full.
	for(int i=0;i<n_packets;i++){
		samples[ch][n_samples[ch]] = data_array[i];
		n_samples[ch]++;
		if(n_samples[ch]>=MAX_SAMPLES){
			pipe_client_set_simple_helper_cb(ch, NULL, NULL);
			return;
		}
	}

	return;
}


// 7 stages, 0-6. 0 is gyro, then remaining 6 for accel
static int _collect_samples(int stage)
{
	int i,j,k;
	for(i=0;i<N_IMUS;i++) n_samples[i]=0;

	// set the callback so new data comes in
	// callback will be turned off by the callback itself when data is full
	for(i=0;i<N_IMUS;i++){
		if(en[i]) pipe_client_set_simple_helper_cb(i, _data_cb, NULL);
	}

	// wait for sampling thread to finish
	sampling_active = 1;
	while(sampling_active){
		if(!main_running) _quit(-1);
		int needs_more=0;
		for(i=0;i<N_IMUS;i++){
			if(en[i] && n_samples[i]<MAX_SAMPLES) needs_more=1;
		}
		if(!needs_more) sampling_active = 0;
		else usleep(100000);
	}

	// check if IMU data is too noisy
	RC_VECTOR_ON_STACK(v,MAX_SAMPLES);
	for(i=0;i<N_IMUS;i++){
		// skip unused imus
		if(!en[i]) continue;
		// check all 3 gyro axis
		for(j=0;j<3;j++){
			// copy data into vector
			for(k=0;k<MAX_SAMPLES;k++) v.d[k]=(double)samples[i][k].gyro_rad[j];
			double dev = rc_vector_std_dev(v);
			//fprintf(stderr, "gyro noise: %f, limit is %f\n", dev, GYRO_MAX_NOISE);
			if(dev>GYRO_MAX_NOISE){
				fprintf(stderr, "Too much gyro noise for imu %d! read %f, limit is %f\n", i, dev, GYRO_MAX_NOISE);
				return -1;
			}
		}
		// check all 3 accl axis
		for(j=0;j<3;j++){
			// copy data into vector
			for(k=0;k<MAX_SAMPLES;k++) v.d[k]=(double)samples[i][k].accl_ms2[j];
			double dev = rc_vector_std_dev(v);
			//fprintf(stderr, "accl noise: %f, limit is %f\n", dev, ACCL_MAX_NOISE);
			if(dev>ACCL_MAX_NOISE){
				fprintf(stderr, "Too much accl noise for imu %d! read %f, limit is %f\n", i, dev, ACCL_MAX_NOISE);
				return -1;
			}
		}
	}

	// calculate the averages
	for(i=0;i<N_IMUS;i++){
		// skip unused imus
		if(!en[i]) continue;
		// calculate for each axis
		for(j=0;j<3;j++){
			// gyro
			gyro_avg[i][j]=0.0;
			for(k=0;k<MAX_SAMPLES;k++) gyro_avg[i][j]+=(double)samples[i][k].gyro_rad[j];
			gyro_avg[i][j]/=MAX_SAMPLES;
			// accl
			accl_avg[i][j]=0.0;
			for(k=0;k<MAX_SAMPLES;k++) accl_avg[i][j]+=(double)samples[i][k].accl_ms2[j];
			accl_avg[i][j]/=MAX_SAMPLES;
		}
		// temp
		temp_avg[i][stage]=0.0;
		for(k=0;k<MAX_SAMPLES;k++) temp_avg[i][stage]+=(double)samples[i][k].temp_c;
		temp_avg[i][stage]/=MAX_SAMPLES;

	}

	return 0;
}


int main()
{
	int i,j,k,done;

	// we won't necessarily be calibrating all imus, so load the current
	// calibration before proceeding.
	if(cal_file_read()){
		fprintf(stderr, "failed to read calibration file\n");
		return -1;
	}

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;

	// request a new pipe from the server and assign callbacks
	// we may be calibrating up to 4 imus at once!
	int flags = CLIENT_FLAG_EN_SIMPLE_HELPER | CLIENT_FLAG_DISABLE_AUTO_RECONNECT;
	en[0] = !pipe_client_open(0, VOXL_IMU0_LOCATION, "cal", flags, IMU_RECOMMENDED_READ_BUF_SIZE);
	en[1] = !pipe_client_open(1, VOXL_IMU1_LOCATION, "cal", flags, IMU_RECOMMENDED_READ_BUF_SIZE);
	en[2] = !pipe_client_open(2, VOXL_IMU2_LOCATION, "cal", flags, IMU_RECOMMENDED_READ_BUF_SIZE);
	en[3] = !pipe_client_open(3, VOXL_IMU3_LOCATION, "cal", flags, IMU_RECOMMENDED_READ_BUF_SIZE);

	// make sure at least one imu is running
	int n_enabled = 0;
	for(i=0;i<N_IMUS;i++){
		if(en[i]){
			n_enabled++;
			printf("Server has IMU%d enabled and available for calibration\n", i);
		}
	}
	if(n_enabled<1){
		fprintf(stderr, "ERROR: Neither IMU pipe appear to be open\n");
		fprintf(stderr, "Make sure the voxl-imu-server service is running and try again\n");
		return -1;
	}

	// put voxl-imu-server into calibration mode, only need to send to one pipe
	for(i=0;i<N_IMUS;i++){
		if(en[i]){
			pipe_client_send_control_cmd(i, COMMAND_START_CAL);
			break;
		}
	}

	// turn off fan if voxl-fan is available
	if(access("/usr/bin/voxl-fan", F_OK )==0){
		printf("detected voxl-fan utility\n");
		printf("turning off fan to reduce imu noise\n");
		system("/usr/bin/voxl-fan off");
	}

	////////////////////////////////////////////////////////////////////////////
	// calculate gyro offsets
	////////////////////////////////////////////////////////////////////////////
	printf("\n");
	printf("We're about to start calibrating the gyro, make sure VOXL is\n");
	printf("COMPLETELY STILL and press ENTER to start.\n");
	continue_or_quit();
	// try to collect samples until imu output is below noise threshold
	done = 0;
	while(!done){
		if(!main_running) _quit(0);
		if(_collect_samples(0)==0) done = 1;
		else{
			printf("Data too noisy. Make sure VOXL is COMPLETELY STILL\n");
			printf("Disconnect or temporarily stop the fan if connected\n");
			printf("press ENTER to try again, Press Ctrl-C to quit.\n");
			continue_or_quit();
		}
	}
	for(i=0;i<N_IMUS;i++){
		if(!en[i]) continue;
		printf("imu%d gyro offset (rad/s):", i);
		for(j=0;j<3;j++){
			gyro_offset[i][j] = gyro_avg[i][j];
			printf(" %7.4f", (double)gyro_offset[i][j]);
		}
		has_baseline_temp[i]=1;
		gyro_baseline_temp[i]=temp_avg[i][0];
		printf("\nimu%d gyro baseline temp (C): %7.2f", i, (double)gyro_baseline_temp[i]);
		printf("\n");
	}

	////////////////////////////////////////////////////////////////////////////
	// Now accelerometer
	////////////////////////////////////////////////////////////////////////////

	// matrix for ellipse fitting
	rc_matrix_t A[N_IMUS];
	rc_vector_t center = RC_VECTOR_INITIALIZER;
	rc_vector_t lengths = RC_VECTOR_INITIALIZER;
	for(i=0;i<N_IMUS;i++){
		A[i] = rc_matrix_empty();
		rc_matrix_alloc(&A[i],6,3);
	}

	printf("\n");
	printf("We're about to start calibrating the accelerometer.\n");
	printf("This process will require you to orient the VOXL board in six roughly\n");
	printf("orthogonal orientations. Ideally this should be with the six sides of\n");
	printf("the board facing down. Try to do this without touching the board.\n");
	printf("\n");
	for(i=0;i<6;i++){
		printf("\n");
		printf("When you are ready to collect data for orientation %d of 6,\n", i+1);
		printf("press ENTER to start or Ctrl-C to quit\n");
		continue_or_quit();
		done = 0;
		while(!done){
			if(!main_running) _quit(0);
			if(_collect_samples(i+1)==0) done = 1;
			else{
				printf("Data too noisy. Make sure VOXL is COMPLETELY STILL\n");
				printf("Disconnect or temporarily stop the fan if connected\n");
				printf("press ENTER to try again, Press Ctrl-C to quit.\n");
				continue_or_quit();
			}
		}
		for(k=0;k<N_IMUS;k++){
			if(!en[k]) continue;
			for(j=0;j<3;j++) A[k].d[i][j] = accl_avg[k][j];
			printf("accl%d temp was %7.2fC for that sample\n", k, temp_avg[k][i+1]);
		}
		printf("\n");
	}


	// calculate the accelerometer characteristics
	printf("RESULTS:\n");
	for(i=0;i<N_IMUS;i++){
		// skip disabled IMUs
		if(!en[i]) continue;
		// fit elipse to 6 vectors
		if(rc_algebra_fit_ellipsoid(A[i],&center,&lengths)){
			fprintf(stderr,"ERROR failed to calculate characteristics for IMU%d accelerometer\n", i);
			_quit(-1);
		}
		// copy results to cal arrays and print results
		printf("accl%d offsets (m/s2): ", i);
		for(j=0;j<3;j++){
			accl_offset[i][j] = center.d[j];
			printf(" %7.4f", (double)accl_offset[i][j]);
		}
		printf("\n");
		printf("accl%d scales        : ", i);
		for(j=0;j<3;j++){
			// each axis should be 1G long, so divide lengths by 1G to get scale
			accl_scale[i][j] = lengths.d[j]/G_TO_MS2;
			printf(" %7.4f", (double)accl_scale[i][j]);
		}
		printf("\n");
		// calc average accel temp through the 6 orientations
		accl_baseline_temp[i] = 0.0;
		for(j=1;j<=6;j++) accl_baseline_temp[i] += (float)temp_avg[i][j];
		accl_baseline_temp[i] /= 6;
		printf("accl%d baseline temp: %7.2fC", i, (double)accl_baseline_temp[i]);
		printf("\n");
	}
	printf("\n");

	// do some validation
	int validation_passed = 1;
	for(i=0;i<N_IMUS;i++){
		// skip disabled IMUs
		if(!en[i]) continue;

		for(j=0;j<3;j++){
			// check wonky calculation
			if(isnan(accl_offset[i][j]) || isnan(accl_scale[i][j])){
				fprintf(stderr,"ERROR data fitting produced NaN for accelerometer %d\n", i);
				fprintf(stderr,"most likely the unit was held in incorrect orientation during data collection\n");
				_quit(-1);
			}
			// check for wonky accel bias
			if(fabs(accl_offset[i][j])>3.0){
				fprintf(stderr,"ERROR accelerometer bias out of bounds for IMU%d\n", i);
				fprintf(stderr,"accelerometer bias should be <3.0 m/s^2\n");
				validation_passed = 0;
			}
			// check wonky scale
			if(accl_scale[i][j]>1.1f || accl_scale[i][j]<0.9f){
				fprintf(stderr,"ERROR accelerometer scale out of bounds for IMU%d\n", i);
				fprintf(stderr,"accelerometer scale should be between 0.9 and 1.1\n");
				validation_passed = 0;
			}
		}
	}

	if(!validation_passed){
		printf("\n");
		printf("Errors were encountered while calibrating. Based on the data\n");
		printf("and messages printed above, you can decide to accept and use\n");
		printf("this calibration anyway. If one IMUs is obviously bad then we\n");
		printf("suggest disabling it in /etc/modalai/voxl-imu-server.conf\n");
		printf("\n");
		printf("Press ENTER to proceed to use this calibration anyway.\n");
		printf("Press Ctrl-C to discard this calibration and quit\n");
		continue_or_quit();
	}

	// write out data
	if(cal_file_write()<0) _quit(-1);
	printf("Successfully wrote calibration to disk\n");
	printf("voxl-imu-server should reload the new calibration automatically.\n");
	printf("You can now run voxl-inspect-imu to check that the new data looks good\n");
	printf("You can also optionally run voxl-calibrate-imu-temp to run a more\n");
	printf("involved temperature calibration.\n");

	// successfully quit
	_quit(0);
	return 0;
}
