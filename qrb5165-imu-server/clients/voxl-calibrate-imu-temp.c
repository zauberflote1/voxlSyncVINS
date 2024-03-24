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
#include <pthread.h>
#include <signal.h>

#include <rc_math.h>
#include <modal_start_stop.h>
#include <modal_pipe_client.h>
#include <cpu_monitor_interface.h>
#include <voxl_imu_server.h>
#include "cal_file.h"
#include "regression.h"


#define MAX_SAMPLES				2000
#define WAIT_BEFORE_SAMPLE_US	250000	// wait 1/4 second after hitting enter to sample
#define T_DELTA					6.5		// temperature difference between auto samples
#define GYRO_MAX_NOISE			0.05	// make sure IMU stays at least this still
#define ACCL_MAX_NOISE			0.25	// make sure IMU stays at least this still

// max limit for IMU temp. This is partly for safety, and partly because it's
// hard to get them any hotter without an enclosure.
#define T_SAFETY				75
#define TIMEOUT_NS				(8LL*60LL*1000000000LL) // 8 minutes, voxlcam takes a long time to warm up
#define CLIENT_NAME				"voxl-calibrate-imu-temp"


#define RESET_COLOR         "\e[39m"
#define COLOR_BLK           "\e[30m"
#define COLOR_RED           "\e[31m"
#define COLOR_GRN           "\e[32m"
#define COLOR_YLW           "\e[33m"
#define COLOR_BLU           "\e[34m"
#define COLOR_MAG           "\e[35m"
#define COLOR_CYN           "\e[36m"
#define COLOR_LIT_GRY       "\e[37m"

// flags indicating if one or both IMUs are actively being calibrated
static int en[N_IMUS];

static int en_debug = 0; // debug mode prints out extra data

static int sampling_active; // flag for sampling of all imus
static int is_sampling[N_IMUS]; // individual flag for each imu
static volatile int n_samples[N_IMUS];
static imu_data_t samples[N_IMUS][MAX_SAMPLES];

// sensor average during sampling
static double gyro_avg[N_IMUS][3];
static double accl_avg[N_IMUS][3];
static double temp_avg[N_IMUS];

// saved samples
static int samples_collected = 0;
#define N_AUTO_SAMPLES 7
static double samples_gyro_avg[N_IMUS][N_AUTO_SAMPLES][3];
static double samples_accl_avg[N_IMUS][N_AUTO_SAMPLES][3];
static double samples_temp_avg[N_IMUS][N_AUTO_SAMPLES];

// stress thread stuff
#define N_STRESS_THREADS	8
static int stress_running = 0;
static pthread_t stress_thread[N_STRESS_THREADS];

// print thread stuff
static pthread_t print_thread;
static int print_running;
static int print_thread_started;
static float current_temp[N_IMUS];
static float current_cpu_temp = -1000;


static void _print_usage(void)
{
	printf("\n\
Tool to run an automated temperature drift calibration for VOXL IMUs.\n\
This is not a substitude for voxl-calibrate-imu, you just run that as well.\n\
This temperature calibration is optional but helpful in some conditions.\n\
\n\
-d, --debug                 print extra debug data\n\
-h, --help                  print this help message\n\
\n");
	return;
}

static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"debug",			no_argument,		0,	'd'},
		{"help",			no_argument,		0,	'h'},
		{0, 0, 0, 0}
	};

	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "dh", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;
		case 'd':
			en_debug = 1;
			break;
		case 'h':
			_print_usage();
			exit(0);
			_print_usage();
			exit(0);
		}
	}
	return 0;
}


static int64_t _time_monotonic_ns()
{
	struct timespec ts;
	if(clock_gettime(CLOCK_MONOTONIC, &ts)){
		fprintf(stderr,"ERROR calling clock_gettime\n");
		return -1;
	}
	return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}

static void* _stress_function(__attribute__((unused)) void* arg)
{

	while(stress_running){
		double s = sqrt(rand());
		s++;
	}
	//printf("stress thread exiting\n");
	return NULL;
}


// thread to print current CPU and IMU temps at 2hz
static void* _print_function(__attribute__((unused)) void* arg)
{
	while(main_running){
		if(print_running){
			printf("\r");
			if(current_cpu_temp > -1000){
				printf("CPU: %4.1fC   ", (double)current_cpu_temp);
			}
			for(int i=0; i<N_IMUS; i++){
				if(en[i]) printf("IMU%d: %4.1fC   ", i, (double)current_temp[i]);
			}
			fflush(stdout);
		}
		usleep(500000);
	}
	print_running = 0;
	return NULL;
}


static int _start_stress()
{
	stress_running = 1;
	for(int i=0; i<N_STRESS_THREADS; i++){
		pthread_create(&stress_thread[i], NULL, _stress_function, NULL);
	}
	return 0;
}


static int _stop_stress()
{
	//printf("stopping stress\n");
	if(stress_running){
		stress_running = 0;
		for(int i=0; i<N_STRESS_THREADS; i++){
			pthread_join(stress_thread[i], NULL);
		}
	}
	return 0;
}


static int _quit(int ret)
{
	main_running = 0;
	sampling_active = 0;

	// turn off all the data callbacks
	for(int i=0; i<N_IMUS; i++){
		if(en[i]){
			pipe_client_set_simple_helper_cb(i, NULL, NULL);
		}
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
		system("/usr/bin/voxl-fan auto");
	}

	_stop_stress();

	if(print_thread_started){
		//printf("joining print thread\n");
		pthread_join(print_thread, NULL);
	}

	// print a newline because many of our previous prints use \r
	printf("\n");

	exit(ret);
	return 0;
}



/**
 * This is a blocking function which returns 0 if the user presses ENTER.
 * If ctrl-C is pressed it will quit the program
 */
static int continue_or_quit(void)
{
	//print_running = 1;

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

// called whenever the simple helper has data for us to process
static void _cpu_cb(__attribute__((unused))int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
	int n_packets;
	cpu_stats_t *data_array = modal_cpu_validate_pipe_data(data, bytes, &n_packets);
	if(data_array == NULL) return;
	current_cpu_temp = data_array[0].cpu_t_max;
	return;
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

	// keep a local copy of the current temperature
	current_temp[ch] = data_array[0].temp_c;

	// nothing else to do if not sampling
	if(!is_sampling[ch]) return;

	// copy new data into the growing list of samples. quit once full.
	for(int i=0;i<n_packets;i++){
		samples[ch][n_samples[ch]] = data_array[i];
		n_samples[ch]++;
		if(n_samples[ch]>=MAX_SAMPLES){
			is_sampling[ch] = 0;
			return;
		}
	}

	return;
}



static int _collect_samples()
{
	int i,j,k;
	for(i=0;i<N_IMUS;i++) n_samples[i]=0;

	// set the callback so new data comes in
	// is_sampling will be turned off by the callback itself when data is full
	for(i=0;i<N_IMUS;i++){
		if(en[i]) is_sampling[i] = 1;
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
				fprintf(stderr, "\nToo much gyro noise for imu %d! read %f, limit is %f\n", i, dev, GYRO_MAX_NOISE);
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
				fprintf(stderr, "\nToo much accl noise for imu %d! read %f, limit is %f\n", i, dev, ACCL_MAX_NOISE);
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
		temp_avg[i]=0.0;
		for(k=0;k<MAX_SAMPLES;k++) temp_avg[i]+=(double)samples[i][k].temp_c;
		temp_avg[i]/=MAX_SAMPLES;

	}

	return 0;
}


int main(int argc, char* argv[])
{
	// check for options
	if(_parse_opts(argc, argv)) return -1;

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
	en[0] = !pipe_client_open(0, VOXL_IMU0_LOCATION, CLIENT_NAME, flags, IMU_RECOMMENDED_READ_BUF_SIZE);
	en[1] = !pipe_client_open(1, VOXL_IMU1_LOCATION, CLIENT_NAME, flags, IMU_RECOMMENDED_READ_BUF_SIZE);
	en[2] = !pipe_client_open(2, VOXL_IMU2_LOCATION, CLIENT_NAME, flags, IMU_RECOMMENDED_READ_BUF_SIZE);
	en[3] = !pipe_client_open(3, VOXL_IMU3_LOCATION, CLIENT_NAME, flags, IMU_RECOMMENDED_READ_BUF_SIZE);

	// make sure at least one imu is running
	int n_enabled = 0;
	for(i=0;i<N_IMUS;i++){
		if(en[i]){
			n_enabled++;
			pipe_client_set_simple_helper_cb(i, _data_cb, NULL);
			printf("Server has IMU%d enabled and available for temperature calibration\n", i);
		}
	}
	if(n_enabled<1){
		fprintf(stderr, "ERROR: Neither IMU pipe appear to be open\n");
		fprintf(stderr, "Make sure the voxl-imu-server service is running and try again\n");
		return -1;
	}

	// subscribe to CPU data if it's available
	pipe_client_set_simple_helper_cb(4, _cpu_cb, NULL);
	pipe_client_open(4, "cpu_monitor", CLIENT_NAME, CLIENT_FLAG_EN_SIMPLE_HELPER, CPU_STATS_RECOMMENDED_READ_BUF_SIZE);

	// put voxl-imu-server into calibration mode, only need to send to one pipe
	for(i=0;i<N_IMUS;i++){
		if(en[i]){
			pipe_client_send_control_cmd(i, COMMAND_START_CAL);
			break;
		}
	}

	// turn on fan if voxl-fan is available
	if(access("/usr/bin/voxl-fan", F_OK )==0){
		//printf("turning on fan to cool board\n");
		system("/usr/bin/voxl-fan on");
	}
	else{
		fprintf(stderr, "WARNING, voxl-fan utility missing, can't turn on fan\n");
	}



	////////////////////////////////////////////////////////////////////////////
	// Start COLD stage
	////////////////////////////////////////////////////////////////////////////
	printf("\n");
	printf(COLOR_GRN "============================================================================\n" RESET_COLOR);
	printf("We're now starting the " COLOR_BLU "COLD" RESET_COLOR " stage of the calibration.\n");
	printf("Please stop all unnecessary services in another terminal, and try your best\n");
	printf("to cool the PCB with a fan. Alternatively, start this process on a cold PCB.\n");
	printf("\n");
	printf("We will be measuring the static gyro and accel offset as the board heats up.\n");
	printf("VOXL must remain completely still through this entire process!\n");
	printf("The IMU temperatures will be printed here continuously.\n");
	printf("\n");
	printf("Press ENTER to start the heating process when you are happy that the IMUs\n");
	printf("are " COLOR_BLU "cold" RESET_COLOR " enough for the lower bound. 25-35C is a reasonable goal for this.\n");
	printf(COLOR_GRN "============================================================================\n" RESET_COLOR);

	// start thread printing temperatures
	print_thread_started = 1;
	pthread_create(&print_thread, NULL, _print_function, NULL);

	// start printing while we wait for user input
	print_running = 1;
	continue_or_quit();
	print_running = 0;

	// turn off fan to reduce noise if voxl-fan is available
	if(access("/usr/bin/voxl-fan", F_OK )==0){
		//printf("\nturning off fan\n");
		system("/usr/bin/voxl-fan off");
	}
	else{
		fprintf(stderr, "WARNING, voxl-fan utility missing, can't turn off fan\n");
	}

	usleep(WAIT_BEFORE_SAMPLE_US);

	// try to collect samples until imu output is below noise threshold
	done = 0;
	while(!done){
		if(!main_running) _quit(0);
		if(_collect_samples()==0){
			done = 1;
		}
		else{
			printf("Data too noisy. Make sure VOXL is COMPLETELY STILL\n");
			printf("press ENTER to try again, Press Ctrl-C to quit.\n");
			print_running = 1;
			continue_or_quit();
			usleep(WAIT_BEFORE_SAMPLE_US);
			print_running = 0;
		}
	}
	print_running = 0;
	for(i=0;i<N_IMUS;i++){
		if(!en[i]) continue;

		if(en_debug){
			// print in such a way that we can easily copy/paste into
			printf("imu%d cold temp (C): %6.3f\n", i, (double)temp_avg[i]);
			printf("imu%d gyro cold values (rad/s): %9.5f, %9.5f, %9.5f\n", i, (double)gyro_avg[i][0], (double)gyro_avg[i][1], (double)gyro_avg[i][2]);
			printf("imu%d accl cold values (m/s^2): %9.5f, %9.5f, %9.5f\n", i, (double)accl_avg[i][0], (double)accl_avg[i][1], (double)accl_avg[i][2]);
			//printf("for spreadsheet: %0.3f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f\n",(double)temp_avg[i], (double)gyro_avg[i][0], (double)gyro_avg[i][1], (double)gyro_avg[i][2], (double)accl_avg[i][0], (double)accl_avg[i][1], (double)accl_avg[i][2]);
		}
		// save to the auto-sample array
		samples_temp_avg[i][samples_collected]=temp_avg[i];
		for(j=0;j<3;j++){
			samples_gyro_avg[i][samples_collected][j] = gyro_avg[i][j];
			samples_accl_avg[i][samples_collected][j] = accl_avg[i][j];
		}
	}
	samples_collected = 1; // first sample collected!



	////////////////////////////////////////////////////////////////////////////
	// Start HOT stage
	////////////////////////////////////////////////////////////////////////////

	_start_stress();

	printf("\n\n\n\n");
	printf(COLOR_GRN "============================================================================\n" RESET_COLOR);
	printf("We're now starting the " COLOR_RED "HOT" RESET_COLOR " stage of the calibration. We just turned off\n");
	printf("the fan, and started stressing the CPU to heat up the PCB.\n");
	printf("This will run automatically for a while collecting data points until enough\n");
	printf("data has been collected, something gets too " COLOR_RED "hot" RESET_COLOR ", or the timeout is reached.\n");
	printf(COLOR_GRN "============================================================================\n" RESET_COLOR);

	print_running = 1;
	int64_t start_time = _time_monotonic_ns();

	// pick first enabled imu to monitor for now
	int monitored_imu = 0;
	for(i=0;i<N_IMUS;i++){
		if(en[i]){
			monitored_imu = i;
			break;
		}
	}
	// // if another IMU started out hotter, monitor that instead
	// for(i=0;i<N_IMUS;i++){
	// 	if(!en[i]) continue;
	// 	if(samples_temp_avg[i]>samples_temp_avg[monitored_imu]){
	// 		monitored_imu = i;
	// 	}
	// }
	if(en_debug){
		printf("monitoring imu%d\n", monitored_imu);
	}

	// set up last temperature
	double last_temp[N_IMUS];
	for(i=0;i<N_IMUS;i++) last_temp[i]=0.0f;

	print_running = 1;
	int auto_running = 1;
	while(auto_running){
		if(!main_running) _quit(0);

		int should_sample = 0;

		if(_collect_samples()!=0){
			printf("Data too noisy. Make sure VOXL is COMPLETELY STILL\n");
		}

		// check for temp limit
		for(i=0;i<N_IMUS;i++){
			if(!en[i]) continue;
			if(temp_avg[i]>T_SAFETY){
				printf("IMU%d hit temp limit, finishing test\n", i);
				auto_running = 0;
				should_sample = 1;
			}
		}

		// check for timeout
		if((_time_monotonic_ns()-start_time)>TIMEOUT_NS){
			printf("timeout, finishing test\n");
			auto_running = 0;
			should_sample = 1;
		}

		// check for temperature steady state
		static int steady_counter[N_IMUS] = {0};
		for(i=0;i<N_IMUS;i++){
			if(!en[i]) continue;

			// bump counter if the temp is not climbing. This may happen a few times
			// at the beginning of the test, hence the need for a counter and reset
			if((temp_avg[i]-last_temp[i]) < 0.08){
				steady_counter[i]++;
				if(en_debug){
					printf("\nimu%d steady counter: %d\n", i, steady_counter[i]);
				}
			}
			/*
			// reset counter once it starts heating up faster
			// disabled for now, causes more problems than it helps
			else if((temp_avg[i]-last_temp[i]) > 0.15){
				if(en_debug && steady_counter[i]>0){
					printf("\nimu%d reset steady counter\n", i);
				}
				steady_counter[i] = 0;
			}
			*/
			last_temp[i]=temp_avg[i];
		}

		int are_any_still_heating = 0;
		for(i=0;i<N_IMUS;i++){
			if(!en[i]) continue;
			if(steady_counter[i] < 10){
				are_any_still_heating = 1;
			}
		}
		if(!are_any_still_heating){
			printf("all imus reached steady state, finishing test\n");
			auto_running = 0;
			should_sample = 1;
		}


		// check if the monitored imu has gotten at least 5 degrees hotter
		if(temp_avg[monitored_imu] > samples_temp_avg[monitored_imu][samples_collected-1]+T_DELTA){
			should_sample = 1;
		}

		// if the safety checks passed, collect the sample
		if(should_sample){

			printf("taking sample #%d\n", samples_collected+1);
			for(i=0;i<N_IMUS;i++){
				if(!en[i]) continue;
				// save averages in memory for the hot reading
				samples_temp_avg[i][samples_collected]=temp_avg[i];
				for(j=0;j<3;j++){
					samples_gyro_avg[i][samples_collected][j] = gyro_avg[i][j];
					samples_accl_avg[i][samples_collected][j] = accl_avg[i][j];
				}
			}
			// increment sample counter
			samples_collected++;
		}

		// check for finish condition
		if(samples_collected>=N_AUTO_SAMPLES){
			auto_running = 0;
		}
	}


	_stop_stress();
	print_running = 0;


	// print data for spreadsheet in debug mode
	if(en_debug){
		for(i=0;i<N_IMUS;i++){
			if(!en[i]) continue;

			printf("imu%d for spreadsheet:\n", i);

			// print in such a way that we can easily copy/paste into
			for(j=0;j<samples_collected;j++){
				printf("%0.3f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f\n",\
						(double)samples_temp_avg[i][j],\
						(double)samples_gyro_avg[i][j][0]-(double)samples_gyro_avg[i][0][0],\
						(double)samples_gyro_avg[i][j][1]-(double)samples_gyro_avg[i][0][1],\
						(double)samples_gyro_avg[i][j][2]-(double)samples_gyro_avg[i][0][2],\
						(double)samples_accl_avg[i][j][0]-(double)samples_accl_avg[i][0][0],\
						(double)samples_accl_avg[i][j][1]-(double)samples_accl_avg[i][0][1],\
						(double)samples_accl_avg[i][j][2]-(double)samples_accl_avg[i][0][2]);
			}
		}
	}

	// make sure we have enough data for the quadratic fit
	if(samples_collected<3){
		fprintf(stderr, "not enough samples, need at least 3\n");
		_quit(-1);
	}



	////////////////////////////////////////////////////////////////////////////
	// calculate results
	////////////////////////////////////////////////////////////////////////////

	printf("\nCOMPUTING RESULTS\n");

	for(i=0;i<N_IMUS;i++){
		// skip disabled IMUs
		if(!en[i]) continue;

		// do gyro for x,y,z
		for(j=0;j<3;j++){

			// populate x and y vectors from temp and gyro samples
			double x[MAX_SAMPLES], y[MAX_SAMPLES];
			for(k=0;k<samples_collected;k++){
				x[k] = samples_temp_avg[i][k];
				y[k] = samples_gyro_avg[i][k][j] - samples_gyro_avg[i][0][j];
			}
			// solve for the quadratic fit
			double coeff[3];
			_polynomial_regression(samples_collected, 2, x, y, coeff);

			if(en_debug){
				printf("%0.9f,%0.9f,%0.9f\n", coeff[0], coeff[1], coeff[2]);
			}

			// save to config arrays
			for(k=0;k<3;k++){
				gyro_drift_coeff[i][j][k] = coeff[k];
			}
		}

		// do accl for x,y,z
		for(j=0;j<3;j++){

			// populate x and y vectors from temp and accl samples
			double x[MAX_SAMPLES], y[MAX_SAMPLES];
			for(k=0;k<samples_collected;k++){
				x[k] = samples_temp_avg[i][k];
				y[k] = samples_accl_avg[i][k][j] - samples_accl_avg[i][0][j];
			}
			// solve for the quadratic fit
			double coeff[3];
			_polynomial_regression(samples_collected, 2, x, y, coeff);

			if(en_debug){
				printf("%0.9f,%0.9f,%0.9f\n", coeff[0], coeff[1], coeff[2]);
			}

			// save to config arrays
			for(k=0;k<3;k++){
				accl_drift_coeff[i][j][k] = coeff[k];
			}
		}

		has_temp_cal[i]=1; // flag in config file that this IMU now has a temp cal
	}


	// write out data
	if(cal_file_write()<0) _quit(-1);
	printf("\nSuccessfully wrote calibration to disk\n");
	printf("voxl-imu-server should reload the new calibration automatically.\n");
	printf("You can now run voxl-inspect-imu to check that the new data looks good\n");
	printf("\n");
	printf("Now is probably a good time to run the normal voxl-calibrate-imu\n");
	printf("calibration routine since the IMUs should both be up to temperature.\n");

	// successfully quit
	_quit(0);
	return 0;
}
