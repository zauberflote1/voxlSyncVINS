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
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <getopt.h>
#include <string.h>
#include <stdlib.h> // for atoi(), exit(), and system()
#include <sched.h>
#include <errno.h>
#include <inttypes.h>

#include <modal_start_stop.h>
#include <modal_pipe_server.h>

#include "config_file.h"
#include "cal_file.h"
#include "imu_interface.h"
#include "misc.h"
#include "fft.h"

#define PROCESS_NAME "voxl-imu-server" // for PID file name

// mpu9250 can read the most in one go: 292. Increase this is a new imu is added
// which allows for more. For now use buffers with 300 for some padding.
#define MAX_FIFO_SAMPLES 300

// extern debug vars shared across anything that might want to print debug info
#include "global_debug_flags.h"
int en_print_fifo_count = 0;
int en_print_data = 0;
int en_print_timesync = 0;
int en_print_timestamps = 0;

// local vars
static int en_basic_read = 1;
static int en_self_test = 0;
static int self_test_imu = -1; // changed if user specifies one imu to be tested
static int force_cal_off = 0;
static int delay = 0;
static int force_common_frame_off = 0; // turn common frame off for calibration
static int force_imu[N_IMUS]; // debug tool set by args to force an IMU on
static pthread_t read_thread[N_IMUS];
static pthread_t fft_thread[N_IMUS];
static fft_buffer_t fft_buf[N_IMUS];
static cJSON* pipe_info_json[N_IMUS];

// update the pip info json fields after a cal and during setup
static void _update_info_json();


// printed if some invalid argument was given
static void _print_usage(void)
{
	printf("\n\
voxl-imu-server usually runs as a systemd background service. However, for debug\n\
purposes it can be started from the command line manually with any of the following\n\
debug options. When started from the command line, voxl-imu-server will automatically\n\
stop the background service so you don't have to stop it manually\n\
\n\
-b, --basic               perform basic register reads to get data instead of\n\
                            reading the IMU's FIFO buffer\n\
-c, --config              only parse the config file and exit, don't run\n\
-d, --delay {us}          simulate the CPU getting backed up by adding a delay\n\
                            in microseconds after each fifo read\n\
-f, --print_fifo_count    print num packets read from fifo each cycle\n\
-h, --help                print this help message\n\
-i, --enable_imu {i}      force enable an imu (0-%d) that might be disabled\n\
                            in the config file\n\
-p, --print_data          print all data as it's read, this can be a LOT of data\n\
                            when running at high sample rates, be careful\n\
-s, --print_timesync      print data about the timesync between apps proc and imu\n\
-t, --test                run the factory self-test. Both onboard IMUs (0 and 1)\n\
                            will be tested along with any auxilliary IMUs enabled\n\
                            in the config file.\n\
-u, --print_timestamps    print every sample timestamp in nanoseconds for debugging\n\
\n", MAX_IMU);
	return;
}

#define CONTROL_COMMANDS (COMMAND_START_CAL "," COMMAND_STOP_CAL)
// control listens for a calibration command.
static void _control_pipe_handler(int ch, char* string, int bytes, __attribute__((unused)) void* context)
{
	if(strncmp(string, COMMAND_START_CAL, strlen(COMMAND_START_CAL))==0){
		printf("Starting calibration mode!!\n");
		force_cal_off = 1;
		force_common_frame_off = 1;
		return;
	}
	if(strncmp(string, COMMAND_STOP_CAL, strlen(COMMAND_STOP_CAL))==0){
		printf("Stopping calibration mode!!\n");
		cal_file_read();
		_update_info_json();
		printf("new calibration data:\n");
		cal_file_print();
		force_cal_off = 0;
		force_common_frame_off = 0;
		return;
	}

	printf("WARNING: Server received unknown command through the control pipe!\n");
	printf("got %d bytes. Command is: %s on ch %d\n", bytes, string, ch);
	return;
}


// when a client connects make sure the fifo turns on
static void _connect_handler(int ch, int client_id, char* client_name, __attribute__((unused)) void* context)
{
	// sanity check
	int i = ch%N_IMUS;
	if(i<0 || i>MAX_IMU) return;

	printf("client id %d connected to channed %d, name: %s\n",client_id, ch, client_name);
	// The main purpose of this callback is to start the FIFO
	// no need to do that in basic read mode
	if(!en_basic_read){
		imu_fifo_start(i);
	}
	return;
}

static void _disconnect_handler(int ch, int client_id, char* name, __attribute__((unused)) void* context)
{
	printf("client \"%s\" with id %d has disconnected from channel %d\n", name, client_id, ch);
	return;
}

// called after an imu cal to update the server pipe info files
static void _update_info_json(void)
{
	for(int i=0; i<N_IMUS; i++){
		// skip disabled imus
		if(!imu_enable[i]) continue;
		if(pipe_info_json[i]==NULL) continue;

		cJSON* item = cJSON_GetObjectItem(pipe_info_json[i], "is_calibrated");
		if(item==NULL){
			fprintf(stderr, "WARNING: could not fetch is_calibrated from JSON\n");
			continue;
		}
		if(has_static_cal) item->type = cJSON_True;
		else item->type = cJSON_False;

		item = cJSON_GetObjectItem(pipe_info_json[i], "is_temp_calibrated");
		if(item==NULL){
			fprintf(stderr, "WARNING: could not fetch is_calibrated from JSON\n");
			continue;
		}
		if(has_temp_cal[i]) item->type = cJSON_True;
		else item->type = cJSON_False;
		pipe_server_update_info(i);
	}
	return;

}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"basic",                 no_argument,       0, 'b'},
		{"config",                no_argument,       0, 'c'},
		{"delay",                 required_argument, 0, 'd'},
		{"print_fifo_count",      no_argument,       0, 'f'},
		{"help",                  no_argument,       0, 'h'},
		{"enable_imu",            required_argument, 0, 'i'},
		{"print_data",            no_argument,       0, 'p'},
		{"print_timesync",        no_argument,       0, 's'},
		{"test",                  no_argument,       0, 't'},
		{"print_timestamps",      no_argument,       0, 'u'},
		{0, 0, 0, 0}
	};

	while(1){
		int id;
		int option_index = 0;
		int c = getopt_long(argc, argv, "bcd:fhi:pstu", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;

		case 'b':
			printf("enabling basic read instead of FIFO\n");
			en_basic_read = 1;
			break;

		case 'c':
			if(config_file_read()){
				exit(-1);
			}
			// read cal file so that during the voxl-mpa-configure procedure
			// we can validate the cal file and delete old invalid ones
			if(cal_file_read()){
				exit(-1);
			}
			exit(0);
			break;

		case 'd':
			delay = atoi(optarg);
			if(delay<0){
				fprintf(stderr, "ERROR: delay must be positive\n");
				_print_usage();
				return -1;
			}
			else printf("using delay of %dus\n", delay);
			break;

		case 'f':
			printf("enabling debug print of FIFO count\n");
			en_print_fifo_count = 1;
			break;

		case 'h':
			_print_usage();
			return -1;

		case 'i':
			id = atoi(optarg);
			if(id<0 || id>MAX_IMU){
				fprintf(stderr, "ERROR: must specify 0-%d for enable_imu argument\n", MAX_IMU);
				return -1;
			}
			force_imu[id] = 1;
			printf("Forcing imu%d on\n", id);
			break;

		case 'p':
			printf("enabling debug print of imu_data\n");
			en_print_data = 1;
			break;

		case 's':
			printf("enabling debug print of timesync\n");
			en_print_timesync = 1;
			break;

		case 't':
			en_self_test = 1;
			// if an imu was specified, only enable that one
			// if(optarg){
			// 	id = atoi(optarg);
			// 	if(id<0 || id>MAX_IMU){
			// 		fprintf(stderr, "ERROR: must specify 0-%d for --test argument\n", MAX_IMU);
			// 		fprintf(stderr, "or use --test with no option to test IMUs 0 and 1\n");
			// 		fprintf(stderr, "plus any aux IMUs enabled in the config file\n");
			// 		return -1;
			// 	}
			// 	self_test_imu = id;
			// }
			break;

		case 'u':
			printf("enabling debug print of all timestamps\n");
			en_print_timestamps = 1;
			break;

		default:
			_print_usage();
			return -1;
		}
	}

	return 0;
}


static void _quit(int ret)
{
	for(int i=0;i<N_IMUS;i++){
		imu_close(i);
		fft_buffer_free(&fft_buf[i]);
	}
	pipe_server_close_all();
	remove_pid_file(PROCESS_NAME);
	if(ret==0) printf("Exiting Cleanly\n");
	exit(ret);
	return;
}


static void* _read_thread_func(void* context)
{
	static int64_t next_time = 0;
	int id = (intptr_t)context;
	int ret, i;
	imu_data_t data[MAX_FIFO_SAMPLES];
	int packets_read;		// number of packets read from fifo each cycle

	// assign magic number to each imu_data_struct ahead of time
	for(i=0;i<MAX_FIFO_SAMPLES;i++) data[i].magic_number = IMU_MAGIC_NUMBER;

	// sanity check
	if(id<0 || id>MAX_IMU) return NULL;

	// run until the global main_running flag becomes 0
	while(main_running){
		// normal fifo mode with basic mode turned off
		if(!en_basic_read){
			// if fifo hasn't been started yet just wait for a request
			if(!imu_is_fifo_running(id)){
				usleep(100000);
				continue;
			}
			ret = imu_fifo_read(id, data, &packets_read);
		}
		else{ // basic mode read
			ret = imu_basic_read(id, &data[0]);
			packets_read = 1;
		}

		// bad reading, try again
		if(ret){
			printf("WARNING bad reading on imu %d\n", id);
			continue;
		}

		// apply calibration if not disabled, must do before rotating
		if(!force_cal_off){
			for(i=0;i<packets_read;i++) imu_apply_calibration(id, &data[i]);
		}

		// rotate AFTER applying calibration.
		if(!force_common_frame_off && imu_rotate_common_frame[id]){
			for(i=0;i<packets_read;i++) imu_rotate_to_common_frame(id, &data[i]);
		}

		// optional debug prints
		if(en_print_fifo_count){
			printf("read %3d packets from imu%d\n", packets_read, id);
		}
		if(en_print_data && packets_read>0){
			// only print last packet or it's too much
			imu_print_data(id, data[packets_read-1]);
		}
		if(en_print_timestamps){
			for(i=0;i<packets_read;i++) printf("%10" PRId64 "\n", data[i].timestamp_ns);
		}

		// send to pipe
		if(packets_read>0){
			//int64_t t1 = my_time_monotonic_ns();
			pipe_server_write(id, (char*)data, packets_read*sizeof(imu_data_t));
			//int64_t t2 = my_time_monotonic_ns();
			//fprintf(stderr, "pipe write took %0.2fms\n", (t2-t1)/1000000.0);
		}

		// AFTER sending out critical data, add to the fft buffer which may
		// block waiting to lock a mutex
		if(fft_buf[id].initialized){
			fft_buffer_add(&fft_buf[id], data, packets_read);
		}

		// in basic mode or if delay is enabled in fifo mode, sleep a bit
		if(en_basic_read) usleep(800); //usleep(100000/imu_sample_rate_hz[id]); //
		else if (delay <= 0){
			my_loop_sleep(imu_fifo_poll_rate_hz[id], &next_time);
		}
		else usleep(delay);
	}

	return NULL;
}

static void* _fft_thread_func(void* context)
{
	static int64_t next_time = 0;
	int id = (intptr_t)context;
	//const int n = MAX_FFT_BUF_LEN;
	const int n = 256;

	// sanity check
	if(id<0 || id>MAX_IMU) return NULL;

	if(!fft_buf[id].initialized){
		fprintf(stderr, "WARNING, not running fft thread for imu%d since buffer failed to initialize\n", id);
		return NULL;
	}

	// run until the global main_running flag becomes 0
	while(main_running){
		my_loop_sleep(10.0, &next_time);

		if(pipe_server_get_num_clients(N_IMUS+id)<=0) continue;
		if(fft_buf[id].n<n) continue; // waiting for data still

		imu_fft_data_t data;

		//int64_t t1 = my_time_monotonic_ns();
		if(fft_buffer_calc(&fft_buf[id], n, &data)) continue;
		pipe_server_write(N_IMUS+id, &data, sizeof(imu_fft_data_t));

		//int64_t t2 = my_time_monotonic_ns();
		//fprintf(stderr, "fft calc took %0.2fms\n", (t2-t1)/1000000.0);
	}

	return NULL;
}


int main(int argc, char* argv[])
{
	int i;

	// parse opts first
	if(_parse_opts(argc, argv)) return -1;

////////////////////////////////////////////////////////////////////////////////
// parse arguments
////////////////////////////////////////////////////////////////////////////////

	if(!en_self_test){
		printf("loading calibration file\n");
		cal_file_read();
		cal_file_print();

		printf("loading config file\n");
		if(config_file_read()) return -1;
		config_file_print();
	}

	// enabled forced imu's which where disabled in config
	for(i=0;i<N_IMUS;i++) if(force_imu[i]) imu_enable[i]=1;

	// check if at least one IMU is enabled;
	int n_enabled = 0;
	for(i=0;i<N_IMUS;i++) if(imu_enable[i]) n_enabled++;

	if(!en_self_test && !n_enabled){
		fprintf(stderr, "No IMU is enabled!!\n");
		fprintf(stderr, "Enable at least 1 imu in the config file\n");
		fprintf(stderr, "or use the -i {i} option to force enable an imu\n");
		_quit(-1);
	}

////////////////////////////////////////////////////////////////////////////////
// gracefully handle an existing instance of the process and associated PID file
////////////////////////////////////////////////////////////////////////////////

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(kill_existing_process(PROCESS_NAME, 2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		_quit(-1);
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	make_pid_file(PROCESS_NAME);


////////////////////////////////////////////////////////////////////////////////
// set this critical process to use FIFO scheduler with high priority
////////////////////////////////////////////////////////////////////////////////

	struct sched_param param;
	memset(&param, 0, sizeof(param));
	param.sched_priority = THREAD_PRIORITY_RT_HIGH;
	int ret = sched_setscheduler(0, SCHED_FIFO, &param);
	if(ret==-1){
		fprintf(stderr, "WARNING Failed to set priority, errno = %d\n", errno);
	}
	// check
	ret = sched_getscheduler(0);
	if(ret!=SCHED_FIFO){
		fprintf(stderr, "WARNING: failed to set scheduler\n");
	}


////////////////////////////////////////////////////////////////////////////////
// do the self test if requested, quit right after
////////////////////////////////////////////////////////////////////////////////

	if(en_self_test){
		imu_self_test_result_t result[N_IMUS] = {SELF_TEST_INITIALIZER,SELF_TEST_INITIALIZER,SELF_TEST_INITIALIZER,SELF_TEST_INITIALIZER};
		int ret = 0;
		// only enable one IMU is user specified one IMU in arguments
		if(self_test_imu>=0){
			for(i=0;i<N_IMUS;i++) imu_enable[i]=0;
			imu_enable[self_test_imu] = 1;
		}
		// otherwise enable the one onboard IMU along with any AUX imus already
		// enabled in the config file
		else{
			imu_enable[0] = 1;
		}
		// test any and all enabled IMUs
		for(i=0;i<N_IMUS;i++){
			if(imu_enable[i]){
				if(imu_detect(i)) continue;
				ret |= imu_self_test(i, &result[i]);
			}
		}
		// then print final results
		for(i=0;i<N_IMUS;i++){
			if(imu_enable[i]){
				imu_self_test_print_results(i, result[i]);
			}
		}
		printf("This self-test required the voxl-imu-server service to stop\n");
		printf("You can restart it if desired with:  systemctl restart voxl-imu-server\n");
		_quit(ret);
	}

////////////////////////////////////////////////////////////////////////////////
// start the imu and pipes if enabled
////////////////////////////////////////////////////////////////////////////////
	if(imu_detect_board()){
		fprintf(stderr, "WARNING, failed to detect imu_apps, trying again\n");
		usleep(1000000);
		if(imu_detect_board()){
			fprintf(stderr, "ERROR, failed to detect imu_apps on second try\n");
			_quit(-1);
		}
		else{
			printf("INFO: succeeded to detect imu_apps on second try\n");
		}
	}

////////////////////////////////////////////////////////////////////////////////
// start the imu and pipes if enabled
////////////////////////////////////////////////////////////////////////////////

	int success = 0; // set to 1 if at least 1 imu succeded to init
	imu_data_t dummy; // dummy struct to write data into during basic test
	for(i=0;i<N_IMUS;i++){
		// skip disabled imus
		if(!imu_enable[i]) continue;
		printf("Initializing IMU%d\n", i);
		if(imu_init(i)){
			fprintf(stderr, "ERROR: failed to initialize IMU%d\n", i);
			imu_enable[i]=0; // flag imu as not enabled since it failed
			continue;
		}
		if(imu_basic_read(i, &dummy)){
			fprintf(stderr, "ERROR: failed to read data from IMU%d\n", i);
			imu_enable[i]=0; // flag imu as not enabled since it failed
			continue;
		}

		char* names[N_IMUS]		= VOXL_IMU_NAME_ARRAY;
		char* locations[N_IMUS]	= VOXL_IMU_LOCATION_ARRAY;

		// create the imu data pipe
		pipe_info_t info = { \
			"",\
			"",\
			"imu_data_t",\
			PROCESS_NAME,\
			IMU_RECOMMENDED_PIPE_SIZE,\
			0};

		strcpy(info.name, names[i]);
		strcpy(info.location, locations[i]);

		int flags = SERVER_FLAG_EN_CONTROL_PIPE;
		pipe_server_set_control_cb(i, _control_pipe_handler, NULL);
		pipe_server_set_connect_cb(i, _connect_handler, NULL);
		pipe_server_set_disconnect_cb(i, _disconnect_handler, NULL);

		if(pipe_server_create(i, info, flags)){
			imu_enable[i]=0; // flag imu as not enabled since it failed
			imu_close(i);
			continue;
		}

		// update if the imu is calibrated or not
		pipe_info_json[i] = pipe_server_get_info_json_ptr(i);
		if(cJSON_AddBoolToObject(pipe_info_json[i], "is_calibrated", has_static_cal)==NULL){
			fprintf(stderr, "WARNING: could not add is_calibrated field to info JSON\n");
		}
		if(cJSON_AddBoolToObject(pipe_info_json[i], "is_temp_calibrated", has_temp_cal[i])==NULL){
			fprintf(stderr, "WARNING: could not add is_temp_calibrated field to info JSON\n");
		}
		pipe_server_update_info(i);

		pipe_server_set_available_control_commands(i, CONTROL_COMMANDS);

		// create the fft pipe and buffer
		pipe_info_t info2 = { \
			"",\
			"",\
			"imu_fft_data_t",\
			PROCESS_NAME,\
			IMU_FFT_RECOMMENDED_PIPE_SIZE,\
			0};

		strcpy(info2.name, names[i]);
		strcat(info2.name, "_fft");

		pipe_server_set_connect_cb(N_IMUS+i, _connect_handler, NULL);
		pipe_server_set_disconnect_cb(N_IMUS+i, _disconnect_handler, NULL);

		if(pipe_server_create(N_IMUS+i, info2, 0)){
			fprintf(stderr, "WARNING: failed to create FFT pipe for IMU %d\n", i);
			continue;
		}

		// TODO fetch the real ODR from the timestamp filter while running
		// for now this works for VOXL2
		if(fft_buffer_init(&fft_buf[i], MAX_FFT_BUF_LEN, imu_sample_rate_hz[i]/0.9765625)){
			continue;
		}


		success=1; // flag that at least one IMU succeeded
	}

	// check that at least one IMU enabled
	if(!success){
		fprintf(stderr, "ERROR: failed to init at least one IMU\n");
		_quit(-1);
	}

	// if running in forced mode and not in basic mode, start the fifo now
	// so the user can see debug data without waiting for a request
	for(i=0;i<N_IMUS;i++){
		if(force_imu[i] && !en_basic_read){
			printf("Forcing imu%d to start now instead of waiting for client\n", i);
			imu_fifo_start(i);
		}
	}


////////////////////////////////////////////////////////////////////////////////
// start read threads and wait
////////////////////////////////////////////////////////////////////////////////

	// indicate to the soon-to-be-started read thread that we are initialized
	// and running, this is an extern variable in start_stop.c
	main_running=1;

	pthread_attr_t tattr;
	pthread_attr_init(&tattr);
	for(i=0;i<N_IMUS;i++){
		if(imu_enable[i]){
			int64_t j = i;
			pthread_create(&read_thread[i], &tattr, _read_thread_func, (void*)j);
			pthread_create(&fft_thread[i], &tattr, _fft_thread_func, (void*)j);
		}
	}

	// run until start/stop module catches a signal and changes main_running to 0
	while(main_running) usleep(500000000);


////////////////////////////////////////////////////////////////////////////////
// join read threads before closing everything since they read/write
////////////////////////////////////////////////////////////////////////////////

	for(i=0;i<N_IMUS;i++){
		if(imu_enable[i]){
			printf("joining read thread %d\n", i);
			pthread_join(read_thread[i], NULL);
			printf("joining fft thread %d\n", i);
			pthread_join(fft_thread[i], NULL);
		}
	}
	_quit(0);
	return 0;
}
