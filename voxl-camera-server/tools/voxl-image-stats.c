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

#include <pthread.h>
#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <time.h>
#include <unistd.h>	// for usleep()
#include <string.h>
#include <stdlib.h> // for atoi()
#include <math.h>
#include <errno.h>

#include <modal_start_stop.h>
#include <modal_pipe_client.h>

#include <cv_routines.h>

#define CLIENT_NAME		"voxl-image-noise"

#define CLEAR_TERMINAL		"\033c"		// same as typing "clear" in bash
#define DISABLE_WRAP		"\033[?7l"	// disables line wrap, be sure to enable before exiting
#define ENABLE_WRAP			"\033[?7h"	// default terminal behavior
#define CLEAR_LINE			"\033[2K"	// erases line but leaves curser in place
#define GOTO_TOP_LEFT		"\033[f"	// move curser to top left

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN];



static void _print_usage(void) {
printf("\n\
\n\
prints some statistics for a greyscale image stream\n\
\n\
typical usage:\n\
/# voxl-image-noise tracking_grey\n\
\n");
	return;
}





// we calculate noise by quantifying standard deviation of the image
// before and after applying a 3x3 box blur. Meant to quantify high-frequency noise
static double _calc_noise(uint8_t* in, int w, int h)
{
	uint8_t tmp[w*h];
	apply_3x3_box_blur(in, tmp, w, h);

	uint64_t sum = 0;

	for(int i=0; i<w*h; i++){
		int16_t a = in[i];
		int16_t b = tmp[i];
		sum += (a-b) * (a-b);
	}

	return sqrt((double)sum/(double)(w*h));
}




static void _quit(int exit_code)
{
	pipe_client_close_all();
	printf("\n");
	exit(exit_code);
}


static int _parse_opts(int argc, char* argv[])
{
	static struct option long_options[] =
	{
		{"help",               no_argument,              0, 'h'},
		{0, 0, 0, 0}
	};


	while(1){
		int option_index = 0;
		int c = getopt_long(argc, argv, "h", long_options, &option_index);

		if(c == -1) break; // Detect the end of the options.

		switch(c){
		case 0:
			// for long args without short equivalent that just set a flag
			// nothing left to do so just break.
			if (long_options[option_index].flag != 0) break;
			break;
		case 'h':
			_print_usage();
			exit(0);
			break;
		default:
			_print_usage();
			exit(0);
		}
	}

	// scan through the non-flagged arguments for the desired pipe
	for(int i=optind; i<argc; i++){
		if(pipe_path[0]!=0){
			fprintf(stderr, "ERROR: Please specify only one pipe\n");
			_print_usage();
			exit(-1);
		}
		if(pipe_expand_location_string(argv[i], pipe_path)<0){
			fprintf(stderr, "ERROR: Invalid pipe name: %s\n", argv[i]);
			exit(-1);
		}
	}

	// make sure a pipe was given
	if(pipe_path[0] == 0){
		fprintf(stderr, "ERROR: There must be one input pipe argument given\n");
		_print_usage();
		exit(-1);
	}

	return 0;
}



// called whenever we disconnect from the server
static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	printf("Server Disconnected\n");
	return;
}





// camera helper callback whenever a frame arrives
static void _helper_cb(__attribute__((unused)) int ch, camera_image_metadata_t meta, __attribute__((unused))char* frame, __attribute__((unused)) void* context)
{

	if(meta.format != IMAGE_FORMAT_RAW8){
		fprintf(stderr, "can only parse IMAGE_FORMAT_RAW8 images\n");
		_quit(-1);
	}

	int w = meta.width;
	int h = meta.height;

	// min, max, and mean stuff
	uint8_t min = 255;
	uint8_t max = 0;
	uint64_t sum = 0;

	// saturation counter stuff
	uint8_t low_cut = 20;
	uint8_t high_cut = 255 - 20;
	int low_ctr = 0;
	int high_ctr = 0;

	for(int i=0; i<w*h; i++){

		// find min/max limits
		if(frame[i]<min){
			min = frame[i];
		}
		else if(frame[i]>max){
			max = frame[i];
		}

		// for total mean
		sum += (uint64_t)frame[i];

		// checking for saturation
		if(frame[i]<low_cut){
			low_ctr++;
		}
		else if(frame[i]>high_cut){
			high_ctr++;
		}
	}

	sum/=(uint64_t)(w*h);

	double perc_low = 100.0 * (double)low_ctr / (double)(w*h);
	double perc_high = 100.0 * (double)high_ctr / (double)(w*h);

	double noise = _calc_noise((uint8_t*)frame, w, h);
	double exp_ms = (double)meta.exposure_ns/1000000.0;

	printf("\r%3d   %3d  %3d   %4.1f%%  %4.1f%%   %3.1f    %4.1f    %3d  ", min, max, (int)sum, perc_low, perc_high, noise, exp_ms, meta.gain);
	fflush(stdout);

	return;
}

static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	printf(CLEAR_TERMINAL GOTO_TOP_LEFT);
	printf(" min  max  mean  black  white  noise  exp(ms)  gain\n");
	return;
}



int main(int argc, char* argv[])
{
	// check for options
	if(_parse_opts(argc, argv)) return -1;

	// set some basic signal handling for safe shutdown.
	// quitting without cleanup up the pipe can result in the pipe staying
	// open and overflowing, so always cleanup properly!!!
	enable_signal_handler();
	main_running = 1;


	// set up all our MPA callbacks
	pipe_client_set_camera_helper_cb(0, _helper_cb, NULL);
	pipe_client_set_disconnect_cb(0, _disconnect_cb, NULL);
	pipe_client_set_connect_cb(0, _connect_cb, NULL);
	printf("waiting for server at %s\n", pipe_path);
	pipe_client_open(0, pipe_path, CLIENT_NAME, EN_PIPE_CLIENT_CAMERA_HELPER, 0);

	// keep going until the  signal handler sets the running flag to 0
	while(main_running) usleep(500000);

	_quit(0);
}
