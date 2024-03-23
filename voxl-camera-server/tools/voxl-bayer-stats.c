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

#define CLIENT_NAME		"voxl-bayer-stats"


#define CLEAR_TERMINAL		"\033c"		// same as typing "clear" in bash
#define DISABLE_WRAP		"\033[?7l"	// disables line wrap, be sure to enable before exiting
#define ENABLE_WRAP			"\033[?7h"	// default terminal behavior
#define CLEAR_LINE			"\033[2K"	// erases line but leaves curser in place
#define GOTO_TOP_LEFT		"\033[f"	// move curser to top left

static char pipe_path[MODAL_PIPE_MAX_PATH_LEN];



static void _print_usage(void) {
printf("\n\
\n\
prints some statistics for a bayered image published by camera server\n\
it's up to the user to know what sensor in what orientation they are inspecting\n\
in order to correlate the 4 quadrants to RGB pixels.\n\
\n\
typical usage:\n\
/# voxl-bayer-stats tracking_bayer\n\
\n\
\n");
	return;
}


static void _quit(int exit_code)
{
	pipe_client_close_all();
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
	uint8_t min[4] = {255, 255,255,255};
	uint8_t max[4] = {0,0,0,0};
	uint64_t sum[4] = {0,0,0,0};

	// saturation counter stuff
	int low_ctr[4] = {0,0,0,0};
	int high_ctr[4] = {0,0,0,0};

	void check(int quad, uint8_t val)
	{
		const uint8_t low_cut = 20;
		const uint8_t high_cut = 255 - 20;

		// find min/max limits
		if(val<min[quad]){
			min[quad] = val;
		}
		else if(val>max[quad]){
			max[quad] = val;
		}

		// for total mean
		sum[quad] += (uint64_t)val;

		// checking for saturation
		if(val<low_cut){
			low_ctr[quad]++;
		}
		else if(val>high_cut){
			high_ctr[quad]++;
		}
	}

	uint8_t val;

	// now go through the image checking all 4 quadrants
	for(int r=0; r<h; r+=2){
		for(int c=0; c<w; c+=2){

			val=frame[((r+0)*w) + c + 0];
			check(0, val);

			val=frame[((r+0)*w) + c + 1];
			check(1, val);

			val=frame[((r+1)*w) + c + 0];
			check(2, val);

			val=frame[((r+1)*w) + c + 1];
			check(3, val);
		}
	}


	printf(CLEAR_TERMINAL GOTO_TOP_LEFT);
	printf("OV9782 is BGGR when upright, RGGB when rotated.\n");
	printf("Quadrants are as follows:\n Q0 Q1\n Q2 Q3\n");
	printf("             min  max  mean  black  white\n");

	for(int i=0; i<4; i++){

		int mean = sum[i]/(uint64_t)(w*h/4);

		double perc_low = 100.0 * (double)low_ctr[i] / (double)(w*h/4);
		double perc_high = 100.0 * (double)high_ctr[i] / (double)(w*h/4);

		printf("quadrant %d: %3d   %3d  %3d    %3.1f%%   %3.1f%%\n", \
				i, min[i], max[i], mean, perc_low, perc_high);

	}

	fflush(stdout);


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
	printf("waiting for server at %s\n", pipe_path);
	pipe_client_open(0, pipe_path, CLIENT_NAME, EN_PIPE_CLIENT_CAMERA_HELPER, 0);

	// keep going until the  signal handler sets the running flag to 0
	while(main_running) usleep(500000);

	_quit(0);
}
