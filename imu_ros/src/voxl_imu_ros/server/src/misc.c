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
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include "misc.h"


int64_t my_time_monotonic_ns()
{
	struct timespec ts;
	if(clock_gettime(CLOCK_MONOTONIC, &ts)){
		fprintf(stderr,"ERROR calling clock_gettime\n");
		return -1;
	}
	return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}

int64_t my_time_realtime_ns()
{
	struct timespec ts;
	if(clock_gettime(CLOCK_REALTIME, &ts)){
		fprintf(stderr,"ERROR calling clock_gettime\n");
		return -1;
	}
	return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}


// sleep and timing functions, TODO these should go into a misc.c!
void my_nanosleep(uint64_t ns){
	struct timespec req,rem;
	req.tv_sec = ns/1000000000;
	req.tv_nsec = ns%1000000000;
	// loop untill nanosleep sets an error or finishes successfully
	errno=0; // reset errno to avoid false detection
	while(nanosleep(&req, &rem) && errno==EINTR){
		req.tv_sec = rem.tv_sec;
		req.tv_nsec = rem.tv_nsec;
	}
	return;
}


int my_loop_sleep(double rate_hz, int64_t* next_time)
{
	int64_t current_time = my_time_monotonic_ns();

	// static variable so we remember when we last woke up
	if(*next_time<=0){
		*next_time = current_time;
	}

	// try to maintain output data rate
	*next_time += (1000000000.0/rate_hz);

	// uh oh, we fell behind, reset
	if(*next_time<=current_time){
		*next_time = current_time + (1000000000.0/rate_hz);
		return 0;
	}

	// normal operation
	my_nanosleep(*next_time-current_time);
	return 0;
}

