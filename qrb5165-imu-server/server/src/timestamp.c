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

#include "timestamp.h"
#include "global_debug_flags.h"


// this tries to estimate the timestamp of the newest packet read from the fifo
int64_t calc_filtered_ts_ns(int64_t time_before_read, int records, double* clock_ratio,\
							int64_t last_ts_ns, double odr, int last_read_was_good)
{

	// if there was only one reading, the sdsp probably sat waiting for that
	// data and returned right after grabbing it so ignore the time_before
	// subtract a fudge factor for the RPC call latency
	int64_t monotonic;


	// otherwise assume what we read from the fifo was there when we went to
	// subtract 5.5ms for the register read time and subtract half of the odr
	// that 5.5ms was tuned based on VIO offset from camera timestamp
	monotonic = (time_before_read - 5500000) - (500000000/odr);

	// this is our best-guess given only clock_monotonic, but this is noisy!
	int64_t filtered_ts_ns = monotonic;

	// if the last read was good, (and assuming this read was good), we can guess
	// the next timestamp by adding the known dt to the last timestamp
	if(last_read_was_good){
		// start by guessing and see how far off we are
		int64_t guess_from_last_read = last_ts_ns + \
					(int64_t)((double)records*(*clock_ratio)*1000000000.0/odr);
		int64_t diff = monotonic - guess_from_last_read;

		// if we are within 50ms we guessed right, try to converge on the right ts
		if(diff<=50000000 && diff>=-50000000){
			// this filter seeks to converge on the static offset between our noisy
			// monotonic clock reading and the predicted timestamp
			// this should converge quite quickly to catch up after missed packets
			filtered_ts_ns = guess_from_last_read + (diff/50);
			if(en_print_timesync){
				//printf("scale: %f diff: %lld monotonic: %lld guess: %lld\n",clock_ratio[bus], diff, monotonic, guess_from_last_read);
				int64_t new_dt = 1000000000.0/(odr/(*clock_ratio));
				printf("scale: %f diff_ms: %6.1f dt_ms %7.3f\n", *clock_ratio, diff/1000000.0, new_dt/1000000.0);
			}
		}
		else if(en_print_timesync){
			printf("using monotonic time, diff too big: %lu\n", diff);
		}

		// diff should hover around 0 and just represent noise from the irregularity
		// in when the apps proc wakes up to service the data. If it trends up or
		// down then that indicates a difference in clock speed between apps and imu.
		// try to converge on that clock ratio.
		*clock_ratio += ((double)diff/1000000000.0)/50;
	}
	else{
		if(en_print_timesync){
			printf("using monotonic time, lost packets since last read\n");
		}
	}

	return filtered_ts_ns;
}
