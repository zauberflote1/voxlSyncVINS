/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
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

#ifndef FFT_H
#define FFT_H

#include "f32_ringbuf.h"

#define IMU_FFT_MAGIC_NUMBER  (0x564F584C + 101)

#define MAX_FFT_BUF_LEN 512
#define MAX_FFT_FREQ_BINS ((MAX_FFT_BUF_LEN/2)+1)

typedef struct imu_fft_data_t{
    uint32_t magic_number; ///< Set to IMU_FFT_MAGIC_NUMBER
    int n_freq;
    float max_freq_hz; // min frequency should always be 0

    float accl_ms2[3][MAX_FFT_FREQ_BINS];     ///< XYZ acceleration in m/s^2
    float gyro_rad[3][MAX_FFT_FREQ_BINS];     ///< XYZ gyro rotation in rad/s

} __attribute__((packed)) imu_fft_data_t;



/**
 * You don't have to use this read buffer size, but it is HIGHLY
 * recommended to use a multiple of the packet size so that you never read a
 * partial packet which would throw the reader out of sync. Here we use a nice
 * number of 400 packets which is perhaps more than necessary but only takes a
 * little under 16K of memory which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define IMU_FFT_RECOMMENDED_READ_BUF_SIZE   (sizeof(imu_fft_data_t) * 5)
#define IMU_FFT_RECOMMENDED_PIPE_SIZE       (128*1024)





typedef struct fft_buffer_t{
	int initialized;
	f32_ringbuf_t buf[6];
	float* tmp[6];
	float sample_rate_hz;
	int size;
	int n;
	pthread_mutex_t mtx;
} fft_buffer_t;


int fft_buffer_init(fft_buffer_t* buf, int size, float sample_rate_hz);
int fft_buffer_free(fft_buffer_t* buf);

int fft_buffer_add(fft_buffer_t* buf, imu_data_t* data, int n);


// this will also lock the buffer mutex, but only for the short time that
// is necessary
// n is the number of samples to calc over
int fft_buffer_calc(fft_buffer_t* buf, int n, imu_fft_data_t* out);


#endif // FFT_H
