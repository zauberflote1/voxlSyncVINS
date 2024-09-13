#include <iostream>
#include <complex>
#include <string.h>

#include <modal_pipe_interfaces.h>

#include "fft.h"

using namespace std;

//#define M_PI 3.1415926535897932384

static int log2(int N)    /*function to calculate the log2(.) of int numbers*/
{
	int k = N, i = 0;
	while(k) {
		k >>= 1;
		i++;
	}
	return i - 1;
}

static int check(int n)    //checking if the number of element is a power of 2
{
	return n > 0 && (n & (n - 1)) == 0;
}

static int reverse(int N, int n)    //calculating revers number
{
	int j, p = 0;
	for(j = 1; j <= log2(N); j++) {
		if(n & (1 << (log2(N) - j)))
			p |= 1 << (j - 1);
	}
	return p;
}

static void ordina(complex<float>* f1, int N) //using the reverse order in the array
{
	complex<float> f2[MAX_FFT_BUF_LEN];
	for(int i = 0; i < N; i++)
		f2[i] = f1[reverse(N, i)];
	for(int j = 0; j < N; j++)
		f1[j] = f2[j];
}

static void fft(complex<float>* f, int N) //
{
	ordina(f, N);    //first: reverse order
	complex<float> *W;
	W = (complex<float> *)malloc(N / 2 * sizeof(complex<float>));
	W[1] = polar(1., -2. * M_PI / N);
	W[0] = 1;
	for(int i = 2; i < N / 2; i++)
		W[i] = pow(W[1], i);
	int n = 1;
	int a = N / 2;
	for(int j = 0; j < log2(N); j++) {
		for(int i = 0; i < N; i++) {
			if(!(i & n)) {
				complex<float> temp = f[i];
				complex<float> Temp = W[(i * a) % (n * a)] * f[i + n];
				f[i] = temp + Temp;
				f[i + n] = temp - Temp;
			}
		}
		n *= 2;
		a = a / 2;
	}
	free(W);
}


static void single_sided_fft_magnitude(complex<float>* f, int n_samples, float* mag)
{
	fft(f, n_samples);
	int n_out = (n_samples/2)+1;
	for(int i=0; i<n_out; i++){
		mag[i] = 2.0f*fabs(f[i])/n_samples;
	}
	return;
}


static void single_sided_fft_magnitude_array(float* f, int n_samples, float* mag)
{
	complex<float> vec[n_samples];
	for(int i=0; i<n_samples; i++){
		vec[i]=f[i];
	}
	single_sided_fft_magnitude(vec,n_samples, mag);
	return;
}


static void calc_freq_array(int n_samples, float dt, float* freq_hz)
{
	int n_out = (n_samples/2)+1;
	for(int i=0; i<n_out; i++){
		freq_hz[i] = (float)i/(dt*n_samples);
	}
}


static int is_power_of_two(int x)
{
	//checks whether a number is zero or not
	if(x == 0) return 0;

	//true till x is not equal to 1
	while( x != 1){
		//checks whether a number is divisible by 2
		if(x % 2 != 0) return 0;
		x /= 2;
	}
	return 1;
}


/*
int main()
{
	int n_in = 128;
	int n_out = (n_in/2)+1;
	float dt = 0.001;

	complex<float> vec[MAX] = {0,
	0.9009,
	1.4095,
	1.3368,
	0.7911,
	0.1122,
   -0.3165,
   -0.2780,
	0.1628,
	0.6981,
	0.9511,
	0.6885,
   -0.0433,
   -0.9344,
   -1.5706,
   -1.6511,
   -1.1475,
   -0.3176,
	0.4329,
	0.7660,
	0.5878,
	0.0910,
   -0.3591,
   -0.4317,
   -0.0188,
	0.7000,
	1.3503,
	1.5643,
	1.1820,
	0.3416,
   -0.5878,
   -1.1986,
   -1.2558,
   -0.8150,
   -0.1840,
	0.2511,
	0.2391,
   -0.1982,
   -0.7796,
   -1.1211,
   -0.9511,
   -0.2654,
	0.6601,
	1.4106,
	1.6480,
	1.2878,
	0.5404,
   -0.2042,
   -0.5866,
   -0.4682,
   -0.0000,
	0.4682,
	0.5866,
	0.2042,
   -0.5404,
   -1.2878,
   -1.6480,
   -1.4106,
   -0.6601,
	0.2654,
	0.9511,
	1.1211,
	0.7796,
	0.1982,
   -0.2391,
   -0.2511,
	0.1840,
	0.8150,
	1.2558,
	1.1986,
	0.5878,
   -0.3416,
   -1.1820,
   -1.5643,
   -1.3503,
   -0.7000,
	0.0188,
	0.4317,
	0.3591,
   -0.0910,
   -0.5878,
   -0.7660,
   -0.4329,
	0.3176,
	1.1475,
	1.6511,
	1.5706,
	0.9344,
	0.0433,
   -0.6885,
   -0.9511,
   -0.6981,
   -0.1628,
	0.2780,
	0.3165,
   -0.1122,
   -0.7911,
   -1.3368,
   -1.4095,
   -0.9009,
   -0.0000,
	0.9009,
	1.4095,
	1.3368,
	0.7911,
	0.1122,
   -0.3165,
   -0.2780,
	0.1628,
	0.6981,
	0.9511,
	0.6885,
   -0.0433,
   -0.9344,
   -1.5706,
   -1.6511,
   -1.1475,
   -0.3176,
	0.4329,
	0.7660,
	0.5878,
	0.0910,
   -0.3591,
   -0.4317,
   -0.0188,
	0.7000,
	1.3503,
	1.5643};

	int i;
	float mag[n_out];
	float freq[n_out];

	single_sided_fft_magnitude(vec, n_in, mag);
	calc_freq_array(n_in, dt, freq);

	printf("\n\n freq(hz) magnitude\n");
	for(i=0; i<n_out; i++){
		printf("%6.2f  %6.4f\n", freq[i], mag[i]);
	}


	return 0;
}
*/


int fft_buffer_init(fft_buffer_t* buf, int size, float sample_rate_hz)
{
	if(buf->initialized){
		fprintf(stderr, "ERROR in %s, buffer already initialized\n", __FUNCTION__);
		return -1;
	}
	if(size>MAX_FFT_BUF_LEN){
		fprintf(stderr, "ERROR in %s, buffer size must be <=%d\n", __FUNCTION__, MAX_FFT_BUF_LEN);
		return -1;
	}

	for(int i=0; i<6; i++){
		if(f32_ringbuf_alloc(&buf->buf[i], size)){
			fprintf(stderr, "ERROR in %s, allocating f32_ringbuf\n", __FUNCTION__);
			return -1;
		}

		buf->tmp[i] = (float*)malloc(size*sizeof(float));
		if(buf->tmp[i]<=0){
			fprintf(stderr, "ERROR in %s, allocating memory\n", __FUNCTION__);
			return -1;
		}
	}

	buf->size = size;
	buf->n = 0;
	buf->sample_rate_hz = sample_rate_hz;
	buf->initialized = 1;
	return 0;
}


int fft_buffer_free(fft_buffer_t* buf)
{
	if(!buf->initialized){
		return 0;
	}

	for(int i=0; i<6; i++){
		f32_ringbuf_free(&buf->buf[i]);
		if(buf->tmp[i]>0) free(buf->tmp[i]);
		buf->tmp[i] = 0;
	}

	buf->initialized = 0;
	buf->size = 0;
	buf->n = 0;
	return 0;
}




int fft_buffer_add(fft_buffer_t* buf, imu_data_t* data, int n)
{
	if(!buf->initialized){
		fprintf(stderr, "ERROR in %s, fft_buffer not initialized\n", __FUNCTION__);
		return -1;
	}

	pthread_mutex_lock(&buf->mtx);

	for(int j=0; j<n; j++){
		for(int i=0; i<3; i++){
			f32_ringbuf_insert(&buf->buf[i],   (float)data[j].gyro_rad[i]);
			f32_ringbuf_insert(&buf->buf[3+i], (float)data[j].accl_ms2[i]);
		}
		if(buf->n < buf->size) buf->n++;
	}
	pthread_mutex_unlock(&buf->mtx);
	return 0;
}


int fft_buffer_calc(fft_buffer_t* buf, int n, imu_fft_data_t* out)
{
	if(!buf->initialized){
		fprintf(stderr, "ERROR in %s, fft_buffer not initialized\n", __FUNCTION__);
		return -1;
	}
	if(n > buf->size){
		fprintf(stderr, "ERROR in %s, requested %d samples but buffer is only %d long\n", __FUNCTION__, n, buf->size);
		return -1;
	}
	if(n > buf->n){
		fprintf(stderr, "ERROR in %s, requested %d samples but buffer only contains %d samples\n", __FUNCTION__, n, buf->n);
		return -1;
	}
	if(!is_power_of_two(n)){
		fprintf(stderr, "ERROR in %s, n should be a power of 2, got %d\n", __FUNCTION__, n);
		return -1;
	}

	// lock and copy out data to temp buffers so that the data is in order
	// and so that the driver can add more data while we calc the fft
	pthread_mutex_lock(&buf->mtx);
	int i;
	for(i=0;i<6;i++){
		f32_ringbuf_copy_out_n_newest(&buf->buf[i], n, buf->tmp[i]);
	}
	pthread_mutex_unlock(&buf->mtx);

	// now actually calc the results
	for(i=0;i<3;i++){
		single_sided_fft_magnitude_array(buf->tmp[i],   n, out->gyro_rad[i]);
		single_sided_fft_magnitude_array(buf->tmp[i+3], n, out->accl_ms2[i]);
	}

	out->magic_number = IMU_FFT_MAGIC_NUMBER;
	out->n_freq = (n/2)+1;
	out->max_freq_hz = buf->sample_rate_hz/2.0f;
	return 0;
}

