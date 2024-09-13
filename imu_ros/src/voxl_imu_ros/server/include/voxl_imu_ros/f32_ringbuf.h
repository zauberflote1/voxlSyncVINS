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

#ifndef F32_RINGBUF_H
#define F32_RINGBUF_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct f32_ringbuf_t {
    float* d;      ///< pointer to dynamically allocated data
    int size;       ///< number of elements the buffer can hold
    int index;      ///< index of the most recently added value
    int initialized;///< flag indicating if memory has been allocated for the buffer
} f32_ringbuf_t;

#define F32_RINGBUF_INITIALIZER {\
    .d = NULL,\
    .size = 0,\
    .index = 0,\
    .initialized = 0}


f32_ringbuf_t f32_ringbuf_empty(void);
int   f32_ringbuf_alloc(f32_ringbuf_t* buf, int size);
int   f32_ringbuf_free(f32_ringbuf_t* buf);
int   f32_ringbuf_reset(f32_ringbuf_t* buf);
int   f32_ringbuf_insert(f32_ringbuf_t* buf, float val);
float f32_ringbuf_get_value(f32_ringbuf_t* buf, int position);
int   f32_ringbuf_copy_out_n_newest(f32_ringbuf_t* buf, int n, float* out);


#ifdef __cplusplus
}
#endif


#endif // F32_RINGBUF_H
