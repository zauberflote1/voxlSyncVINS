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

#ifndef CAM_SERVER_CV_ROUTINES_H
#define CAM_SERVER_CV_ROUTINES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


void debayer_BGGR8_or_RGGB8_to_mono(uint8_t* in, uint8_t* out, int w, int h);

void debayer_BGGR8_to_rgb8(uint8_t* in, uint8_t* out, int w, int h);
void debayer_RGGB8_to_rgb8(uint8_t* in, uint8_t* out, int w, int h);

void apply_3x3_box_blur(uint8_t* in, uint8_t* out, int w, int h);

void removePlaneStride(uint32_t stride, uint32_t width, uint32_t height,
        uint8_t const* __restrict__ src_plane, uint8_t* __restrict__ dest_plane);

void Mipi12ToRaw16(int meta, uint8_t *raw12Buf, uint16_t *raw16Buf);


void ConvertTo8bitRaw(uint8_t* pImg, uint32_t widthPixels, uint32_t heightPixels);

int Check10bit(uint8_t* pImg, uint32_t widthPixels, uint32_t heightPixels);



#ifdef __cplusplus
}
#endif

#endif // CAM_SERVER_CV_ROUTINES_H

