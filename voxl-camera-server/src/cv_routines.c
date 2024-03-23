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
#include <string.h>
#include <modal_journal.h>

#include "misc.h"
#include "cv_routines.h"

//#define TIMING



// use nearest neighbor to populate missing edges
static void _populate_single_perimeter(uint8_t* in, int w, int h)
{
    int c,r;

    // top row
    for(c=1; c<(w-1); c++){
        in[c]=in[c+w];
    }

    // bottom row
    int start = (w*(h-1))+1;
    int end = (w*h)-1;
    for(c=start; c<end; c++){
        in[c]=in[c-w];
    }

    // left/right
    for(r=0; r<h; r++){
        in[(r*w)]       = in[(r*w)+1];
        in[((r+1)*w)-1] = in[((r+1)*w)-2];
    }

    return;
}


// use nearest neighbor to populate missing edges
static void _populate_double_perimeter(uint8_t* in, int w, int h)
{
    int c,r;

    // top row
    for(c=1; c<(w-1); c++){
        in[c]   = in[c+(2*w)];
        in[c+w] = in[c+(2*w)];
    }

    // bottom row
    int start = (w*(h-1))+1;
    int end = (w*h)-1;
    for(c=start; c<end; c++){
        in[c-w] = in[c-(2*w)];
        in[c]   = in[c-(2*w)];
    }

    // left/right
    for(r=0; r<h; r++){
        in[(r*w)]       = in[(r*w)+2];
        in[(r*w)+1]     = in[(r*w)+2];
        in[((r+1)*w)-1] = in[((r+1)*w)-3];
        in[((r+1)*w)-2] = in[((r+1)*w)-3];
    }

    return;
}



typedef struct bayer_kernel_3x3_t{
    // top_left_corner
    uint16_t k1[9];
    uint16_t k1_den;
    // top right corner
    uint16_t k2[9];
    uint16_t k2_den;
    // bottom left corner
    uint16_t k3[9];
    uint16_t k3_den;
    // bottom right corner
    uint16_t k4[9];
    uint16_t k4_den;
} bayer_kernel_3x3_t;


/*
also works for BGGR!
ov9782 upside-down

RG RG RG
GB GB GB
RG RG RG
GB GB GB
RG RG RG
GB GB GB
*/
static bayer_kernel_3x3_t get_kernel_RGGB8_to_mono8(void)
{
    bayer_kernel_3x3_t ret;

    // top left red pixel, 3x3 block has 1R 4G 4B
    // B G B
    // G R G
    // B G B
    const uint16_t k1[] = { 1, 1, 1,   1, 4, 1,   1, 1, 1};
    ret.k1_den = 12;
    memcpy(ret.k1, k1, 9*sizeof(uint16_t));

    // top right green pixel, 3x3 block has 2R 5G 2B
    // G B G
    // R G R
    // G B G
    const uint16_t k2[] = { 2, 5, 2,   5, 2, 5,   2, 5, 2};
    ret.k2_den = 30;
    memcpy(ret.k2, k2, 9*sizeof(uint16_t));

    // bottom left green pixel, 3x3 block has 2R 5G 2B
    // G R G
    // B G B
    // G R G
    const uint16_t k3[] = { 2, 5, 2,   5, 2, 5,   2, 5, 2};
    ret.k3_den = 30;
    memcpy(ret.k3, k3, 9*sizeof(uint16_t));

    // bottom right blue pixel, 3x3 block has 4R 4G 1B
    // R G R
    // G B G
    // R G R
    const uint16_t k4[] = { 1, 1, 1,   1, 4, 1,   1, 1, 1};
    ret.k4_den = 12;
    memcpy(ret.k4, k4, 9*sizeof(uint16_t));

    return ret;
}


/*
also works for BGGR!
ov9782 upside-down

This is not perfect, it's an approximation, but it does allow for the
denominator divide to be reduced to a bitshift which is %10 faster overall.
output will never exceed 252

RG RG RG
GB GB GB
RG RG RG
GB GB GB
RG RG RG
GB GB GB
*/
static bayer_kernel_3x3_t get_kernel_RGGB8_to_mono8_estimate(void)
{
    bayer_kernel_3x3_t ret;

    // top left red pixel, 3x3 block has 1R 4G 4B
    // B G B
    // G R G
    // B G B
    const uint16_t k1[] = { 21, 21, 21,   21, 85, 21,   21, 21, 21};
    ret.k1_den = 256;
    memcpy(ret.k1, k1, 9*sizeof(uint16_t));

    // top right green pixel, 3x3 block has 2R 5G 2B
    // G B G
    // R G R
    // G B G
    const uint16_t k2[] = { 17, 42, 17,   42, 17, 42,   17, 42, 17};
    ret.k2_den = 256;
    memcpy(ret.k2, k2, 9*sizeof(uint16_t));

    // bottom left green pixel, 3x3 block has 2R 5G 2B
    // G R G
    // B G B
    // G R G
    const uint16_t k3[] = { 17, 42, 17,   42, 17, 42,   17, 42, 17};
    ret.k3_den = 256;
    memcpy(ret.k3, k3, 9*sizeof(uint16_t));

    // bottom right blue pixel, 3x3 block has 4R 4G 1B
    // R G R
    // G B G
    // R G R
    const uint16_t k4[] = { 21, 21, 21,   21, 85, 21,   21, 21, 21};
    ret.k4_den = 256;
    memcpy(ret.k4, k4, 9*sizeof(uint16_t));

    return ret;

}



static void _apply_3x3_bayer_kernel(uint8_t* in, uint8_t* out, int w, int h, bayer_kernel_3x3_t k)
{
    int pos;
    uint16_t acc; // accumulator

    // itterate one 2x2 group at a time skipping top and bottom
    int r,c;
    r = 2;
    while(r<(h-2)){

        // itterate through columns 2 at a time
        c = 2;

        uint8_t* s0 = &in[((r-1)*w)+c-1]; // start of row above
        uint8_t* s1 = &in[((r-0)*w)+c-1]; // start of this row
        uint8_t* s2 = &in[((r+1)*w)+c-1]; // start of row below
        uint8_t* s3 = &in[((r+2)*w)+c-1]; // start of row below

        while(c<(w-2)){

            // top left corner of the kernel
            pos = (r*w)+c;

            // top left red pixel
            acc =   (k.k1[0]*s0[0]) + (k.k1[1]*s0[1]) + (k.k1[2]*s0[2]) + \
                    (k.k1[3]*s1[0]) + (k.k1[4]*s1[1]) + (k.k1[5]*s1[2]) + \
                    (k.k1[6]*s2[0]) + (k.k1[7]*s2[1]) + (k.k1[8]*s2[2]);

            out[pos] = acc / k.k1_den;

            // top right green pixel
            acc =   (k.k2[0]*s0[1]) + (k.k2[1]*s0[2]) + (k.k2[2]*s0[3]) + \
                    (k.k2[3]*s1[1]) + (k.k2[4]*s1[2]) + (k.k2[5]*s1[3]) + \
                    (k.k2[6]*s2[1]) + (k.k2[7]*s2[2]) + (k.k2[8]*s2[3]);
            out[pos+1] = acc / k.k2_den;

            // bottom left green pixel
            acc =   (k.k3[0]*s1[0]) + (k.k3[1]*s1[1]) + (k.k3[2]*s1[2]) + \
                    (k.k3[3]*s2[0]) + (k.k3[4]*s2[1]) + (k.k3[5]*s2[2]) + \
                    (k.k3[6]*s3[0]) + (k.k3[7]*s3[1]) + (k.k3[8]*s3[2]);
            out[pos + w] = acc / k.k3_den;

            // bottom right red
            acc =   (k.k4[0]*s1[1]) + (k.k4[1]*s1[2]) + (k.k4[2]*s1[3]) + \
                    (k.k4[3]*s2[1]) + (k.k4[4]*s2[2]) + (k.k4[5]*s2[3]) + \
                    (k.k4[6]*s3[1]) + (k.k4[7]*s3[2]) + (k.k4[8]*s3[3]);
            out[pos + w + 1] = acc / k.k4_den;

            // update our row pointers
            s0+=2;
            s1+=2;
            s2+=2;
            s3+=2;

            c+=2;
        }
        // end of row, go down 2
        r+=2;
    }
    return;
}


void debayer_BGGR8_or_RGGB8_to_mono(uint8_t* in, uint8_t* out, int w, int h)
{
    #ifdef TIMING
    int64_t t_start = start_clock();
    #endif

    bayer_kernel_3x3_t k_mono = get_kernel_RGGB8_to_mono8_estimate();
    //bayer_kernel_3x3_t k_mono = get_kernel_RGGB8_to_mono8();
    _apply_3x3_bayer_kernel(in, out, w, h, k_mono);
    _populate_double_perimeter(out, w, h);

    #ifdef TIMING
    stop_clock_and_print_time("debayer to mono", t_start, w*h);
    #endif
}



/*
ov9782 right-side-up
BG BG BG
GR GR GR
BG BG BG
GR GR GR
BG BG BG
GR GR GR
*/
void debayer_BGGR8_to_rgb8(uint8_t* in, uint8_t* out, int w, int h)
{
    int pos;

    // itterate one 2x2 group at a time skipping top and bottom
    int r = 2;
    while(r<(h-2)){
        uint8_t* s0 = &in[(r-1)*w]; // start of row above
        uint8_t* s1 = &in[r*w]; // start of this row
        uint8_t* s2 = &in[(r+1)*w]; // start of row below
        uint8_t* s3 = &in[(r+2)*w]; // start of row below

        // itterate through columns 2 at a time
        int c = 2;
        while(c<(w-2)){

            // top left blue pixel
            pos = ((r*w)+c)*3;
            out[pos+0]=(s0[c-1] + s0[c+1] + s2[c-1] + s2[c+1]) / 4; // X pattern
            out[pos+1]=(s0[c] + s1[c-1] + s1[c+1] + s2[c]) / 4;     // + pattern
            out[pos+2]=s1[c];                                       // itself
            c++;

            // top right green pixel
            pos += 3;
            out[pos+0]=(s0[c] + s2[c]) / 2;     // | pattern
            out[pos+1]=s1[c];                   // itself
            out[pos+2]=(s1[c-1] + s1[c+1]) / 2; // -- pattern
            r++;
            c--;

            // bottom left green pixel
            pos = ((r*w)+c)*3;
            out[pos+0]=(s2[c-1] + s2[c+1]) / 2; // -- pattern;
            out[pos+1]=s2[c];                       // itself;
            out[pos+2]=(s1[c] + s3[c]) / 2;     // | pattern
            c++;

            // bottom right red
            pos += 3;
            out[pos+0]=s2[c];                   // itself
            out[pos+1]=(s1[c] + s2[c-1] + s2[c+1] + s3[c]) / 4; // + pattern
            out[pos+2]=(s1[c-1] + s1[c+1] + s3[c-1] + s3[c+1]) / 4; // X pattern
            r--;
            c++;
        }
        // end of row, go down 2
        r+=2;
    }
}



/*
ov9782 upside-down
RG RG RG
GB GB GB
RG RG RG
GB GB GB
RG RG RG
GB GB GB
*/
void debayer_RGGB8_to_rgb8(uint8_t* in, uint8_t* out, int w, int h)
{
    int pos;

    // itterate one 2x2 group at a time skipping top and bottom
    int r = 2;
    while(r<(h-2)){
        uint8_t* s0 = &in[(r-1)*w]; // start of row above
        uint8_t* s1 = &in[r*w]; // start of this row
        uint8_t* s2 = &in[(r+1)*w]; // start of row below
        uint8_t* s3 = &in[(r+2)*w]; // start of row below

        // itterate through columns 2 at a time
        int c = 2;
        while(c<(w-2)){

            // top left red pixel
            pos = ((r*w)+c)*3;
            out[pos+0]=s1[c];                                       // itself
            out[pos+1]=(s0[c] + s1[c-1] + s1[c+1] + s2[c]) / 4;     // + pattern
            out[pos+2]=(s0[c-1] + s0[c+1] + s2[c-1] + s2[c+1]) / 4; // X pattern
            c++;

            // top right green pixel
            pos += 3;
            out[pos+0]=(s1[c-1] + s1[c+1]) / 2; // -- pattern
            out[pos+1]=s1[c];                   // itself
            out[pos+2]=(s0[c] + s2[c]) / 2;     // | pattern
            r++;
            c--;

            // bottom left green pixel
            pos = ((r*w)+c)*3;
            out[pos+0]=(s1[c] + s3[c]) / 2;     // | pattern
            out[pos+1]=s2[c];                   // itself;
            out[pos+2]=(s2[c-1] + s2[c+1]) / 2; // -- pattern;
            c++;

            // bottom right blue
            pos += 3;
            out[pos+0]=(s1[c-1] + s1[c+1] + s3[c-1] + s3[c+1]) / 4; // X pattern
            out[pos+1]=(s1[c] + s2[c-1] + s2[c+1] + s3[c]) / 4;     // + pattern
            out[pos+2]=s2[c];                                       // itself
            r--;
            c++;
        }
        // end of row, go down 2
        r+=2;
    }
}



// 0.25ms on laptop
void apply_3x3_box_blur(uint8_t* in, uint8_t* out, int w, int h)
{
    uint16_t acc; // accumulator
    int pos = w+1;

    for(int r=1;r<(h-1);r++){

        // make pointers to the start of the input rows for 3x3 block
        uint8_t* s0 = &in[((r-1)*w)]; // start of row above
        uint8_t* s1 = &in[((r-0)*w)]; // start of this row
        uint8_t* s2 = &in[((r+1)*w)]; // start of row below

        for(int c=1;c<(w-1);c++){

            acc =   (s0[0]) + (s0[1]) + (s0[2]) + \
                    (s1[0]) + (s1[1]) + (s1[2]) + \
                    (s2[0]) + (s2[1]) + (s2[2]);
            out[pos] = acc / 9;

            // update our position and row pointers
            s0++; s1++; s2++; pos++;
        }
        // skipping left and right edges
        pos+=2;
    }

    _populate_single_perimeter(out, w, h);
    return;
}



/**
 * Given a generic plane pointer with a given stride, width, and height, copy it
 * into `dest_buf` such that stride = width.
 *
 * `dest_buf` must be at least width*height bytes long. The source and
 * destination buffers *cannot* overlap.
 */
void removePlaneStride(uint32_t stride, uint32_t width, uint32_t height,
        uint8_t const* __restrict__ src_plane, uint8_t* __restrict__ dest_plane)
{
    for (uint32_t row = 0; row < height; row++) {
        uint8_t* dest_ptr      = dest_plane + (width * row);
        uint8_t const* src_ptr = src_plane  + (stride * row);
        memcpy(dest_ptr, src_ptr, width);
    }
}



void Mipi12ToRaw16(int size_bytes, uint8_t *raw12Buf, uint16_t *raw16Buf)
{
    // Convert image buffer from MIPI RAW12 to RAW16 format
    // ToF MIPI RAW12 is stored in the format of:
    // P1[11:4] P2[11:4] P2[3:0] P1[3:0]
    // 2 pixels occupy 3 bytes, no padding needed

    // ToF data is 12bit held in an 8bit format, use seperate counter to track 1.5x difference in data size
    int buf8Idx, buf16Idx;
    for (buf8Idx = 0, buf16Idx = 0; buf16Idx < size_bytes / 2; buf8Idx += 3, buf16Idx++){
        raw16Buf[(buf16Idx*2)]   = (raw12Buf[buf8Idx] << 4) + (raw12Buf[buf8Idx + 2] & 0x0F);
        raw16Buf[(buf16Idx*2)+1] = (raw12Buf[buf8Idx + 1] << 4) + ((raw12Buf[buf8Idx + 2] & 0xF0) >> 4);
    }
}



// -----------------------------------------------------------------------------------------------------------------------------
// Convert 10-bit RAW to 8-bit RAW
// -----------------------------------------------------------------------------------------------------------------------------
void ConvertTo8bitRaw(uint8_t* pImg, uint32_t widthPixels, uint32_t heightPixels)
{
    #ifdef TIMING
    int64_t t_start = start_clock();
    #endif

    // This link has the description of the RAW10 format:
    // https://gitlab.com/SaberMod/pa-android-frameworks-base/commit/d1988a98ed69db8c33b77b5c085ab91d22ef3bbc

    uint32_t *destBuffer = (uint32_t*) pImg;
    // Figure out size of the raw8 destination buffer in 32 bit words
    uint32_t destSize = (widthPixels * heightPixels) / 4;

    for (uint32_t i = 0; i < destSize; i++) {
        *destBuffer++ = *((uint32_t*) pImg);
        // Skip every fifth byte because that is just a collection of the 2
        // least significant bits from the previous four pixels. We don't want
        // those least significant bits.
        pImg += 5;
    }

    #ifdef TIMING
    stop_clock_and_print_time("convert 10bit to 8bit", t_start, widthPixels*heightPixels);
    #endif
}

int Check10bit(uint8_t* pImg, uint32_t widthPixels, uint32_t heightPixels)
{
    if (pImg == NULL) {
        fprintf(stderr, "%s was given NULL pointer for image\n", __FUNCTION__);
        return -1;
    }

    //check the row that is 4/5ths of the way down the image, if we just converted a
    //raw8 image to raw8, it will be empty
    uint8_t* row = &(pImg[(heightPixels * widthPixels)]);
    for(unsigned int i = 0; i < widthPixels; i++){
        if(row[i] != 0){
            return 1;
        }
    }
    return 0;
}
