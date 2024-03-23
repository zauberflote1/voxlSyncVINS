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

#include <stdint.h>

#include <modal_journal.h>



// hal3 gives us a jpeg in buffer of size buffersize
// this find where the jpeg actually lives inside that buffer
size_t find_jpeg_buffer_size(const uint8_t* buffer, int buffersize, int* start_index) {

    // Find the start and end of the JPEG image
    int i = 0;
    while (i < buffersize - 1) {
        if (buffer[i] == 0xFF && buffer[i+1] == 0xD8) { 
            // Found the start of the image
            *start_index = i;
            int j = i + 2;
            while (j < buffersize - 1) {
                // Found a marker segment
                if (buffer[j] == 0xFF) { 
                    // End of the image
                    if (buffer[j+1] == 0xD9) { 
                        return j+2 - i;
                    } else if (buffer[j+1] == 0x00) { // Ignore "stuffing" byte
                        j += 2;
                    } else { 
                        // Make sure there's enough data for the length field
                        if (j+3 >= buffersize) { 
                            M_ERROR("Error: incomplete marker segment at byte %d\n", j);
                            return 1;
                        }
                        int segmentLength = (buffer[j+2] << 8) | buffer[j+3];
                        // Invalid segment length
                        if (segmentLength < 0 || segmentLength > 0xFFFF) { 
                            M_ERROR("Error: invalid marker segment length %d at byte %d\n", segmentLength, j+2);
                            return 1;
                        }
                        // Make sure there's enough data for the segment data
                        if (j+3+segmentLength >= buffersize) {
                            M_ERROR("Error: incomplete marker segment data at byte %d\n", j+2);
                            return 1;
                        }
                        // Skip the marker type and length fields
                        j += segmentLength + 2; 
                    }
                } else { 
                    // Not a marker segment, continue searching for end of image
                    j++;
                }
            }
        }
        i++;
    }
    return 1;
}

