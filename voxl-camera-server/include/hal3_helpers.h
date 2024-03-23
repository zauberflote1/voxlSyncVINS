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

#ifndef HAL3_HELPERS
#define HAL3_HELPERS

#include <camera/CameraMetadata.h>
#include <hardware/camera3.h>

#include <modal_pipe.h>


//------------------------------------------------------------------------------------------------------------------------------
// Main HAL interface
//------------------------------------------------------------------------------------------------------------------------------
camera_module_t* HAL3_get_camera_module();

bool HAL3_is_cam_alive(int camId);

/**
 * @brief      Prints the resolutions of camera(s)
 *
 * @param[in]  camId  The camera to print resolutions for
 * 					-1 prints all available cameras
 */
void HAL3_print_camera_resolutions(int camId);


/**
 * @brief      Checks to see if a given config is supported for a camera
 *
 * @param[in]  camId  The camera to print resolutions for
 * 					-1 prints all available cameras
 */
bool HAL3_is_config_supported(int camId, int width, int height, int format);

/**
 * @brief      Generates a list of cameras to run based off what's plugged in
 *
 * @param[in]  cameras  Reference to list that we'll populate with camera info
 */
Status HAL3_get_debug_configuration(PerCameraInfo* cameras, int* num_cameras);

int estimateJpegBufferSize(camera_metadata_t* cameraCharacteristics, uint32_t width, uint32_t height);

void initMetaFromBlock(BufferBlock const& block, camera_image_metadata_t& meta);


// see the cpp file for details
int HAL3_get_gain_limits(int camId, uint32_t* min, uint32_t* max, \
                            uint32_t hard_lower_limit, uint32_t hard_upper_limit);

#endif
