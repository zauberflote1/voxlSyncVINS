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
#include <modal_journal.h>

#ifndef APQ8096
#include <cci_direct.h>
#endif

#include <camera/CameraMetadata.h>
#include <hardware/camera_common.h>

#include "common_defs.h"
#include "config_defaults.h"
#include "hal3_helpers.h"


#define NUM_MODULE_OPEN_ATTEMPTS 10


// Callback to indicate device status change
static void CameraDeviceStatusChange(const struct camera_module_callbacks* callbacks, int camera_id, int new_status)
{
    M_DEBUG("Camera %d device status change: %d\n", camera_id, new_status);
}
// Callback to indicate torch mode status change
static void TorchModeStatusChange(const struct camera_module_callbacks* callbacks, const char* camera_id, int new_status)
{
}
static const camera_module_callbacks_t moduleCallbacks = {CameraDeviceStatusChange, TorchModeStatusChange};

// -----------------------------------------------------------------------------------------------------------------------------
// Get the camera module (and initialize it if it hasn't been)
// -----------------------------------------------------------------------------------------------------------------------------
camera_module_t* HAL3_get_camera_module()
{

    static camera_module_t* cameraModule = NULL;

    if(cameraModule != NULL){
        return cameraModule;
    }

    M_DEBUG("Attempting to open the hal module\n");

    int i;
    for (i = 0;
         i < NUM_MODULE_OPEN_ATTEMPTS && hw_get_module(CAMERA_HARDWARE_MODULE_ID, (const hw_module_t**)&cameraModule);
         i++)
    {

        M_WARN("Camera module not opened, %d attempts remaining\n",
                        NUM_MODULE_OPEN_ATTEMPTS-i);
        sleep(1);
    }

    if(cameraModule == NULL){
        M_ERROR("Camera module not opened after %d attempts\n", NUM_MODULE_OPEN_ATTEMPTS);
        return NULL;
    }

    M_DEBUG("SUCCESS: Camera module opened on attempt %d\n", i);

    //This check should never fail but we should still make it
    if (cameraModule->init != NULL)
    {
        cameraModule->init();
        if (cameraModule->init == NULL)
        {
            M_ERROR("Camera module failed to init\n");
            return NULL;
        }
    }

    int numCameras = cameraModule->get_number_of_cameras();

    M_DEBUG("----------- Number of cameras: %d\n\n", numCameras);

    cameraModule->set_callbacks(&moduleCallbacks);

    #ifdef QRB5165
    if(voxl_cci_init()) {
        M_ERROR("Failed to open CCI interface\n");
        cameraModule = NULL;
        return NULL;
    }
    #endif

    return cameraModule;

}


// very dumb, used as a sanity check before polling camera specs.
bool HAL3_is_cam_alive(int camId)
{
    camera_info halCameraInfo;
    camera_module_t* cameraModule = HAL3_get_camera_module();

    if(cameraModule == NULL){
        M_ERROR("Failed to call HAL3_get_camera_module\n");
        return true;
    }

    int ret = cameraModule->get_camera_info(camId, &halCameraInfo);
    if(ret){
        return true;
    }
    return false;
}


bool HAL3_is_config_supported(int camId, int width, int height, int format)
{
    camera_info halCameraInfo;
    camera_module_t* cameraModule = HAL3_get_camera_module();

    if(cameraModule == NULL){
        M_ERROR("Failed to call HAL3_get_camera_module\n");
        return false;
    }

    int ret = cameraModule->get_camera_info(camId, &halCameraInfo);
    if(ret){
        M_ERROR("Failed to request camera info for camID %d, cam likely disconnected\n", camId);
        return false;
    }

    camera_metadata_t* pStaticMetadata = (camera_metadata_t *)halCameraInfo.static_camera_characteristics;
    camera_metadata_ro_entry entry;

    // Get the list of all stream resolutions supported and then go through each one of them looking for a match
    memset(&entry, 0, sizeof(entry));
    ret = find_camera_metadata_ro_entry(pStaticMetadata, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);

    // Sanity checks
    if(ret){
        M_ERROR("Failed to request camera stream configurations for camID %d\n", camId);
        return false;
    }
    if((entry.count % 4) != 0){
        M_ERROR("unknown format for camera stream configurations for camID %d\n", camId);
        return false;
    }

    // print the list of resolutions and formats hal3 says "should" be okay
    M_DEBUG("cam ID %d checking for fmt: %4d  w: %4d h: %4d o:%4d\n", camId, format, width, height, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT);
    for (size_t i = 0; i < entry.count; i+=4){
        M_DEBUG("i:%3d fmt:%3d w:%4d h:%4d o:%d\n", i, entry.data.i32[i], entry.data.i32[i+1], entry.data.i32[i+2], entry.data.i32[i+3]);
    }

    // search
    for (size_t i = 0; i < entry.count; i+=4){
        if ((entry.data.i32[i]   == format) &&
            (entry.data.i32[i+1] == width ) &&
            (entry.data.i32[i+2] == height) &&
            (entry.data.i32[i+3] == ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT))
        {
            M_VERBOSE("Successfully found configuration match for camera %d: %dx%d\n", camId, width, height);
            return true;
        }
    }

    return false;
}

// see android_pixel_format_t in /usr/include/system/graphics.h
// there are more but this covers the ones we have seen
static void _print_format(int f){
    switch(f){
    case HAL_PIXEL_FORMAT_RAW16: //32
        // also called HAL_PIXEL_FORMAT_RAW_SENSOR some places
        M_PRINT("HAL_PIXEL_FORMAT_RAW16");
        break;
    case HAL_PIXEL_FORMAT_RAW10: //37
        M_PRINT("HAL_PIXEL_FORMAT_RAW10");
        break;
    case HAL_PIXEL_FORMAT_RAW12: //38
        M_PRINT("HAL_PIXEL_FORMAT_RAW12");
        break;
    case HAL_PIXEL_FORMAT_BLOB: //33
        M_PRINT("HAL_PIXEL_FORMAT_BLOB");
        break;
    case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED: //34
        M_PRINT("HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED");
        break;
    case HAL_PIXEL_FORMAT_YCbCr_420_888: //35 NV12? (probably)
        M_PRINT("HAL_PIXEL_FORMAT_YCbCr_420_888");
        break;
    case HAL_PIXEL_FORMAT_RAW_OPAQUE: //36
        M_PRINT("HAL_PIXEL_FORMAT_RAW_OPAQUE");
        break;

#ifndef APQ8096
    case HAL_PIXEL_FORMAT_JPEG: //256 never seen this, shows as BLOB instead
        M_PRINT("HAL_PIXEL_FORMAT_JPEG");
        break;
#endif

    default:
        M_PRINT("unknown format: %d", f);
    }
    return;
}

void HAL3_print_camera_resolutions(int camId){

    camera_module_t* cameraModule = HAL3_get_camera_module();
    int width, height;

    if(cameraModule == NULL){
        M_ERROR("Failed to open hal3 module\n");
        return;
    }
    int numCameras = cameraModule->get_number_of_cameras();

    // recursively call myself for every camera
    if(camId == -1){

        M_DEBUG("Note: This list comes from the HAL module and may not be indicative\n");
        M_DEBUG("\tof configurations that have full pipelines\n\n");
        M_DEBUG("Number of cameras: %d\n\n", numCameras);

        for(int i = 0; i < numCameras; i++){
            HAL3_print_camera_resolutions(i);
        }
        return;
    }

    M_PRINT("\n====================================\n");
    M_PRINT("Stats for camera: %d\n", camId);
    camera_info cameraInfo;
    int ret = cameraModule->get_camera_info(camId, &cameraInfo);
    if(ret){
        M_ERROR("failed to call HAL3 get_camera_info for cam %d\n", camId);
        return;
    }

    camera_metadata_t* meta = (camera_metadata_t *)cameraInfo.static_camera_characteristics;
    camera_metadata_ro_entry entry;

    //get raw sizes
    memset(&entry, 0, sizeof(entry));
    find_camera_metadata_ro_entry(meta, ANDROID_SCALER_AVAILABLE_RAW_SIZES, &entry);
    M_PRINT("\nANDROID_SCALER_AVAILABLE_RAW_SIZES:\n");
    M_PRINT("These are likely supported by the sensor\n");
    for (uint32_t i = 0 ; i < entry.count; i += 2) {
        width = entry.data.i32[i+0];
        height = entry.data.i32[i+1];
        M_PRINT("%4d x%5d\n",width ,height);
    }

    // //get processed sizes (empty?)
    // memset(&entry, 0, sizeof(entry));
    // find_camera_metadata_ro_entry(meta, ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES, &entry);
    // M_PRINT("\nANDROID_SCALER_AVAILABLE_PROCESSED_SIZES:");
    // for (uint32_t i = 0 ; i < entry.count; i += 2) {
    //     if (i%16==0)
    //         M_PRINT("\n\t");
    //     width = entry.data.i32[i+0];
    //     height = entry.data.i32[i+1];
    //     M_PRINT("%4dx%5d, ",width ,height);
    // }
    // M_PRINT("\n");

    // //get video sizes (empty?)
    // memset(&entry, 0, sizeof(entry));
    // find_camera_metadata_ro_entry(meta, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT, &entry);
    // M_PRINT("\nANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT:");
    // for (uint32_t i = 0 ; i < entry.count; i += 2) {
    //     if (i%16==0)
    //         M_PRINT("\n\t");
    //     width = entry.data.i32[i];
    //     height = entry.data.i32[i+1];
    //     M_PRINT("%4dx%5d, ",width ,height);
    // }
    // M_PRINT("\n");

    //get video sizes
    memset(&entry, 0, sizeof(entry));
    find_camera_metadata_ro_entry(meta, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);
    M_PRINT("\nANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS:\n");
    M_PRINT("These are NOT necessarily supported by the sensor\n");
    for(uint32_t i = 0 ; i < entry.count; i += 4){
        // skip input resolutions, only show outputs
        if(entry.data.i32[i+3] != ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT) continue;

        // skip this unused format to reduce clutter
        if(entry.data.i32[i] == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) continue;

        M_PRINT("%4d x%5d ",\
                        entry.data.i32[i+1],
                        entry.data.i32[i+2]);
        _print_format(entry.data.i32[i+0]);
        M_PRINT("\n");
    }

    // //get snapshot sizes (empty?)
    // memset(&entry, 0, sizeof(entry));
    // find_camera_metadata_ro_entry(meta, ANDROID_SCALER_AVAILABLE_JPEG_SIZES, &entry);
    // M_PRINT("ANDROID_SCALER_AVAILABLE_JPEG_SIZES:");
    // for (uint32_t i = 0 ; i < entry.count; i += 2) {
    //     if (i%32==0)
    //         M_PRINT("\n\t");
    //     width = entry.data.i32[i];
    //     height = entry.data.i32[i+1];
    //     M_PRINT("%4dx%5d, ",width ,height);
    // }
    // M_PRINT("\n");

    memset(&entry, 0, sizeof(entry));
    find_camera_metadata_ro_entry(meta, ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, &entry);
    uint32_t min_sensitivity = entry.data.i32[0];
    uint32_t max_sensitivity = entry.data.i32[1];
    M_PRINT("\nANDROID_SENSOR_INFO_SENSITIVITY_RANGE\n\tmin = %d\n\tmax = %d\n",min_sensitivity,max_sensitivity);

    find_camera_metadata_ro_entry(meta, ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY, &entry);
    M_PRINT("\nANDROID_SENSOR_MAX_ANALOG_SENSITIVITY\n\t%d\n",entry.data.i32[0]);


    memset(&entry, 0, sizeof(entry));
    find_camera_metadata_ro_entry(meta, ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, &entry);
    unsigned long long min_exposure = entry.data.i64[0];  //ns
    unsigned long long max_exposure = entry.data.i64[1];  //ns
    M_PRINT("\nANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE\n\tmin = %lluns\n\tmax = %lluns\n",min_exposure,max_exposure);

    M_PRINT("\n");

    if(camId == (numCameras-1)){
        printf("====================================\n");
        printf("Number of cameras detected: %d\n", numCameras);
        printf("====================================\n");
    }

    return;
}



Status HAL3_get_debug_configuration(PerCameraInfo* cameras, int* numCameras)
{
    camera_module_t* cameraModule = HAL3_get_camera_module();

    if(cameraModule == NULL){
        M_ERROR("Failed to open hal3 module\n");
        return S_ERROR;
    }

    *numCameras = cameraModule->get_number_of_cameras();

    if(*numCameras == 0) {
        M_ERROR("Did not detect any cameras plugged in\n");
        return S_ERROR;
    }

    for(int i = 0; i < *numCameras; i++){

        sensor_t type;

        // Best way for now to detect camera type right now
        // TODO put TOF first since it's most unique
        // then add resolutions for 412 and 678
        if(HAL3_is_config_supported(i, 3840, 2160, HAL_PIXEL_FORMAT_BLOB)){
            type = SENSOR_IMX214;
            M_PRINT("Assuming type: IMX214 for camera %d\n", i);
        } else if(HAL3_is_config_supported(i, 1280, 800, HAL3_FMT_YUV)){
            type = SENSOR_OV9782;
            M_PRINT("Assuming type: OV9782 for camera %d\n", i);
        } else if(HAL3_is_config_supported(i, 640, 480, HAL_PIXEL_FORMAT_RAW10)){
            type = SENSOR_OV7251;
            M_PRINT("Assuming type: OV7251 for camera %d\n", i);
        } else {
            type = SENSOR_TOF;
            M_PRINT("Assuming type: PMD_TOF for camera %d\n", i);
        }

        cameras[i] = getDefaultCameraInfo(type);

        sprintf(cameras[i].name, "cam%d", i);
        cameras[i].camId = i;
        cameras[i].camId2 = -1;

    }


    return S_OK;
}

static const int  minJpegBufferSize = sizeof(camera3_jpeg_blob) + 1024 * 512;


// estimate how big we need to make the buffers for hal3 to put JPEGs into
// this should be bigger than the actual JPEGs
int estimateJpegBufferSize(camera_metadata_t* cameraCharacteristics, uint32_t width, uint32_t height)
{

    int maxJpegBufferSize = 0;
    camera_metadata_ro_entry jpegBufferMaxSize;
    find_camera_metadata_ro_entry(cameraCharacteristics,
                                        ANDROID_JPEG_MAX_SIZE,
                                        &jpegBufferMaxSize);
    if (jpegBufferMaxSize.count == 0) {
        M_ERROR("Find maximum JPEG size from metadata failed.!\n");
        return 0;
    }
    maxJpegBufferSize = jpegBufferMaxSize.data.i32[0];

    float scaleFactor = ((float)width * (float)height) /
        (((float)maxJpegBufferSize - (float)sizeof(camera3_jpeg_blob)) / 3.0f);
    int jpegBufferSize = minJpegBufferSize + (maxJpegBufferSize - minJpegBufferSize) * scaleFactor;

    return jpegBufferSize;
}


/**
 * Partially initializes the given camera_image_metadata_t from the given BufferBlock. This only modifies the
 * fields magic_number, width, height, and stride within the metadata.
 */
void initMetaFromBlock(BufferBlock const& block, camera_image_metadata_t& meta)
{
    meta.magic_number = CAMERA_MAGIC_NUMBER;
    meta.width  = block.width;
    meta.height = block.height;
    meta.stride = block.stride;
    meta.format = IMAGE_FORMAT_RAW8; // just a common default, stereo and color frames will change this later
}


// fetch the gain bounds from hal3 and additionally constrain that range
// within limits. For example, if the sensor allows a gain of 8000 but we only
// want it to go as high as 2000 set the hard upper limit to 2000
// hard lower limit should probably be set to 54 since old qeb5165 system images
// had that as the lower limit for all sensors. That was rescaled to a min of 100
// in system image 1.7.1 so setting the lower bount to 54 is safe, it will not
// constrain on newer system images.
// This function is meant to be used with libmodal_exposure to find the limits
// it can expect to set the sensor to
int HAL3_get_gain_limits(int camId, uint32_t* min, uint32_t* max, \
                            uint32_t hard_lower_limit, uint32_t hard_upper_limit)
{
    camera_module_t* cameraModule = HAL3_get_camera_module();

    if(cameraModule == NULL){
        M_ERROR("Failed to open hal3 module\n");
        return S_ERROR;
    }

    camera_info cameraInfo;
    int ret = cameraModule->get_camera_info(camId, &cameraInfo);
    if(ret){
        M_ERROR("failed to call HAL3 get_camera_info for cam %d\n", camId);
        return S_ERROR;
    }

    camera_metadata_t* meta = (camera_metadata_t *)cameraInfo.static_camera_characteristics;
    camera_metadata_ro_entry entry;

    memset(&entry, 0, sizeof(entry));
    find_camera_metadata_ro_entry(meta, ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, &entry);
    *min = entry.data.i32[0];
    *max = entry.data.i32[1];

    // check if hal3 simply doesn't know
    if(*min==0 || *max==0){
        *min = hard_lower_limit;
        *max = hard_upper_limit;
    }

    // apply bounds
    if(hard_lower_limit>*min) *min = hard_lower_limit;
    if(hard_upper_limit<*max) *max = hard_upper_limit;

    return 0;
}

