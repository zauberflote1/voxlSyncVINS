/*******************************************************************************
 * CHANGED THE DISCREPANCY OF THE STEREO FROM 30MS TO 1MS 
 * 
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
#include <cstdint>
#include <hardware/gralloc.h>
#include <modal_pipe_interfaces.h>
#include <modal_pipe_server.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <camera/CameraMetadata.h>
#include <hardware/camera_common.h>
#include <algorithm>
#include <royale/DepthData.hpp>
#include <tof_interface.hpp>
#include <voxl_cutils.h>
#include <modal_journal.h>
#include <modal_pipe.h>

#include "buffer_manager.h"
#include "common_defs.h"
#include "hal3_camera_manager.hpp"
#include "hal3_helpers.h"
#include "voxl_camera_server.h"
#include "voxl_cutils.h"
#include "jpeg_size.h"
#include "cv_routines.h"
#include "misc.h"
#include "pipe_clients.h"
#include "cci_direct_helpers.h"

#define EXPOSURE_CONTROL_COMMANDS "set_exp_gain,set_exp,set_gain,start_ae,stop_ae"

#define NUM_PREVIEW_BUFFERS  16 // used to be 32, really shouldnt need to be more than 7
#define NUM_SNAPSHOT_BUFFERS 16 // used to be 8, just making it consistent with the rest that are now 16
#define SMALL_VID_ALLOWED_ITEMS_IN_OMX_QUEUE 1 // favor latency when dropping frames
#define LARGE_VID_ALLOWED_ITEMS_IN_OMX_QUEUE 2 // only drop frames when really getting behind

#define JPEG_DEFUALT_QUALITY        75

// #define MAX_STEREO_DISCREPENCY_NS ((1000000000/configInfo.fps)*0.9)
#define MAX_STEREO_DISCREPENCY_NS (5000000)
// Platform Specific Flags
#ifdef APQ8096
    #define ROTATION_MODE  CAMERA3_STREAM_ROTATION_0
    #define OPERATION_MODE QCAMERA3_VENDOR_STREAM_CONFIGURATION_RAW_ONLY_MODE
    #define ENCODER_USAGE  GRALLOC_USAGE_HW_COMPOSER | GRALLOC_USAGE_HW_TEXTURE
    #define SNAPSHOT_DS    HAL_DATASPACE_JFIF
    #define NUM_STREAM_BUFFERS   16
    #define NUM_RECORD_BUFFERS   16
#elif QRB5165
    #define ROTATION_MODE  2
    #define OPERATION_MODE CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE
    #define ENCODER_USAGE  GRALLOC_USAGE_HW_VIDEO_ENCODER
    #define SNAPSHOT_DS    HAL_DATASPACE_V0_JFIF
    #define NUM_STREAM_BUFFERS   16 // shouldn't need more than 10, if the buffer pool is empty then OMX should be dropping more frames
    #define NUM_RECORD_BUFFERS   16 // shouldn't need more than 10, if the buffer pool is empty then OMX should be dropping more frames
#else
    #error "No Platform defined"
#endif

// Libexif includes for QRB platform
#ifndef APQ8096
#include <libexif/exif-data.h>
#include <libexif/exif-entry.h>
#include <libexif/exif-utils.h>
/* raw EXIF header data */
static const unsigned char exif_header[] = {
  0xff, 0xd8, 0xff, 0xe1
};
static const unsigned int exif_header_len = sizeof(exif_header);
#define FILE_BYTE_ORDER EXIF_BYTE_ORDER_INTEL
#endif


// -----------------------------------------------------------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------------------------------------------------------
PerCameraMgr::PerCameraMgr(PerCameraInfo pCameraInfo) :
    configInfo        (pCameraInfo),
    cameraId          (pCameraInfo.camId),
    en_preview        (pCameraInfo.en_preview),
    en_small_video         (pCameraInfo.en_small_video),
    en_large_video         (pCameraInfo.en_large_video),
    en_snapshot       (pCameraInfo.en_snapshot),
    fps               (pCameraInfo.fps),
    pre_width         (pCameraInfo.pre_width),
    pre_height        (pCameraInfo.pre_height),
    pre_halfmt        (previewHalFmtFromInfo(pCameraInfo)),
    vid_halfmt                (HAL_PIXEL_FORMAT_YCbCr_420_888),
    small_video_width         (pCameraInfo.small_video_width),
    small_video_height        (pCameraInfo.small_video_height),
    large_video_width         (pCameraInfo.large_video_width),
    large_video_height        (pCameraInfo.large_video_height),
    snap_width        (pCameraInfo.snap_width),
    snap_height       (pCameraInfo.snap_height),
    snap_halfmt       (HAL_PIXEL_FORMAT_BLOB),
    ae_mode           (pCameraInfo.ae_mode),
    pCameraModule     (HAL3_get_camera_module()),
    pVideoEncoderSmall(NULL),
    pVideoEncoderLarge(NULL),
    expHistInterface  (pCameraInfo.ae_hist_info),
    expMSVInterface   (pCameraInfo.ae_msv_info)
{

    strcpy(name, pCameraInfo.name);

    cameraCallbacks.cameraCallbacks = {&CameraModuleCaptureResult, &CameraModuleNotify};
    cameraCallbacks.pPrivate        = this;

    if(pCameraModule == NULL ){
        M_ERROR("Failed to get HAL module!\n");
        throw -EINVAL;
    }

    // Check if the stream configuration is supported by the camera or not. If cameraid doesnt support the stream configuration
    // we just exit. The stream configuration is checked into the static metadata associated with every camera.
    if (en_preview && !HAL3_is_config_supported(cameraId, pre_width, pre_height, pre_halfmt))
    {
        M_ERROR("Camera %d failed to find supported preview config: %dx%d\n", cameraId, pre_width, pre_height);

        throw EBADSLT;
    }
    if (en_small_video && !HAL3_is_config_supported(cameraId, small_video_width, small_video_height, vid_halfmt))
    {
        M_ERROR("Camera %d failed to find supported stream config: %dx%d\n", cameraId, small_video_width, small_video_height);

        throw EBADSLT;
    }
    if (en_large_video && !HAL3_is_config_supported(cameraId, large_video_width, large_video_height, vid_halfmt))
    {
        M_ERROR("Camera %d failed to find supported record config: %dx%d\n", cameraId, large_video_width, large_video_height);

        throw EBADSLT;
    }
    if (en_snapshot && !HAL3_is_config_supported(cameraId, snap_width, snap_height, snap_halfmt))
    {
        M_ERROR("Camera %d failed to find supported snapshot config: %dx%d\n", cameraId, snap_width, snap_height);

        throw EBADSLT;
    }

    char cameraName[20];
    sprintf(cameraName, "%d", cameraId);

    if (pCameraModule->common.methods->open(&pCameraModule->common, cameraName, (hw_device_t**)(&pDevice)))
    {
        M_ERROR("Open camera %s failed!\n", name);

        throw -EINVAL;
    }

    if (pDevice->ops->initialize(pDevice, (camera3_callback_ops*)&cameraCallbacks))
    {
        M_ERROR("Initialize camera %s failed!\n", name);

        throw -EINVAL;
    }

    if (ConfigureStreams())
    {
        M_ERROR("Failed to configure streams for camera: %s\n", name);

        throw -EINVAL;
    }

    if (en_preview) {
        if (bufferAllocateBuffers(pre_bufferGroup,
                                  NUM_PREVIEW_BUFFERS,
                                  pre_stream.width,
                                  pre_stream.height,
                                  pre_stream.format,
                                  pre_stream.usage)) {
            M_ERROR("Failed to allocate preview buffers for camera: %s\n", name);

            throw -EINVAL;
        }
        M_DEBUG("Successfully set up pipeline for stream: PREVIEW\n");
    }

    if (en_small_video) {

        if (bufferAllocateBuffers(small_vid_bufferGroup,
                                  NUM_STREAM_BUFFERS,
                                  small_vid_stream.width,
                                  small_vid_stream.height,
                                  small_vid_stream.format,
                                  small_vid_stream.usage)) {
            M_ERROR("Failed to allocate encode buffers for camera: %s\n", name);
            throw -EINVAL;
        }

        try{
            VideoEncoderConfig enc_info;
            enc_info = {
                .width =             (uint32_t)small_video_width,
                .height =            (uint32_t)small_video_height,
                .format =            (uint32_t)vid_halfmt,
                .venc_config =       pCameraInfo.small_venc_config,
                .frameRate =         pCameraInfo.fps,
                .inputBuffers =      &small_vid_bufferGroup,
                .outputPipe =        &smallVideoPipeEncoded
            };
            pVideoEncoderSmall = new VideoEncoder(&enc_info);
        } catch(int) {
            M_ERROR("Failed to initialize encoder for camera: %s\n", name);
            throw -EINVAL;
        }
        M_DEBUG("Successfully set up pipeline for stream: STREAM_SMALL_VID\n");
    }

    if (en_large_video) {

        if (bufferAllocateBuffers(large_vid_bufferGroup,
                                  NUM_RECORD_BUFFERS,
                                  large_vid_stream.width,
                                  large_vid_stream.height,
                                  large_vid_stream.format,
                                  large_vid_stream.usage)) {
            M_ERROR("Failed to allocate encode buffers for camera: %s\n", name);
            throw -EINVAL;
        }

        try{
            VideoEncoderConfig enc_info;
            enc_info = {
                .width =             (uint32_t)large_video_width,
                .height =            (uint32_t)large_video_height,
                .format =            (uint32_t)vid_halfmt,
                .venc_config =       pCameraInfo.large_venc_config,
                .frameRate =         pCameraInfo.fps,
                .inputBuffers =      &large_vid_bufferGroup,
                .outputPipe =        &largeVideoPipeEncoded
            };

            pVideoEncoderLarge = new VideoEncoder(&enc_info);
        } catch(int) {
            M_ERROR("Failed to initialize encoder for camera: %s\n", name);
            throw -EINVAL;
        }
        M_DEBUG("Successfully set up pipeline for stream: STREAM_LARGE_VID\n");
    }

    if (en_snapshot) {

        camera_info halCameraInfo;
        pCameraModule->get_camera_info(cameraId, &halCameraInfo);
        camera_metadata_t* pStaticMetadata = (camera_metadata_t *)halCameraInfo.static_camera_characteristics;

        int blobWidth = estimateJpegBufferSize(pStaticMetadata, snap_width, snap_height);

        if(bufferAllocateBuffers(snap_bufferGroup,
                                 NUM_SNAPSHOT_BUFFERS,
                                 blobWidth,
                                 1,
                                 snap_stream.format,
                                 snap_stream.usage)) {
            M_ERROR("Failed to allocate snapshot buffers for camera: %s\n", name);

            throw -EINVAL;
        }
        M_DEBUG("Successfully set up pipeline for stream: SNAPSHOT\n");
    }

#ifndef APQ8096

    // apply any register tweaks through cci-direct, things that should go into
    // the camera driver in a future system image
    cci_direct_apply_register_tweaks(configInfo.camId, configInfo.type);

    // set rotation and white balance, only for certain cameras
    cci_direct_set_rotation(configInfo.camId, configInfo.type, configInfo.en_rotate);

    if(en_preview && configInfo.en_raw_preview){
        cci_direct_set_white_balance(configInfo.camId, configInfo.type);
    }
#endif

    // set up slave for stereo mode
    if(configInfo.camId2 == -1){
        partnerMode = MODE_MONO;
    } else {
        // recursive CTOR on stereo cams
        partnerMode = MODE_STEREO_MASTER;

        PerCameraInfo newInfo = configInfo;
        sprintf(newInfo.name, "%s%s", name, "_child");
        newInfo.camId = newInfo.camId2;
        newInfo.camId2 = -1;

        // These are disabled until(if) we figure out a good way to handle them
        newInfo.en_small_video = false;
        newInfo.en_large_video = false;
        newInfo.en_snapshot = false;

        // since this is a camId_second, look at the rotation_second value for
        // independent rotation h/w config
        newInfo.en_rotate = newInfo.en_rotate2;

        otherMgr = new PerCameraMgr(newInfo);

        otherMgr->setMaster(this);
    }
}

PerCameraMgr::~PerCameraMgr() {
    if (partnerMode == MODE_STEREO_MASTER)
        delete otherMgr;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Create the streams that we will use to communicate with the camera module
// -----------------------------------------------------------------------------------------------------------------------------
int PerCameraMgr::ConfigureStreams()
{

    std::vector<camera3_stream_t*> streams;

    camera3_stream_configuration_t streamConfig = { 0 };
    streamConfig.num_streams    = 0;

#ifndef APQ8096
    const char* cameraName = "";
#endif

    if(en_preview){
        pre_stream.stream_type = CAMERA3_STREAM_OUTPUT;
        pre_stream.width       = pre_width;
        pre_stream.height      = pre_height;
        pre_stream.format      = pre_halfmt;
        pre_stream.data_space  = HAL_DATASPACE_UNKNOWN;
        // TODO: figure out if there's another way to output nv12 instead of
        // nv21 for preview. these flags do it, but they also cause buffers to
        // be way oversized.
        pre_stream.usage       = GRALLOC_USAGE_HW_COMPOSER | GRALLOC_USAGE_HW_TEXTURE;
        pre_stream.rotation    = ROTATION_MODE;
        pre_stream.max_buffers = NUM_PREVIEW_BUFFERS;
        pre_stream.priv        = 0;
#ifndef APQ8096
        pre_stream.physical_camera_id = cameraName;
#endif

        streams.push_back(&pre_stream);
        streamConfig.num_streams ++;
        M_VERBOSE("Adding preview stream for camera: %d\n", cameraId);
    }

    if(en_small_video) {
        small_vid_stream.stream_type = CAMERA3_STREAM_OUTPUT;
        small_vid_stream.width       = small_video_width;
        small_vid_stream.height      = small_video_height;
        small_vid_stream.format      = vid_halfmt;
        small_vid_stream.data_space  = HAL_DATASPACE_UNKNOWN;
        small_vid_stream.usage       = ENCODER_USAGE;
        small_vid_stream.rotation    = ROTATION_MODE;
        small_vid_stream.max_buffers = NUM_STREAM_BUFFERS;
        small_vid_stream.priv        = 0;
#ifndef APQ8096
        small_vid_stream.physical_camera_id = cameraName;
#endif

        streams.push_back(&small_vid_stream);
        streamConfig.num_streams ++;
        M_VERBOSE("Adding small video stream for camera: %d\n", cameraId);
    }

    if(en_large_video) {
        large_vid_stream.stream_type = CAMERA3_STREAM_OUTPUT;
        large_vid_stream.width       = large_video_width;
        large_vid_stream.height      = large_video_height;
        large_vid_stream.format      = vid_halfmt;
        large_vid_stream.data_space  = HAL_DATASPACE_UNKNOWN;
        large_vid_stream.usage       = ENCODER_USAGE;
        large_vid_stream.rotation    = ROTATION_MODE;
        large_vid_stream.max_buffers = NUM_RECORD_BUFFERS;
        large_vid_stream.priv        = 0;
#ifndef APQ8096
        large_vid_stream.physical_camera_id = cameraName;
#endif

        streams.push_back(&large_vid_stream);
        streamConfig.num_streams ++;
        M_VERBOSE("Adding large video stream for camera: %d\n", cameraId);
    }

    if(en_snapshot) {
        snap_stream.stream_type = CAMERA3_STREAM_OUTPUT;
        snap_stream.width       = snap_width;
        snap_stream.height      = snap_height;
        snap_stream.format      = snap_halfmt;
        snap_stream.data_space  = SNAPSHOT_DS;
        snap_stream.usage       = GRALLOC_USAGE_SW_READ_OFTEN;
        snap_stream.rotation    = ROTATION_MODE;
        snap_stream.max_buffers = NUM_SNAPSHOT_BUFFERS;
        snap_stream.priv        = 0;
#ifndef APQ8096
        snap_stream.physical_camera_id = cameraName;
#endif

        streams.push_back(&snap_stream);
        streamConfig.num_streams ++;
        M_VERBOSE("Adding snapshot stream for camera: %d\n", cameraId);
    }

    if(streamConfig.num_streams==0){
        M_ERROR("No streams enabled for for camera: %d\n", cameraId);
        return -EINVAL;
    }

    num_streams = streamConfig.num_streams;
    streamConfig.streams        = streams.data();
    streamConfig.operation_mode = OPERATION_MODE;
#ifndef APQ8096
    pSessionParams = allocate_camera_metadata(2, 8);
    int32_t frame_rate_rate[] = {fps,fps};
    add_camera_metadata_entry(pSessionParams,
                              ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
                              frame_rate_rate, 2);

    streamConfig.session_parameters = pSessionParams;
#endif
    // Call into the camera module to check for support of the required stream config i.e. the required usecase
    if (pDevice->ops->configure_streams(pDevice, &streamConfig))
    {
        M_ERROR("Configure streams failed for camera: %d\n", cameraId);
        return -EINVAL;
    }

    return S_OK;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Construct default camera settings that will be passed to the camera module to be used for capturing the frames
// -----------------------------------------------------------------------------------------------------------------------------
int PerCameraMgr::ConstructDefaultRequestSettings()
{

    // Get the default baseline settings
    camera_metadata_t* pDefaultMetadata =
            (camera_metadata_t *)pDevice->ops->construct_default_request_settings(pDevice, CAMERA3_TEMPLATE_PREVIEW);

    if(en_snapshot){
        pDefaultMetadata =
                    //(camera_metadata_t *)pDevice->ops->construct_default_request_settings(pDevice, CAMERA3_TEMPLATE_STILL_CAPTURE);
                    (camera_metadata_t *)pDevice->ops->construct_default_request_settings(pDevice, CAMERA3_TEMPLATE_VIDEO_RECORD);
    }

    // Modify all the settings that we want to
    requestMetadata = clone_camera_metadata(pDefaultMetadata);

    switch (ae_mode){

        case AE_OFF :
        case AE_LME_HIST :
        case AE_LME_MSV : {

            //This covers the 5 below modes, we want them all off
            uint8_t controlMode = ANDROID_CONTROL_MODE_OFF;
            uint8_t aeMode            =  ANDROID_CONTROL_AE_MODE_OFF;
            uint8_t antibanding       =  ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF;
            uint8_t afMode            =  ANDROID_CONTROL_AF_MODE_OFF;
            uint8_t awbMode           =  ANDROID_CONTROL_AWB_MODE_OFF;
            uint8_t faceDetectMode    =  ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;

            //This covers the 5 below modes, we want them all off
            requestMetadata.update(ANDROID_CONTROL_MODE,                &controlMode,        1);
            requestMetadata.update(ANDROID_CONTROL_AE_MODE,             &aeMode,             1);
            requestMetadata.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &faceDetectMode,     1);
            requestMetadata.update(ANDROID_CONTROL_AF_MODE,             &afMode,             1);
            requestMetadata.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &antibanding,        1);
            requestMetadata.update(ANDROID_CONTROL_AWB_MODE,            &awbMode,            1);

            break;
        }

        case AE_ISP : {

            uint8_t aeMode            =  ANDROID_CONTROL_AE_MODE_ON;
            uint8_t antibanding       =  ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF;
            uint8_t awbMode           =  ANDROID_CONTROL_AWB_MODE_AUTO;
            uint8_t afMode            =  ANDROID_CONTROL_AF_MODE_OFF;
            uint8_t faceDetectMode    =  ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;

            requestMetadata.update(ANDROID_CONTROL_AE_MODE,             &aeMode,             1);
            requestMetadata.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &antibanding,        1);
            requestMetadata.update(ANDROID_CONTROL_AWB_MODE,            &awbMode,            1);
            requestMetadata.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &faceDetectMode,     1);
            requestMetadata.update(ANDROID_CONTROL_AF_MODE,             &afMode,             1);
            break;
        }

   	    default:
   	        M_ERROR("unknown ae mode: %d\n", ae_mode);
   	        return -1;

    }

    if(en_snapshot){
        uint8_t jpegQuality     = JPEG_DEFUALT_QUALITY;

        requestMetadata.update(ANDROID_JPEG_QUALITY, &(jpegQuality), sizeof(jpegQuality));
    }

    int fpsRange[] = {configInfo.fps, configInfo.fps};
    int64_t frameDuration = 1e9 / configInfo.fps;

    requestMetadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, &fpsRange[0],        2);
    requestMetadata.update(ANDROID_SENSOR_FRAME_DURATION,       &frameDuration,      1);

    if(configInfo.type == SENSOR_TOF) {

        setExposure = 2259763;
        setGain     = 200;

        if(configInfo.fps != 5 && configInfo.fps != 15) {
            M_ERROR("Invalid TOF framerate: %d, must be either 5 or 15\n", configInfo.fps);
            return -1;
        }

        RoyaleListenerType dataType = RoyaleListenerType::LISTENER_DEPTH_DATA;

        TOFInitializationData initializationData = { 0 };

        initializationData.pDataTypes    = &dataType;
        initializationData.numDataTypes  = 1;
        initializationData.pListener     = this;
        initializationData.frameRate     = configInfo.fps;
        initializationData.range         = RoyaleDistanceRange::LONG_RANGE;

        #ifdef APQ8096
            VCU_silent(
                tof_interface = TOFCreateInterface();
            )
            if( tof_interface == NULL) {
                M_ERROR("Failed to create tof interface\n");
                return -1;
            }
            initializationData.pTOFInterface = tof_interface;
            int ret;
            VCU_silent(
                ret = TOFInitialize(&initializationData);
            )
            if(ret) {
                M_ERROR("Failed to initialize tof interface\n");
                return -1;
            }
        #elif QRB5165
            initializationData.cameraId      = cameraId;
            tof_interface = new TOFInterface(&initializationData);
        #endif
        M_VERBOSE("TOF interface created!\n");
    }

    return 0;
}


// just malloc space for now. Can put other setup stuff here in the future
void PerCameraMgr::SetupRawProcessing()
{
    if(configInfo.type == SENSOR_TOF) return;
    if(!en_preview) return;

    // only use raw processing with RAW types
    if( pre_halfmt != HAL_PIXEL_FORMAT_RAW10 && \
        pre_halfmt != HAL_PIXEL_FORMAT_RAW12 && \
        pre_halfmt != HAL_PIXEL_FORMAT_RAW16)
    {
        return;
    }

    // all cameras get a grey output
    // no need to malloc for the bayered data, that we read straight from image buffer
    previewFrameGrey8 = (uint8_t*)malloc(pre_width * pre_height);

    // color cameras also get a color output
    if(configInfo.bayer_fmt != BAYER_MONO){
        previewFrameRGB8 = (uint8_t*)malloc(pre_width * pre_height * 3);
    }

    return;
}

// -----------------------------------------------------------------------------------------------------------------------------
// This function opens the camera and starts sending the capture requests
// -----------------------------------------------------------------------------------------------------------------------------
void PerCameraMgr::Start()
{
    if(partnerMode != MODE_STEREO_SLAVE){
        if(SetupPipes()){
            M_ERROR("Failed to setup pipes for camera: %s\n", name);

            throw -EINVAL;
        }
    }

    SetupRawProcessing();

    if(pVideoEncoderSmall) {
        pVideoEncoderSmall->Start();
    }

    if(pVideoEncoderLarge) {
        pVideoEncoderLarge->Start();
    }

    pthread_condattr_t condAttr;
    pthread_condattr_init(&condAttr);
    pthread_condattr_setclock(&condAttr, CLOCK_MONOTONIC);
    pthread_mutex_init(&resultMutex, NULL);
    pthread_mutex_init(&stereoMutex, NULL);
    pthread_mutex_init(&aeMutex, NULL);
    pthread_cond_init(&resultCond, &condAttr);
    pthread_cond_init(&stereoCond, &condAttr);
    pthread_condattr_destroy(&condAttr);

    // Start the thread that will process the camera capture result. This thread wont exit till it consumes all expected
    // output buffers from the camera module or it encounters a fatal error
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_create(&requestThread, &attr, [](void* data){return ((PerCameraMgr*)data)->ThreadIssueCaptureRequests();}, this);
    pthread_create(&resultThread,  &attr, [](void* data){return ((PerCameraMgr*)data)->ThreadPostProcessResult();},  this);
    pthread_attr_destroy(&attr);

    if(partnerMode == MODE_STEREO_MASTER){
        otherMgr->Start();
    }

}

// -----------------------------------------------------------------------------------------------------------------------------
// This function stops the camera and does all necessary clean up
// -----------------------------------------------------------------------------------------------------------------------------
void PerCameraMgr::Stop()
{

    stopped = true;

    if(partnerMode == MODE_STEREO_MASTER){
        otherMgr->stopped = true;
    }

    pthread_join(requestThread, NULL);

    pthread_cond_broadcast(&stereoCond);
    pthread_cond_broadcast(&resultCond);

    pthread_cond_signal(&resultCond);
    pthread_mutex_unlock(&resultMutex);
    pthread_join(resultThread, NULL);


    pthread_mutex_destroy(&resultMutex);
    pthread_cond_destroy(&resultCond);

    if(partnerMode == MODE_STEREO_MASTER){
        M_DEBUG("Camera %s Master calling stop to slave\n", name);
        otherMgr->Stop();
        M_DEBUG("Camera %s Master finished stopping slave\n", name);
    }

    if(pVideoEncoderSmall) {
        pVideoEncoderSmall->Stop();
        delete pVideoEncoderSmall;
    }

    if(pVideoEncoderLarge) {
        pVideoEncoderLarge->Stop();
        delete pVideoEncoderLarge;
    }

    M_DEBUG("Deleting buffers for %s preview stream\n", name);
    bufferDeleteBuffers(pre_bufferGroup);
    M_DEBUG("Deleting buffers for %s small video stream\n", name);
    bufferDeleteBuffers(small_vid_bufferGroup);
    M_DEBUG("Deleting buffers for %s large video stream\n", name);
    bufferDeleteBuffers(large_vid_bufferGroup);
    M_DEBUG("Deleting buffers for %s snapshot stream\n", name);
    bufferDeleteBuffers(snap_bufferGroup);

#ifdef APQ8096
    // Stereo master on apq8096 seems to hang on "close" for some reason
    if(pDevice != NULL && partnerMode != MODE_STEREO_MASTER){
        pDevice->common.close(&pDevice->common);
        M_DEBUG("Done closing device for camera %s\n", name);
        pDevice = NULL;
    }
#else
    if(pDevice != NULL){
        pDevice->common.close(&pDevice->common);
        M_DEBUG("Done closing device for camera %s\n", name);
        pDevice = NULL;
    }
#endif

    if(pSessionParams != NULL){
        free_camera_metadata(pSessionParams);
    }

    pthread_mutex_destroy(&stereoMutex);
    pthread_cond_destroy(&stereoCond);

    pthread_mutex_destroy(&aeMutex);

    close_my_pipes();

    M_DEBUG("Done calling stop for camera %s\n", name);
    return;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Function that will process one capture result sent from the camera module. Remember this function is operating in the camera
// module thread context. So we do the bare minimum work that we need to do and return control back to the camera module. The
// bare minimum we do is to dispatch the work to another worker thread who consumes the image buffers passed to it from here.
// Our result worker thread is "ThreadPostProcessResult(..)"
// -----------------------------------------------------------------------------------------------------------------------------
void PerCameraMgr::ProcessOneCaptureResult(const camera3_capture_result* pHalResult)
{
    M_VERBOSE("Received %d buffers from camera %s, partial result:%d\n", pHalResult->num_output_buffers, name, pHalResult->partial_result);

    if(pHalResult->partial_result > 1){

        camera_image_metadata_t meta;
        meta.frame_id=pHalResult->frame_number;
        meta.framerate = configInfo.fps;

        M_VERBOSE("Received metadata for frame %d from camera %s\n", pHalResult->frame_number, name);

        int result = 0;
        camera_metadata_ro_entry entry;

        result = find_camera_metadata_ro_entry(pHalResult->result, ANDROID_SENSOR_TIMESTAMP, &entry);

        if (!result && entry.count)
        {
            meta.timestamp_ns = entry.data.i64[0];
            M_VERBOSE("\tTimestamp: %llu\n", meta.timestamp_ns);
        }

        result = find_camera_metadata_ro_entry(pHalResult->result, ANDROID_SENSOR_SENSITIVITY, &entry);

        if (!result && entry.count)
        {
            meta.gain = entry.data.i32[0];
            M_VERBOSE("\tGain: %d\n", meta.gain);
        }

        result = find_camera_metadata_ro_entry(pHalResult->result, ANDROID_SENSOR_EXPOSURE_TIME, &entry);

        if (!result && entry.count)
        {
            meta.exposure_ns = entry.data.i64[0];
            M_VERBOSE("\tExposure: %ld\n", meta.exposure_ns);
        }

        resultMetaRing.insert_data(meta);

        // check if there are result buffer returned before the meta data is avaliable
        image_result result_list[16]; // only 3 is needed
        int num = getAllResult(pHalResult->frame_number, result_list);
        if (num > 0 ) {
            M_VERBOSE("Find %d buffer in result ring for frame %d\n", num, pHalResult->frame_number);
            pthread_mutex_lock(&resultMutex);

            // Queue up work for the result thread "ThreadPostProcessResult"
            for (int i = 0; i < num; i++){
               resultMsgQueue.push(result_list[i]);
            }
            pthread_cond_signal(&resultCond);
            pthread_mutex_unlock(&resultMutex);
        }

    }


    for (uint i = 0; i < pHalResult->num_output_buffers; i++)
    {

        M_VERBOSE("Received output buffer %d from camera %s\n", pHalResult->frame_number, name);

        // Mutex is required for msgQueue access from here and from within the thread wherein it will be de-queued
        // if buffer arrive before meta data, save them in the ringbuffer
        image_result i_result = {pHalResult->frame_number, pHalResult->output_buffers[i]};
        camera_image_metadata_t imageInfo;
        if(getMeta(i_result.first, &imageInfo)) {
            M_VERBOSE("Buffer arrive before meta frame %d\n", i_result.first);
            resultMsgRing.insert_data(i_result);
        } else {
            pthread_mutex_lock(&resultMutex);

            // Queue up work for the result thread "ThreadPostProcessResult"
            resultMsgQueue.push(i_result);
            pthread_cond_signal(&resultCond);
            pthread_mutex_unlock(&resultMutex);
        }

    }

}

// -----------------------------------------------------------------------------------------------------------------------------
// Process the result from the camera module. Essentially handle the metadata and the image buffers that are sent back to us.
// We call the PerCameraMgr class function to handle it so that it can have access to any (non-static)class member data it needs
// Remember this function is operating in the camera module thread context. So we should do the bare minimum work and return.
// -----------------------------------------------------------------------------------------------------------------------------
void PerCameraMgr::CameraModuleCaptureResult(const camera3_callback_ops_t *cb, const camera3_capture_result* pHalResult)
{
    M_VERBOSE("Received result from HAl3 for frame number %d\n", pHalResult->frame_number);
    Camera3Callbacks* pCamera3Callbacks = (Camera3Callbacks*)cb;
    PerCameraMgr* pPerCameraMgr = (PerCameraMgr*)pCamera3Callbacks->pPrivate;

    pPerCameraMgr->ProcessOneCaptureResult(pHalResult);
}

// -----------------------------------------------------------------------------------------------------------------------------
// Handle any messages sent to us by the camera module
// -----------------------------------------------------------------------------------------------------------------------------
void PerCameraMgr::CameraModuleNotify(const camera3_callback_ops_t *cb, const camera3_notify_msg_t *msg)
{

    PerCameraMgr* pPerCameraMgr = (PerCameraMgr*)((Camera3Callbacks*)cb)->pPrivate;
    if(pPerCameraMgr->stopped) return;

    if (msg->type == CAMERA3_MSG_ERROR)
    {
        switch (msg->message.error.error_code) {

            case CAMERA3_MSG_ERROR_DEVICE:
                //Another thread has already detected the fatal error, return since it has already been handled
                if(pPerCameraMgr->EStopped) return;

                M_ERROR("Received \"Device\" error from camera: %s\n",
                             pPerCameraMgr->name);
                M_PRINT(  "                          Camera server will be stopped\n");
                EStopCameraServer();
                break;
            case CAMERA3_MSG_ERROR_REQUEST:
                M_ERROR("Received \"Request\" error from camera: %s\n",
                             pPerCameraMgr->name);
                break;
            case CAMERA3_MSG_ERROR_RESULT:
                M_ERROR("Received \"Result\" error from camera: %s\n",
                             pPerCameraMgr->name);
                break;
            case CAMERA3_MSG_ERROR_BUFFER:
                M_ERROR("Received \"Buffer\" error from camera: %s\n",
                             pPerCameraMgr->name);
                break;

            default:
                M_ERROR("Camera: %s Framenumber: %d ErrorCode: %d\n", pPerCameraMgr->name,
                       msg->message.error.frame_number, msg->message.error.error_code);
        }
    }
}



// this is used for preview, small, and large video streams
void PerCameraMgr::publishISPFrame(BufferBlock const& b, camera_image_metadata_t& meta, int32_t fmt,
                      int grey_pipe, int color_pipe)
{
    uint8_t const* first_plane_ptr;
    size_t const first_plane_size = b.width * b.height;

    // on apq, hal3 lies about stride -- don't actually need to remove
    // add some more platform specific code to handle this
    // handle it in conditionals -- compiler should be smart enough to remove branches, and it improves code clarity
    bool needs_remove_plane_stride = false;
#ifndef APQ8096
    // is10bit flag only set when using monochrome raw mode and driver happens
    // to give 10-bit, in which case we already converted to 8-bit flat before here
    if((b.width != b.stride)  && !is10bit){
        needs_remove_plane_stride = true;
    }
#endif

    if(needs_remove_plane_stride){
        first_plane_ptr = new uint8_t[first_plane_size];
        removePlaneStride(b.stride, b.width, b.height,
                          static_cast<uint8_t const*>(b.vaddress),
                          const_cast<uint8_t*>(first_plane_ptr));
    }
    else{
        first_plane_ptr = static_cast<uint8_t const*>(b.vaddress);
    }

    meta.width  = static_cast<int16_t>(b.width);
    meta.stride = static_cast<int32_t>(b.width);
    meta.height = static_cast<int16_t>(b.height);
    meta.format = IMAGE_FORMAT_RAW8;
    meta.size_bytes = meta.width * meta.height;
    meta.magic_number = CAMERA_MAGIC_NUMBER;
    pipe_server_write_camera_frame(grey_pipe, meta, first_plane_ptr);

    // also send to color pipe if color camera
    if(fmt == HAL3_FMT_YUV){
        uint8_t const* uv_plane_ptr;
        size_t const uv_plane_size = first_plane_size / 2;
        if(needs_remove_plane_stride){
            uv_plane_ptr = new uint8_t[uv_plane_size];
            removePlaneStride(b.stride, b.width, b.height / 2,
                              static_cast<uint8_t const*>(b.uvHead),
                              const_cast<uint8_t*>(uv_plane_ptr));
        }
        else{
            uv_plane_ptr = static_cast<uint8_t const*>(b.uvHead);
        }

        meta.format = IMAGE_FORMAT_NV12;
        meta.size_bytes = static_cast<int32_t>(first_plane_size + uv_plane_size);
        const void* buffers[3] = { &meta, first_plane_ptr, uv_plane_ptr };
        size_t lengths[3] = { sizeof(camera_image_metadata_t), first_plane_size, uv_plane_size };
        pipe_server_write_list(color_pipe, 3, buffers, lengths);

        if(needs_remove_plane_stride){
            delete[] uv_plane_ptr;
        }
    }

    if(needs_remove_plane_stride){
        delete[] first_plane_ptr;
    }
}


void PerCameraMgr::ProcessTOFPreviewFrame(BufferBlock* bufferBlockInfo, camera_image_metadata_t meta)
{
    tofFrameCounter++;

    if(grab_cpu_standby_active() && tofFrameCounter % (int)configInfo.decimator != 0){
        return;
    }
    #ifdef APQ8096
        TOFProcessRAW16(tof_interface,
                    (uint16_t *)bufferBlockInfo->vaddress,
                    meta.timestamp_ns);
    #elif QRB5165

        uint16_t srcPixel16[pre_width * pre_height] = {0};
        // NOTE we don't actually puvblish tis particular metadata to the pipe
        // TOF data is published separately in a very different way to cameras
        meta.format     = IMAGE_FORMAT_RAW8;
        meta.size_bytes = pre_width * pre_height;
        meta.stride     = pre_width;

        // TODO: instead of creating new array you can do this raw12->raw16 cvt in place
        Mipi12ToRaw16(meta.size_bytes, (uint8_t *)bufferBlockInfo->vaddress, srcPixel16);
        tof_interface->ProcessRAW16(srcPixel16, meta.timestamp_ns);
    #endif
    M_VERBOSE("Sent tof data to royale for processing\n");

    return;
}

void PerCameraMgr::publishRAWPreviewFrame(BufferBlock* bufferBlockInfo, camera_image_metadata_t meta)
{
    M_VERBOSE("publishing raw preview frame for mono camera %s\n", name);

    meta.format     = IMAGE_FORMAT_RAW8;
    meta.size_bytes = pre_width*pre_height;
    meta.stride     = pre_width;

    // TODO check if we need to remove plane stride here

    // if the bayer pipe is set up, we must be a color raw sensor, write the raw bayer data
    if(previewPipeBayer >= 0){
        pipe_server_write_camera_frame(previewPipeBayer, meta, (uint8_t*)bufferBlockInfo->vaddress);
    }

    // all pipes have a grey channel, publish that.
    pipe_server_write_camera_frame(previewPipeGrey, meta, previewFrameGrey8);


    // only debayer to color if there are clients
    if(previewPipeColor>=0 && pipe_server_get_num_clients(previewPipeColor)>0)
    {
        if( (configInfo.bayer_fmt == BAYER_BGGR && configInfo.en_rotate==0) ||\
            (configInfo.bayer_fmt == BAYER_RGGB && configInfo.en_rotate!=0))
        {
            debayer_BGGR8_to_rgb8((uint8_t*)bufferBlockInfo->vaddress, previewFrameRGB8, pre_width, pre_height);
        }else{
            debayer_RGGB8_to_rgb8((uint8_t*)bufferBlockInfo->vaddress, previewFrameRGB8, pre_width, pre_height);
        }

        meta.format = IMAGE_FORMAT_RGB;
        meta.size_bytes = pre_width*pre_height*3;
        pipe_server_write_camera_frame(previewPipeColor, meta, previewFrameRGB8);
    }

    return;
}



void PerCameraMgr::publishStereoRAWPreviewFrame(camera_image_metadata_t meta)
{
    M_VERBOSE("publishing raw preview frame for stereo camera %s\n", name);

    // TODO check if we need to remove plane stride here

    // raw cameras only publish a grey debayered image pair for DFS or VIO
    size_t ylen = pre_width*pre_height;
    const void* bufs[] = {&meta, previewFrameGrey8, childFrame};
    size_t lens[] = {sizeof(camera_image_metadata_t), ylen, ylen};
    meta.format = IMAGE_FORMAT_STEREO_RAW8;
    meta.size_bytes = ylen*2;
    meta.stride     = pre_width;
    pipe_server_write_list(previewPipeGrey, 3, bufs, lens);

    return;
}


// this is used for preview, small, and large video streams
void PerCameraMgr::publishStereoISPFrame(BufferBlock* bufferBlockInfo, camera_image_metadata_t meta)
{
    // these are lengths to be written to a pipe which require stride=width
    size_t ylen =  bufferBlockInfo->width * bufferBlockInfo->height;
    size_t uvlen = ylen/2;

    // write the Y data out to grey pipe for both
    const void* bufs[] = {&meta, bufferBlockInfo->vaddress, childFrame};
    size_t lens[] = {sizeof(camera_image_metadata_t), ylen, ylen};
    meta.format = IMAGE_FORMAT_STEREO_RAW8;
    meta.size_bytes = 2*ylen;
    pipe_server_write_list(previewPipeGrey, 3, bufs, lens);

    // for color cameras also write UV
    if(previewPipeColor>=0 && pipe_server_get_num_clients(previewPipeColor)>0)
    {
        const void* bufs[] = {  &meta,
                                bufferBlockInfo->vaddress,
                                bufferBlockInfo->uvHead,
                                childFrame,
                                childFrame_uvHead};
        size_t lens[] = {sizeof(camera_image_metadata_t), ylen, uvlen, ylen, uvlen};
        meta.format = IMAGE_FORMAT_STEREO_NV12;
        meta.size_bytes = 2*(ylen+uvlen);
        pipe_server_write_list(previewPipeColor, 5, bufs, lens);
    }

    return;
}


void PerCameraMgr::runModalExposure(uint8_t* grey_ptr, camera_image_metadata_t meta)
{
    if(ae_mode != AE_LME_HIST && \
        ae_mode != AE_LME_MSV){
        return;
    }

    int64_t    new_exposure_ns;
    int32_t    new_gain;

    pthread_mutex_lock(&aeMutex);

    if(ae_mode == AE_LME_HIST && expHistInterface.update_exposure(
            grey_ptr,
            pre_width,
            pre_height,
            meta.exposure_ns,
            meta.gain,
            &new_exposure_ns,
            &new_gain))
    {
        setExposure = new_exposure_ns;
        setGain     = new_gain;
    }

    if(ae_mode == AE_LME_MSV && expMSVInterface.update_exposure(
            grey_ptr,
            pre_width,
            pre_height,
            meta.exposure_ns,
            meta.gain,
            &new_exposure_ns,
            &new_gain))
    {
        setExposure = new_exposure_ns;
        setGain     = new_gain;
    }
    pthread_mutex_unlock(&aeMutex);

    return;
}


// return -1 to indicate that ProcessPreviewFrame should NOT continue
int PerCameraMgr::handleStereoSync(camera_image_metadata_t* meta)
{
    NEED_CHILD:
    pthread_mutex_lock(&stereoMutex);
    if(childFrame == NULL){
        pthread_cond_wait(&stereoCond, &stereoMutex);
    }

    if(EStopped | stopped) {
        pthread_cond_signal(&(otherMgr->stereoCond));
        pthread_mutex_unlock(&stereoMutex);
        return -1;
    }

    if(childFrame == NULL){
        pthread_mutex_unlock(&stereoMutex);
        M_WARN("Child frame not received, assuming missing and discarding master\n");
        return -1;
    }

    int64_t diff = meta->timestamp_ns - childInfo.timestamp_ns;
    M_VERBOSE("----- STEREO TIMING: %s timestamps(ms): %llu, %llu, diff: %0.1fms\n",
        name,
        meta->timestamp_ns/1000000,
        childInfo.timestamp_ns/1000000,
        diff/1000000.0);

    //Much newer master, discard the child and get a new one
    if(diff > MAX_STEREO_DISCREPENCY_NS){
        M_WARN("Camera %s Received much newer master than child (%0.1fms), discarding child and trying again\n",
            name, diff/1000000.0);
        childFrame = NULL;
        pthread_mutex_unlock(&stereoMutex);
        pthread_cond_signal(&(otherMgr->stereoCond));
        goto NEED_CHILD;
    }

    diff *= -1;
    //Much newer child, discard master but keep the child
    if(diff > MAX_STEREO_DISCREPENCY_NS){
        M_WARN("Camera %s Received much newer child than master (%0.1fms), discarding master and trying again\n",
            name, diff/1000000.0);
        pthread_mutex_unlock(&stereoMutex);
        return -1;
    }

    // Assume the earlier timestamp is correct
    if(meta->timestamp_ns > childInfo.timestamp_ns){
        meta->timestamp_ns = childInfo.timestamp_ns;
    }


    return 0;
}



void PerCameraMgr::ProcessPreviewFrame(image_result result)
{
    if(!has_set_preview_thread_affinity){
        M_DEBUG("setting thread affinity for cam %s preview\n", name);
        lock_this_thread_to_big_cores();
        has_set_preview_thread_affinity = 1;
    }


    BufferBlock* bufferBlockInfo = bufferGetBufferInfo(&pre_bufferGroup, result.second.buffer);

    // fetch metadata with timestamp that should have come from hal3 via callback already
    camera_image_metadata_t meta;
    if(getMeta(result.first, &meta)) {
        M_WARN("Trying to process encode buffer without metadata\n");
        return;
    }
    initMetaFromBlock(*bufferBlockInfo, meta);

    // Tof is different from the rest, pass the data off to spectre then send it out
    if(configInfo.type == SENSOR_TOF) {
        ProcessTOFPreviewFrame(bufferBlockInfo, meta);
        return;
    }

    // okay, at this point we know the camera is not tof, might be raw mono,
    // raw bayer, or ISP YUV image

    // if raw, might need to convert from 8 to 10 bit regardless if we are
    // a mono or a stereo camera, so do that here
    if(configInfo.en_raw_preview && meta.frame_id == 1){
        M_DEBUG("%s checking to see if it is raw8 or raw 10\n", name);

        is10bit = Check10bit((uint8_t*)bufferBlockInfo->vaddress, pre_width, pre_height);
        if(is10bit){
            M_PRINT("Received RAW10 frame from camera %s, will be converting to RAW8 on cpu\n", name);
        } else {
            M_DEBUG("Frame was actually 8 bit, sending as is\n");
        }
    }

    // always debayer to mono, this is the most common use case and also
    // needed for autoexposure. TODO this might not be necessary for
    // hires color cams in raw mode. Need to do autoexposure on a color
    // image in that case though. Later need to see if we need to debayer
    // those cameras to YUV or if RGB is okay
    if(configInfo.en_raw_preview){
        if(is10bit){
            ConvertTo8bitRaw((uint8_t*)bufferBlockInfo->vaddress, pre_width, pre_height);
        }
        if(configInfo.bayer_fmt != BAYER_MONO){
            debayer_BGGR8_or_RGGB8_to_mono((uint8_t*)bufferBlockInfo->vaddress, previewFrameGrey8, pre_width, pre_height);
        }
        else{
            previewFrameGrey8 = (uint8_t*)bufferBlockInfo->vaddress;
        }
    }
    else{
        // for most frames, the grey data is just the start of the image buffer
        // when it's not, this will be changed later
        previewFrameGrey8 = (uint8_t*)bufferBlockInfo->vaddress;
    }


    // mono camera is most simple, publish to pipe based on raw or ISP and run AE
    if(partnerMode == MODE_MONO){
        if(configInfo.en_raw_preview){
            publishRAWPreviewFrame(bufferBlockInfo, meta);
        }else{
            publishISPFrame(*bufferBlockInfo, meta, pre_halfmt, previewPipeGrey, previewPipeColor);
        }
        runModalExposure(previewFrameGrey8, meta);
    }


    // stereo slave is fairly simple too, debayer if necessary then indicate
    // to the master that we have a new frame
    else if (partnerMode == MODE_STEREO_SLAVE)
    {
        // lock mutex before modifying the master
        pthread_mutex_lock(&(otherMgr->stereoMutex));

        // point the master to our new image
        otherMgr->childFrame = previewFrameGrey8;
        otherMgr->childInfo  = meta;

        if(configInfo.en_raw_preview){
            otherMgr->childFrame_uvHead = NULL; // no yuv mode for raw images
        }else{
            otherMgr->childFrame_uvHead = (uint8_t*)bufferBlockInfo->uvHead;
        }

        // signal to the master that we have a new frame
        pthread_cond_signal(&(otherMgr->stereoCond));
        pthread_cond_wait(&stereoCond, &(otherMgr->stereoMutex));
        pthread_mutex_unlock(&(otherMgr->stereoMutex));

        // slave runs autoexposure in independent exposure mode only
        if(configInfo.ind_exp){
            runModalExposure(previewFrameGrey8, meta);
        }
    }


    // master is similar to mono mode, except we need to do the stereo sync
    // logic first, then publish two frames with some extra logic
    else if (partnerMode == MODE_STEREO_MASTER)
    {
        // do the stereo syncing duties
        if(handleStereoSync(&meta)) return;

        if(configInfo.en_raw_preview){
            publishStereoRAWPreviewFrame(meta);
            runModalExposure(previewFrameGrey8, meta);
        }else{
            publishStereoISPFrame(bufferBlockInfo, meta);
            runModalExposure((uint8_t*)bufferBlockInfo->vaddress, meta);
        }

        //Pass back the new AE values to the other camera
        if(!configInfo.ind_exp) {
            otherMgr->setExposure = setExposure;
            otherMgr->setGain = setGain;
        }

        //Clear the pointers and signal the child thread for cleanup
        childFrame = NULL;
        pthread_mutex_unlock(&stereoMutex);
        pthread_cond_signal(&(otherMgr->stereoCond));
    }


    else{
        M_ERROR("UNKNOWN partnerMode\n");
    }

    //M_ERROR("1 %s \n", name);
    return;
} // end of PerCameraMgr::ProcessPreviewFrame


void PerCameraMgr::ProcessSmallVideoFrame(image_result result)
{
    BufferBlock* bufferBlockInfo = bufferGetBufferInfo(&small_vid_bufferGroup, result.second.buffer);

    camera_image_metadata_t meta;
    if(getMeta(result.first, &meta)) {
        M_WARN("Trying to process encode buffer without metadata\n");
        bufferPush(small_vid_bufferGroup, result.second.buffer);
        return;
    }

    initMetaFromBlock(*bufferBlockInfo, meta);
    publishISPFrame(*bufferBlockInfo, meta, vid_halfmt, smallVideoPipeGrey,       smallVideoPipeColor);

    // no need to pass data to OMX if there are no h264 clients
    if(pipe_server_get_num_clients(smallVideoPipeEncoded)<1){
        bufferPush(small_vid_bufferGroup, result.second.buffer);
        return;
    }
    
    // check health of the encoder and drop this frame if it's getting backed up
    int n = pVideoEncoderSmall->ItemsInQueue();
    if(n>SMALL_VID_ALLOWED_ITEMS_IN_OMX_QUEUE){
        M_DEBUG("dropping small video frame, OMX is getting backed up, has %d in queue already\n", n);
        bufferPush(small_vid_bufferGroup, result.second.buffer);
        return;
    }
    // add to the OMX queue
    pVideoEncoderSmall->ProcessFrameToEncode(meta, bufferBlockInfo);

}

void PerCameraMgr::ProcessLargeVideoFrame(image_result result)
{
    BufferBlock* bufferBlockInfo = bufferGetBufferInfo(&large_vid_bufferGroup, result.second.buffer);

    camera_image_metadata_t meta;
    if(getMeta(result.first, &meta)) {
        M_WARN("Trying to process encode buffer without metadata\n");
        bufferPush(large_vid_bufferGroup, result.second.buffer);
        return;
    }

    initMetaFromBlock(*bufferBlockInfo, meta);
    publishISPFrame(*bufferBlockInfo, meta, vid_halfmt, largeVideoPipeGrey, largeVideoPipeColor);

    // no need to pass data to OMX if there are no h264 clients
    if(pipe_server_get_num_clients(largeVideoPipeEncoded)<1){
        bufferPush(large_vid_bufferGroup, result.second.buffer);
        return;
    }

    // check health of the encoder and drop this frame if it's getting backed up
    int n = pVideoEncoderLarge->ItemsInQueue();
    if(n>LARGE_VID_ALLOWED_ITEMS_IN_OMX_QUEUE){
        M_DEBUG("dropping large video frame, OMX is getting backed up, has %d in queue already\n", n);
        bufferPush(large_vid_bufferGroup, result.second.buffer);
        return;
    }
    // add to the OMX queue
    pVideoEncoderLarge->ProcessFrameToEncode(meta, bufferBlockInfo);

}

#ifndef APQ8096
/* Get an existing tag, or create one if it doesn't exist */
static ExifEntry *init_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
	ExifEntry *entry;
	/* Return an existing tag if one exists */
	if (!((entry = exif_content_get_entry (exif->ifd[ifd], tag)))) {
	    /* Allocate a new entry */
	    entry = exif_entry_new ();
	    assert(entry != NULL); /* catch an out of memory condition */
	    entry->tag = tag; /* tag must be set before calling
				 exif_content_add_entry */

	    /* Attach the ExifEntry to an IFD */
	    exif_content_add_entry (exif->ifd[ifd], entry);

	    /* Allocate memory for the entry and fill with default data */
	    exif_entry_initialize (entry, tag);

	    /* Ownership of the ExifEntry has now been passed to the IFD.
	     * One must be very careful in accessing a structure after
	     * unref'ing it; in this case, we know "entry" won't be freed
	     * because the reference count was bumped when it was added to
	     * the IFD.
	     */
	    exif_entry_unref(entry);
	}
	return entry;
}

static ExifEntry *create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag, size_t len)
{
	void *buf;
	ExifEntry *entry;
	
	/* Create a memory allocator to manage this ExifEntry */
	ExifMem *mem = exif_mem_new_default();
	assert(mem != NULL); /* catch an out of memory condition */

	/* Create a new ExifEntry using our allocator */
	entry = exif_entry_new_mem (mem);
	assert(entry != NULL);

	/* Allocate memory to use for holding the tag data */
	buf = exif_mem_alloc(mem, len);
	assert(buf != NULL);

	/* Fill in the entry */
	entry->data = (unsigned char*)buf;
	entry->size = len;
	entry->tag = tag;
	entry->components = len;
	entry->format = EXIF_FORMAT_UNDEFINED;

	/* Attach the ExifEntry to an IFD */
	exif_content_add_entry (exif->ifd[ifd], entry);

	/* The ExifMem and ExifEntry are now owned elsewhere */
	exif_mem_unref(mem);
	exif_entry_unref(entry);

	return entry;
}
#endif

void PerCameraMgr::ProcessSnapshotFrame(image_result result)
{
    BufferBlock* bufferBlockInfo = bufferGetBufferInfo(&snap_bufferGroup, result.second.buffer);

    // first write to pipe if subscribed
    camera_image_metadata_t meta;
    if(getMeta(result.first, &meta)) {
        M_WARN("Trying to process encode buffer without metadata\n");
        return;
    }

    int start_index = 0;
    uint8_t* src_data = (uint8_t*)bufferBlockInfo->vaddress;
    int extractJpgSize = find_jpeg_buffer_size(src_data, bufferBlockInfo->size, &start_index);

    if(extractJpgSize == 1){
        M_ERROR("Real Size of JPEG is incorrect");
        return;
    }

    M_DEBUG("Snapshot jpeg start: %6d len %8d\n", start_index, extractJpgSize);

#ifndef APQ8096
    // Load the EXIF data from the file
    unsigned char *exif_data;
    unsigned int exif_data_len;
    ExifEntry *entry;
   
    ExifData* exif = exif_data_new_from_data(src_data + start_index, extractJpgSize);
    if (exif == nullptr) {
        printf("Issue getting exif data from origin\n");
        return;
    }

    gps_data_t gps_grabbed_info = grab_gps_info();

    // Code to add latitude to exif tag
    entry = create_tag(exif, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LATITUDE_REF, 2);
    entry->format = EXIF_FORMAT_ASCII;
    entry->components = 1;
    if (gps_grabbed_info.latitude >= 0) {
        memcpy(entry->data, "N", sizeof("N"));
    } else {
        memcpy(entry->data, "S", sizeof("S"));
        gps_grabbed_info.latitude *= -1;
    }

    entry = create_tag(exif, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LATITUDE, 24);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components = 3;

    ExifLong degrees_lat = (ExifLong)gps_grabbed_info.latitude;
    ExifLong minutes_lat = (ExifLong)(60 * (gps_grabbed_info.latitude - degrees_lat));
    ExifLong microseconds_lat = (ExifLong)(3600000000u * (gps_grabbed_info.latitude - degrees_lat - minutes_lat / 60.0));
    exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, (ExifRational){degrees_lat, 1});
    exif_set_rational(entry->data + sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, (ExifRational){minutes_lat, 1});
    exif_set_rational(entry->data + 2 * sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL,
            (ExifRational){microseconds_lat, 1000000});

    // Code to add longitude to exif tag
    entry = create_tag(exif, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LONGITUDE_REF, 2);
    entry->format = EXIF_FORMAT_ASCII;
    entry->components = 1;
    if (gps_grabbed_info.longitude >= 0) {
        memcpy(entry->data, "E", sizeof("E"));
    } else {
        memcpy(entry->data, "W", sizeof("W"));
        gps_grabbed_info.longitude *= -1;
    }

    entry = create_tag(exif, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LONGITUDE, 24);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components = 3;

    ExifLong degrees_lon = (ExifLong)gps_grabbed_info.longitude;
    ExifLong minutes_lon = (ExifLong)(60 * (gps_grabbed_info.longitude - degrees_lon));
    ExifLong microseconds_lon = (ExifLong)(3600000000u * (gps_grabbed_info.longitude - degrees_lon - minutes_lon / 60.0));
    exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, (ExifRational){degrees_lon, 1});
    exif_set_rational(entry->data + sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, (ExifRational){minutes_lon, 1});
    exif_set_rational(entry->data + 2 * sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL,
            (ExifRational){microseconds_lon, 1000000}); 

    // Code to add altitude to exif tag
    entry = create_tag(exif, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_ALTITUDE, 24);
    entry->format = EXIF_FORMAT_RATIONAL;
    entry->components = 1;

    ExifSLong alt_lon = (ExifSLong)gps_grabbed_info.altitude;
    unsigned int tmp = alt_lon * 1000;
    exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, (ExifRational){tmp, 1000});
    
    /* Get a pointer to the EXIF data block we just created */
    exif_data_fix(exif);
    exif_data_save_data(exif, &exif_data, &exif_data_len);

    assert(exif_data != NULL);
#endif

    meta.magic_number = CAMERA_MAGIC_NUMBER;
    meta.width        = snap_width;
    meta.height       = snap_height;
    meta.format       = IMAGE_FORMAT_JPG;
    meta.size_bytes   = extractJpgSize;
    pipe_server_write_camera_frame(snapshotPipe, meta, &src_data[start_index]);

    // now, if there is a filename in the queue, write it too
    if(snapshotQueue.size() != 0){
        char *filename = snapshotQueue.front();
        snapshotQueue.pop();

        M_PRINT("Camera: %s writing snapshot to :\"%s\"\n", name, filename);
        //WriteSnapshot(bufferBlockInfo, snap_halfmt, filename);

        FILE* file_descriptor = fopen(filename, "wb");
        if(! file_descriptor){

            //Check to see if we were just missing parent directories
            CreateParentDirs(filename);

            file_descriptor = fopen(filename, "wb");

            if(! file_descriptor){
                M_ERROR("failed to open file descriptor for snapshot save to: %s\n", filename);
                return;
            }
        }

#ifndef APQ8096
        /* Write EXIF header */
        if (fwrite(exif_header, exif_header_len, 1, file_descriptor) != 1) {
            fprintf(stderr, "Error writing to file inin exif header %s\n", filename);
        }
        /* Write EXIF block length in big-endian order */
        if (fputc((exif_data_len+2) >> 8, file_descriptor) < 0) {
            fprintf(stderr, "Error writing to file in big endian order %s\n", filename);
        }
        if (fputc((exif_data_len+2) & 0xff, file_descriptor) < 0) {
            fprintf(stderr, "Error writing to file with fputc %s\n", filename);
        }
        /* Write EXIF data block */
        if (fwrite(exif_data, exif_data_len, 1, file_descriptor) != 1) {
            fprintf(stderr, "Error writing to file with data block %s\n", filename);
        }

        // /* Write JPEG image data, skipping the non-EXIF header */
        if (fwrite(src_data + start_index + 2, extractJpgSize - 2, 1, file_descriptor) != 1) {
            fprintf(stderr, "Error writing to file with jpeg %s\n", filename);
        }

        free(exif_data);
        exif_data_unref(exif);
#else 
        int ret = fwrite(&src_data[start_index], extractJpgSize, 1, file_descriptor);
        if(ret!=1){
            M_ERROR("snapshot failed to write to disk\n");
        }
#endif

        fclose(file_descriptor);
        free(filename);
    }
    else{
        M_VERBOSE("wrote snapshot to pipe but not to disk\n");
    }
}

// -----------------------------------------------------------------------------------------------------------------------------
// The TOF library calls this function when it receives data from the Royale PMD libs
// -----------------------------------------------------------------------------------------------------------------------------
// Callback function for the TOF bridge to provide the post processed TOF data
bool PerCameraMgr::RoyaleDataDone(const void*             pData,
                                  uint32_t                size,
                                  int64_t                 timestamp,
                                  RoyaleListenerType      dataType)
{

    M_VERBOSE("Received royale data for camera: %s\n", name);

    constexpr int MAX_IR_VALUE_IN  = 2895;
    constexpr int MAX_IR_VALUE_OUT = (1<<8);

    const royale::DepthData* pDepthData               = static_cast<const royale::DepthData *> (pData);
    const royale::Vector<royale::DepthPoint>& pointIn = pDepthData->points;
    int numPoints = (int)pointIn.size();

    camera_image_metadata_t IRMeta, DepthMeta, ConfMeta;
    point_cloud_metadata_t PCMeta;

    // set up some common metadata
    IRMeta.timestamp_ns = pDepthData->timeStamp.count();
    IRMeta.gain         = 0;
    IRMeta.exposure_ns  = 0;
    IRMeta.frame_id     = ++TOFFrameNumber;
    IRMeta.width        = pDepthData->width;
    IRMeta.height       = pDepthData->height;
    DepthMeta = IRMeta;
    ConfMeta  = IRMeta;

    if(pipe_server_get_num_clients(tofPipeIR)>0){
        IRMeta.stride         = IRMeta.width * sizeof(uint8_t);
        IRMeta.size_bytes     = IRMeta.stride * IRMeta.height;
        IRMeta.format         = IMAGE_FORMAT_RAW8;
        uint8_t IRData[numPoints];
        for (int i = 0; i < numPoints; i++)
        {
            royale::DepthPoint point = pointIn[i];
            uint32_t longval = point.grayValue;
            longval *= MAX_IR_VALUE_OUT;
            longval /= MAX_IR_VALUE_IN;
            IRData[i]    = longval;
        }
        pipe_server_write_camera_frame(tofPipeIR, IRMeta, IRData);
    }

    if(pipe_server_get_num_clients(tofPipeDepth)>0){
        DepthMeta.stride      = DepthMeta.width * sizeof(uint8_t);
        DepthMeta.size_bytes  = DepthMeta.stride * DepthMeta.height;
        DepthMeta.format      = IMAGE_FORMAT_RAW8;
        uint8_t DepthData[numPoints];
        for (int i = 0; i < numPoints; i++)
        {
            DepthData[i] = (uint8_t)((pointIn[i].z / 5) * 255);
        }
        pipe_server_write_camera_frame(tofPipeDepth, DepthMeta, DepthData);
    }

    if(pipe_server_get_num_clients(tofPipeConf)>0){
        ConfMeta.stride       = ConfMeta.width * sizeof(uint8_t);
        ConfMeta.size_bytes   = ConfMeta.stride * ConfMeta.height;
        ConfMeta.format       = IMAGE_FORMAT_RAW8;
        uint8_t ConfData[numPoints];
        for (int i = 0; i < numPoints; i++)
        {
            royale::DepthPoint point = pointIn[i];
            ConfData[i] = point.depthConfidence;
        }
        pipe_server_write_camera_frame(tofPipeConf, ConfMeta, ConfData);
    }

    if(pipe_server_get_num_clients(tofPipePC)>0){
        PCMeta.timestamp_ns   = IRMeta.timestamp_ns;
        PCMeta.n_points       = numPoints;
        float PointCloud[numPoints*3];
        for (int i = 0; i < numPoints; i++)
        {
            royale::DepthPoint point = pointIn[i];
            PointCloud[(i*3)]   = point.x;
            PointCloud[(i*3)+1] = point.y;
            PointCloud[(i*3)+2] = point.z;
        }
        pipe_server_write_point_cloud(tofPipePC, PCMeta, PointCloud);
    }

    if(pipe_server_get_num_clients(tofPipeFull)>0){
        tof_data_t FullData;
        FullData.magic_number = TOF_MAGIC_NUMBER;
        FullData.timestamp_ns = IRMeta.timestamp_ns;
        for (int i = 0; i < numPoints; i++)
        {
            royale::DepthPoint point = pointIn[i];
            FullData.points     [i][0] = point.x;
            FullData.points     [i][1] = point.y;
            FullData.points     [i][2] = point.z;
            FullData.noises     [i]    = point.noise;
            uint32_t longval = point.grayValue;
            longval *= MAX_IR_VALUE_OUT;
            longval /= MAX_IR_VALUE_IN;
            FullData.grayValues [i]    = longval;
            FullData.confidences[i]    = point.depthConfidence;
        }
        pipe_server_write(tofPipeFull, (const char *)(&FullData), sizeof(tof_data_t));
    }

    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------
// PerCameraMgr::CameraModuleCaptureResult(..) is the entry callback that is registered with the camera module to be called when
// the camera module has frame result available to be processed by this application. We do not want to do much processing in
// that function since it is being called in the context of the camera module. So we do the bare minimum processing and leave
// the remaining process upto this function. PerCameraMgr::CameraModuleCaptureResult(..) just pushes a message in a queue that
// is monitored by this thread function. This function goes through the message queue and processes all the messages in it. The
// messages are nothing but camera images that this application has received.
// -----------------------------------------------------------------------------------------------------------------------------
void* PerCameraMgr::ThreadPostProcessResult()
{
    { // Configuration, these variables don't need to persist
        char buf[16];
        pid_t tid = syscall(SYS_gettid);
        sprintf(buf, "cam%d-result", cameraId);
        pthread_setname_np(pthread_self(), buf);
        M_VERBOSE("Entered thread: %s(tid: %lu)\n", buf, tid);

        // Set thread priority
        int which = PRIO_PROCESS;
        int nice  = -10;
        setpriority(which, tid, nice);
    }

    uint8_t num_finished_streams = en_snapshot;

    // The condition of the while loop is such that this thread will not terminate till it receives the last expected image
    // frame from the camera module or detects the ESTOP flag
    while (!EStopped && num_finished_streams != num_streams)
    {
        pthread_mutex_lock(&resultMutex);
        if (resultMsgQueue.empty())
        {
            //Wait for a signal that we have Received a frame or an estop
            pthread_cond_wait(&resultCond, &resultMutex);
        }

        if(EStopped || stopped) {
            pthread_mutex_unlock(&resultMutex);
            break;
        }

        if (resultMsgQueue.empty()) {
            pthread_mutex_unlock(&resultMutex);
            continue;
        }

        image_result result = resultMsgQueue.front();
        resultMsgQueue.pop();
        pthread_mutex_unlock(&resultMutex);

        buffer_handle_t  *handle      = result.second.buffer;
        camera3_stream_t *stream      = result.second.stream;
        BufferGroup      *bufferGroup = GetBufferGroup(stream);


        // Coming here means we have a result frame to process
        M_VERBOSE("%s procesing new buffer\n", name);

        switch (GetStreamId(stream)){
            case STREAM_PREVIEW:
                M_VERBOSE("Camera: %s processing preview frame\n", name);
                ProcessPreviewFrame(result);
                bufferPush(*bufferGroup, handle); // This queues up the buffer for recycling
                break;

            case STREAM_SMALL_VID: // Not Ready
                M_VERBOSE("Camera: %s processing small vid frame\n", name);
                ProcessSmallVideoFrame(result);
                break;

            case STREAM_LARGE_VID: // Not Ready
                M_VERBOSE("Camera: %s processing large vid frame\n", name);
                ProcessLargeVideoFrame(result);
                break;

            case STREAM_SNAPSHOT:
                M_VERBOSE("Camera: %s processing snapshot frame\n", name);
                ProcessSnapshotFrame(result);
                bufferPush(*bufferGroup, handle); // This queues up the buffer for recycling
                break;

            default:
                M_ERROR("Camera: %s Received frame for unknown stream\n", name);
                bufferPush(*bufferGroup, handle); // This queues up the buffer for recycling
                break;
        }

        if (lastResultFrameNumber == result.first){
            num_finished_streams++;
        }



    }

    if(EStopped){
        M_WARN("Thread: %s result thread Received ESTOP\n", name);
    }else{
        M_DEBUG("------ Last %s result frame: %d\n", name, lastResultFrameNumber);
    }

    M_VERBOSE("Leaving %s result thread\n", name);

    return NULL;
}


int PerCameraMgr::HasClientForPreviewFrame()
{
    if(configInfo.type == SENSOR_TOF){
        if(pipe_server_get_num_clients(tofPipeIR   )>0) return 1;
        if(pipe_server_get_num_clients(tofPipeDepth)>0) return 1;
        if(pipe_server_get_num_clients(tofPipeConf )>0) return 1;
        if(pipe_server_get_num_clients(tofPipePC   )>0) return 1;
        if(pipe_server_get_num_clients(tofPipeFull )>0) return 1;
    }
    else if(partnerMode == MODE_STEREO_SLAVE){
        if(pipe_server_get_num_clients(otherMgr->previewPipeGrey)>0) return 1;
        if(otherMgr->previewPipeColor>=0 && pipe_server_get_num_clients(otherMgr->previewPipeColor)>0) return 1;
    }
    else{
        if(pipe_server_get_num_clients(previewPipeGrey)>0) return 1;
        if(previewPipeColor>=0 && pipe_server_get_num_clients(previewPipeColor)>0) return 1;
        if(previewPipeBayer>=0 && pipe_server_get_num_clients(previewPipeBayer)>0) return 1;
    }
    return 0;
}


int PerCameraMgr::HasClientForSmallVideo()
{
    if(pipe_server_get_num_clients(smallVideoPipeGrey  )>0) return 1;
    if(pipe_server_get_num_clients(smallVideoPipeColor )>0) return 1;
    if(pipe_server_get_num_clients(smallVideoPipeEncoded  )>0) return 1;
    return 0;
}

int PerCameraMgr::HasClientForLargeVideo()
{
    if(pipe_server_get_num_clients(largeVideoPipeGrey  )>0) return 1;
    if(pipe_server_get_num_clients(largeVideoPipeColor )>0) return 1;
    if(pipe_server_get_num_clients(largeVideoPipeEncoded  )>0) return 1;
    return 0;
}


// -----------------------------------------------------------------------------------------------------------------------------
// Send one capture request to the camera module
// -----------------------------------------------------------------------------------------------------------------------------
int PerCameraMgr::SendOneCaptureRequest(uint32_t* frameNumber)
{
    camera3_capture_request_t request;

    if(ae_mode != AE_ISP){
        M_VERBOSE("setting hal3 exposure for %s  %10ld %5d\n", name, setExposure, setGain);
        requestMetadata.update(ANDROID_SENSOR_EXPOSURE_TIME, &setExposure, 1);
        requestMetadata.update(ANDROID_SENSOR_SENSITIVITY,   &setGain, 1);
    }

    std::vector<camera3_stream_buffer_t> streamBufferList;
    request.num_output_buffers  = 0;

    // TODO may want to send stream requests to keep AE going for the case where
    // the user is just taking snapshots and not streaming video. This should
    // be a config file option though so we don't waste power if the user is
    // not taking snapshots
    if(en_small_video && HasClientForSmallVideo()){

        int nFree = bufferNumFree(small_vid_bufferGroup);
        if(nFree<1){
            M_WARN("small vid stream buffer pool for Cam(%s), Frame(%d) has %d free, skipping request\n", name, *frameNumber, nFree);
        }
        else{

            camera3_stream_buffer_t streamBuffer;
            if ((streamBuffer.buffer   = (const native_handle_t**)bufferPop(small_vid_bufferGroup)) == NULL) {
                M_ERROR("Failed to get buffer for small vid stream: Cam(%s), Frame(%d)\n", name, *frameNumber);
                EStopCameraServer();
                return -1;
            }

            streamBuffer.stream        = &small_vid_stream;
            streamBuffer.status        = 0;
            streamBuffer.acquire_fence = -1;
            streamBuffer.release_fence = -1;

            request.num_output_buffers ++;
            streamBufferList.push_back(streamBuffer);
             M_VERBOSE("added request for small video stream\n");
        }
    }

    if(en_large_video && HasClientForLargeVideo()){

        int nFree = bufferNumFree(large_vid_bufferGroup);
        if(nFree<1){
            M_WARN("record stream buffer pool for Cam(%s), Frame(%d) has %d free, skipping request\n", name, *frameNumber, nFree);
        }
        else{
            camera3_stream_buffer_t streamBuffer;
            if ((streamBuffer.buffer   = (const native_handle_t**)bufferPop(large_vid_bufferGroup)) == NULL) {
                M_ERROR("Failed to get buffer for record stream: Cam(%s), Frame(%d)\n", name, *frameNumber);
                EStopCameraServer();
                return -1;
            }

            streamBuffer.stream        = &large_vid_stream;
            streamBuffer.status        = 0;
            streamBuffer.acquire_fence = -1;
            streamBuffer.release_fence = -1;

            request.num_output_buffers ++;
            streamBufferList.push_back(streamBuffer);
             M_VERBOSE("added request for large video stream\n");
        }
    }

    if(en_snapshot && numNeededSnapshots > 0){

        int nFree = bufferNumFree(snap_bufferGroup);
        if(nFree<1){
            M_WARN("snapshot buffer pool for Cam(%s), Frame(%d) has %d free, skipping request\n", name, *frameNumber, nFree);
        }
        else{
            numNeededSnapshots --;

            camera3_stream_buffer_t streamBuffer;
            if((streamBuffer.buffer    = (const native_handle_t**)bufferPop(snap_bufferGroup)) == NULL) {
                M_ERROR("Failed to get buffer for snapshot stream: Cam(%s), Frame(%d)\n", name, *frameNumber);
                EStopCameraServer();
                return -1;
            }
            streamBuffer.stream        = &snap_stream;
            streamBuffer.status        = 0;
            streamBuffer.acquire_fence = -1;
            streamBuffer.release_fence = -1;

            request.num_output_buffers ++;
            streamBufferList.push_back(streamBuffer);
             M_VERBOSE("added request for snapshot stream\n");
        }

    }

    if( en_preview &&
        (HasClientForPreviewFrame() ||
        (ae_mode == AE_ISP && !request.num_output_buffers) ||
        (ae_mode != AE_OFF && ae_mode != AE_ISP)))
    {
        int nFree = bufferNumFree(pre_bufferGroup);
        if(nFree<1){
            M_WARN("preview buffer pool for Cam(%s), Frame(%d) has %d free, skipping request\n", name, *frameNumber, nFree);
        }
        else{
            camera3_stream_buffer_t streamBuffer;
            if((streamBuffer.buffer    = (const native_handle_t**)bufferPop(pre_bufferGroup)) == NULL) {
                M_ERROR("Failed to get buffer for preview stream: Cam(%s), Frame(%d)\n", name, *frameNumber);
                EStopCameraServer();
                return -1;
            }
            streamBuffer.stream        = &pre_stream;
            streamBuffer.status        = 0;
            streamBuffer.acquire_fence = -1;
            streamBuffer.release_fence = -1;

            request.num_output_buffers ++;
            streamBufferList.push_back(streamBuffer);
            M_VERBOSE("added request for preview stream\n");
        }
    }

    request.output_buffers      = streamBufferList.data();
    request.frame_number        = *frameNumber;
    request.settings            = requestMetadata.getAndLock();
    request.input_buffer        = nullptr;

    // If there are no output buffers just do nothing
    // Without this an illegal zero output buffer request will be made
    if (request.num_output_buffers == 0){
        // Output buffers are full delay the next request
        // Without this wait at high CPU loads the loop will run away with CPU usage
        usleep(10000);
        return S_OK;
    }

    /* Return values (from hardware/camera3.h):
     *
     *  0:      On a successful start to processing the capture request
     *
     * -EINVAL: If the input is malformed (the settings are NULL when not
     *          allowed, invalid physical camera settings,
     *          there are 0 output buffers, etc) and capture processing
     *          cannot start. Failures during request processing should be
     *          handled by calling camera3_callback_ops_t.notify(). In case of
     *          this error, the framework will retain responsibility for the
     *          stream buffers' fences and the buffer handles; the HAL should
     *          not close the fences or return these buffers with
     *          process_capture_result.
     *
     * -ENODEV: If the camera device has encountered a serious error. After this
     *          error is returned, only the close() method can be successfully
     *          called by the framework.
     *
     */
    M_VERBOSE("Sending request for frame %d for camera %s for %d streams\n", *frameNumber, name, request.num_output_buffers);

    if (int status = pDevice->ops->process_capture_request(pDevice, &request))
    {

        //Another thread has already detected the fatal error, return since it has already been handled
        if(stopped) return 0;

        M_ERROR("Received Fatal error from camera: %s\n", name);
        switch (status){
            case -EINVAL :
                M_ERROR("Sending request %d, ErrorCode: -EINVAL\n", *frameNumber);
                break;
            case -ENODEV:
                M_ERROR("Sending request %d, ErrorCode: -ENODEV\n", *frameNumber);
                break;
            default:
                M_ERROR("Sending request %d, ErrorCode: %d\n", *frameNumber, status);
                break;
        }

        EStopCameraServer();
        return -EINVAL;
    }

    M_VERBOSE("finished sending request for frame %d for camera %s\n", *frameNumber, name);
    *frameNumber = *frameNumber+1;
    requestMetadata.unlock(request.settings);

    M_VERBOSE("returning from SendOneCaptureRequest for frame %d for camera %s\n", *frameNumber, name);

    return S_OK;
}

// -----------------------------------------------------------------------------------------------------------------------------
// Main thread function to initiate the sending of capture requests to the camera module. Keeps on sending the capture requests
// to the camera module till a "stop message" is passed to this thread function
// -----------------------------------------------------------------------------------------------------------------------------
void* PerCameraMgr::ThreadIssueCaptureRequests()
{

    uint32_t frame_number = 0;
    char buf[16];
    sprintf(buf, "cam%d-request", cameraId);
    pthread_setname_np(pthread_self(), buf);

    M_VERBOSE("Entered thread: %s(tid: %lu)\n", buf, syscall(SYS_gettid));

    if (ConstructDefaultRequestSettings()){

        M_ERROR("Failed to construct request settings for camera: %s\n", name);

        EStopCameraServer();
    }

    while (!stopped && !EStopped)
    {

        /** This is an old TODO comment, I think SendOneCaptureRequest handles
          * this now, but leaving the comment here in case it misbahves and better
          * behavior is needed in the future
                    // if(!getNumClients() && !numNeededSnapshots){
                    //     //TODODODODODODOD THIS NEEDS TO BE A COND SLEEP
                    //     usleep(100000);
                    //     if(stopped || EStopped) break;
                    // }
        **/
        SendOneCaptureRequest(&frame_number);
    }

    // Stop message received. Inform about the last framenumber requested from the camera module. This in turn will be used
    // by the result thread to wait for this frame's image buffers to arrive.
    if(EStopped){
        M_WARN("Thread: %s request thread Received ESTOP\n", name);
    }else{
        // SendOneCaptureRequest increments frame_number after successfully
        // sending the last frame -- therefore, the last request which was
        // actually sent to hal3 was frame_number - 1
        lastResultFrameNumber = (frame_number -  1);
        M_DEBUG("------ Last request frame for %s: %d\n", name, frame_number);
    }

    M_VERBOSE("Leaving %s request thread\n", name);

    return NULL;
}

enum AECommandVals {
    SET_EXP_GAIN,
    SET_EXP,
    SET_GAIN,
    START_AE,
    STOP_AE,
    SNAPSHOT,
    SNAPSHOT_NS
};
static const char* CmdStrings[] =
{
    "set_exp_gain",
    "set_exp",
    "set_gain",
    "start_ae",
    "stop_ae",
    "snapshot",
    "snapshot_no_save"
};

static int UpdatePipeJson(int channel, int int_format, int width, int height, int framerate)
{
    cJSON* json = pipe_server_get_info_json_ptr(channel);
    if(json == NULL){
        M_ERROR("got NULL pointer in %s\n", __FUNCTION__);
        return -1;
    }
    cJSON_AddStringToObject(json, "string_format", pipe_image_format_to_string(int_format));
    cJSON_AddNumberToObject(json, "int_format", int_format);
    cJSON_AddNumberToObject(json, "width", width);
    cJSON_AddNumberToObject(json, "height", height);
    cJSON_AddNumberToObject(json, "framerate", framerate);
    pipe_server_update_info(channel);
    return 0;
}

int PerCameraMgr::SetupPipes()
{
    if(configInfo.type != SENSOR_TOF){


        char cont_cmds[256];
        snprintf(cont_cmds, 255, "%s%s",
            EXPOSURE_CONTROL_COMMANDS,
            en_snapshot ? ",snapshot,snapshot_no_save" : "");
        int flags = SERVER_FLAG_EN_CONTROL_PIPE;

        pipe_info_t info;
        strcpy(info.type       , "camera_image_metadata_t");
        strcpy(info.server_name, PROCESS_NAME);
        info.size_bytes = 128*1024*1024;

        // preview streams
        if(en_preview){

            // legacy naming without a suffix, just for
            // old black and white cameras like OV7251 tracking and stereo
            if( configInfo.en_raw_preview && \
                configInfo.bayer_fmt == BAYER_MONO)
            {
                strncpy(info.name, name, MODAL_PIPE_MAX_NAME_LEN-1);
                previewPipeGrey = pipe_server_get_next_available_channel();
                pipe_server_set_control_cb(previewPipeGrey, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
                pipe_server_create(previewPipeGrey, info, flags);
                pipe_server_set_available_control_commands(previewPipeGrey, cont_cmds);
                UpdatePipeJson(previewPipeGrey, IMAGE_FORMAT_RAW8, pre_width, pre_height, fps);
            }

            // for color camera in raw mode and NOT stereo, advertize a raw bayered image
            if( configInfo.en_raw_preview && \
                configInfo.bayer_fmt != BAYER_MONO && \
                configInfo.camId2 < 0)
            {
                snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_bayer", name);
                previewPipeBayer = pipe_server_get_next_available_channel();
                pipe_server_set_control_cb(previewPipeBayer, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
                pipe_server_create(previewPipeBayer, info, flags);
                pipe_server_set_available_control_commands(previewPipeBayer, cont_cmds);
                UpdatePipeJson(previewPipeBayer, IMAGE_FORMAT_RAW8, pre_width, pre_height, fps);
            }

            // color cameras get a grey and color pipe when running RAW or through ISP
            if((configInfo.en_raw_preview && configInfo.bayer_fmt != BAYER_MONO) || pre_halfmt==HAL3_FMT_YUV)
            {
                snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_grey", name);
                previewPipeGrey = pipe_server_get_next_available_channel();
                pipe_server_set_control_cb(previewPipeGrey, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
                pipe_server_create(previewPipeGrey, info, flags);
                pipe_server_set_available_control_commands(previewPipeGrey, cont_cmds);
                UpdatePipeJson(previewPipeGrey, IMAGE_FORMAT_RAW8, pre_width, pre_height, fps);
            }

            if((configInfo.en_raw_preview && configInfo.bayer_fmt != BAYER_MONO && configInfo.camId2 < 0) \
                    || pre_halfmt==HAL3_FMT_YUV)
            {
                snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_color", name);
                previewPipeColor = pipe_server_get_next_available_channel();
                pipe_server_set_control_cb(previewPipeColor, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
                pipe_server_create(previewPipeColor, info, flags);
                pipe_server_set_available_control_commands(previewPipeColor, cont_cmds);
                if(configInfo.en_raw_preview){
                    // raw preview gets debayered to RGB
                    UpdatePipeJson(previewPipeColor, IMAGE_FORMAT_RGB, pre_width, pre_height, fps);
                }else{// yuv ISP generated preview comes out as NV12
                    UpdatePipeJson(previewPipeColor, IMAGE_FORMAT_NV12, pre_width, pre_height, fps);
                }
            }
        }


        // small encoded video stream for hires cameras
        if(en_small_video){
            snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_small_grey", name);
            smallVideoPipeGrey = pipe_server_get_next_available_channel();
            pipe_server_set_control_cb(smallVideoPipeGrey, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
            pipe_server_create(smallVideoPipeGrey, info, flags);
            pipe_server_set_available_control_commands(smallVideoPipeGrey, cont_cmds);
            UpdatePipeJson(smallVideoPipeGrey, IMAGE_FORMAT_RAW8, small_video_width, small_video_height, fps);

            snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_small_color", name);
            smallVideoPipeColor = pipe_server_get_next_available_channel();
            pipe_server_set_control_cb(smallVideoPipeColor, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
            pipe_server_create(smallVideoPipeColor, info, flags);
            pipe_server_set_available_control_commands(smallVideoPipeColor, cont_cmds);
            UpdatePipeJson(smallVideoPipeColor, IMAGE_FORMAT_NV12, small_video_width, small_video_height, fps);

            snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_small_encoded", name);
            smallVideoPipeEncoded = pipe_server_get_next_available_channel();
            pipe_server_set_control_cb(smallVideoPipeEncoded, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
            pipe_server_create(smallVideoPipeEncoded, info, flags);
            pipe_server_set_available_control_commands(smallVideoPipeEncoded, cont_cmds);
            int fmt = IMAGE_FORMAT_H264;
            if(configInfo.small_venc_config.mode == VENC_H265) fmt = IMAGE_FORMAT_H265;
            UpdatePipeJson(smallVideoPipeEncoded, fmt, small_video_width, small_video_height, fps);
        }

        // large encoded video stream for hires cameras
        if(en_large_video){
            snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_large_grey", name);
            largeVideoPipeGrey = pipe_server_get_next_available_channel();
            pipe_server_set_control_cb(largeVideoPipeGrey, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
            pipe_server_create(largeVideoPipeGrey, info, flags);
            pipe_server_set_available_control_commands(largeVideoPipeGrey, cont_cmds);
            (void) UpdatePipeJson(largeVideoPipeGrey, IMAGE_FORMAT_RAW8, large_video_width, large_video_height, fps);

            snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_large_color", name);
            largeVideoPipeColor = pipe_server_get_next_available_channel();
            pipe_server_set_control_cb(largeVideoPipeColor, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
            pipe_server_create(largeVideoPipeColor, info, flags);
            pipe_server_set_available_control_commands(largeVideoPipeColor, cont_cmds);
            (void) UpdatePipeJson(largeVideoPipeColor, IMAGE_FORMAT_NV12, large_video_width, large_video_height, fps);

            snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_large_encoded", name);
            largeVideoPipeEncoded = pipe_server_get_next_available_channel();
            pipe_server_set_control_cb(largeVideoPipeEncoded, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
            pipe_server_create(largeVideoPipeEncoded, info, flags);
            pipe_server_set_available_control_commands(largeVideoPipeEncoded, cont_cmds);
            int fmt = IMAGE_FORMAT_H264;
            if(configInfo.large_venc_config.mode == VENC_H265) fmt = IMAGE_FORMAT_H265;
            UpdatePipeJson(largeVideoPipeEncoded, fmt, large_video_width, large_video_height, fps);
        }


        if(en_snapshot){
            snprintf(info.name, MODAL_PIPE_MAX_NAME_LEN-1, "%s_snapshot", name);
            snapshotPipe = pipe_server_get_next_available_channel();
            pipe_server_set_control_cb(snapshotPipe, [](int ch, char * string, int bytes, void* context){((PerCameraMgr*)context)->HandleControlCmd(string);},this);
            pipe_server_create(snapshotPipe, info, flags);
            pipe_server_set_available_control_commands(snapshotPipe, cont_cmds);
            UpdatePipeJson(snapshotPipe, IMAGE_FORMAT_JPG, snap_width, snap_height, 0);
        }




    } else {

        tofPipeIR    = pipe_server_get_next_available_channel();
        tofPipeDepth = pipe_server_get_next_available_channel();
        tofPipeConf  = pipe_server_get_next_available_channel();
        tofPipePC    = pipe_server_get_next_available_channel();
        tofPipeFull  = pipe_server_get_next_available_channel();

        pipe_info_t IRInfo;
        pipe_info_t DepthInfo;
        pipe_info_t ConfInfo;
        pipe_info_t PCInfo;
        pipe_info_t FullInfo;

        sprintf(IRInfo.name,    "%s%s", name, "_ir");
        sprintf(DepthInfo.name, "%s%s", name, "_depth");
        sprintf(ConfInfo.name,  "%s%s", name, "_conf");
        sprintf(PCInfo.name,    "%s%s", name, "_pc");
        sprintf(FullInfo.name,  "%s%s", name, "");

        strcpy(IRInfo.type,     "camera_image_metadata_t");
        strcpy(DepthInfo.type,  "camera_image_metadata_t");
        strcpy(ConfInfo.type,   "camera_image_metadata_t");
        strcpy(PCInfo.type,     "point_cloud_metadata_t");
        strcpy(FullInfo.type,   "tof_data_t");

        strcpy(IRInfo.server_name,    PROCESS_NAME);
        strcpy(DepthInfo.server_name, PROCESS_NAME);
        strcpy(ConfInfo.server_name,  PROCESS_NAME);
        strcpy(PCInfo.server_name,    PROCESS_NAME);
        strcpy(FullInfo.server_name,  PROCESS_NAME);

        IRInfo.size_bytes    =    1024*1024;
        DepthInfo.size_bytes =    1024*1024;
        ConfInfo.size_bytes  =    1024*1024;
        PCInfo.size_bytes    = 32*1024*1024;
        FullInfo.size_bytes  = 32*1024*1024;

        int flags = 0;
        pipe_server_create(tofPipeIR,    IRInfo,    flags);
        pipe_server_create(tofPipeDepth, DepthInfo, flags);
        pipe_server_create(tofPipeConf,  ConfInfo,  flags);
        pipe_server_create(tofPipePC,    PCInfo,    flags);
        pipe_server_create(tofPipeFull,  FullInfo,  flags);

        UpdatePipeJson(tofPipeIR,    IMAGE_FORMAT_RAW8, 224, 172, fps);
        UpdatePipeJson(tofPipeDepth, IMAGE_FORMAT_RAW8, 224, 172, fps);
        UpdatePipeJson(tofPipeConf,  IMAGE_FORMAT_RAW8, 224, 172, fps);

    }
    return S_OK;
}


void PerCameraMgr::HandleControlCmd(char* cmd)
{

    const float MIN_EXP  = ((float)configInfo.ae_msv_info.exposure_min_us)/1000;
    const float MAX_EXP  = ((float)configInfo.ae_msv_info.exposure_max_us)/1000;
    const int MIN_GAIN = configInfo.ae_msv_info.gain_min;
    const int MAX_GAIN = configInfo.ae_msv_info.gain_max;

    /**************************
     *
     * SET Exposure and Gain
     *
     */
    if(strncmp(cmd, CmdStrings[SET_EXP_GAIN], strlen(CmdStrings[SET_EXP_GAIN])) == 0){

        char buffer[strlen(CmdStrings[SET_EXP_GAIN])+1];
        float exp = -1.0;
        int gain = -1;

        if(sscanf(cmd, "%s %f %d", buffer, &exp, &gain) == 3){
            if(exp < MIN_EXP || exp > MAX_EXP){
                M_ERROR("Invalid Control Pipe Exposure: %f,\n\tShould be between %f and %f\n", exp, MIN_EXP, MAX_EXP);
            } else if(gain < MIN_GAIN || gain > MAX_GAIN){
                M_ERROR("Invalid Control Pipe Gain: %d,\n\tShould be between %d and %d\n", gain, MIN_GAIN, MAX_GAIN);
            } else {
                pthread_mutex_lock(&aeMutex);

                if(ae_mode != AE_OFF) {
                    ae_mode = AE_OFF;
                    ConstructDefaultRequestSettings();

                    if(otherMgr){
                        otherMgr->ae_mode = AE_OFF;
                        otherMgr->ConstructDefaultRequestSettings();
                    }
                }

                M_DEBUG("Camera: %s Received new exp/gain values: %6.3f(ms) %d\n", name, exp, gain);

                setExposure = exp*1000000;
                setGain =     gain;

                if(otherMgr){
                    otherMgr->setExposure = exp*1000000;
                    otherMgr->setGain =     gain;
                }

                pthread_mutex_unlock(&aeMutex);
            }
        } else {
            M_ERROR("Camera: %s failed to get valid exposure/gain values from control pipe\n\tShould follow format: \"%s 25 350\"\n",
             name, CmdStrings[SET_EXP_GAIN]);
        }

    } else
    /**************************
     *
     * SET Exposure
     *
     */
    if(strncmp(cmd, CmdStrings[SET_EXP], strlen(CmdStrings[SET_EXP])) == 0){

        char buffer[strlen(CmdStrings[SET_EXP])+1];
        float exp = -1.0;

        if(sscanf(cmd, "%s %f", buffer, &exp) == 2){
            if(exp < MIN_EXP || exp > MAX_EXP){
                M_ERROR("Invalid Control Pipe Exposure: %f,\n\tShould be between %f and %f\n", exp, MIN_EXP, MAX_EXP);
            } else {
                pthread_mutex_lock(&aeMutex);

                if(ae_mode != AE_OFF) {
                    ae_mode = AE_OFF;
                    ConstructDefaultRequestSettings();

                    if(otherMgr){
                        otherMgr->ae_mode = AE_OFF;
                        otherMgr->ConstructDefaultRequestSettings();
                    }
                }

                M_DEBUG("Camera: %s Received new exp value: %6.3f(ms)\n", name, exp);
                setExposure = exp*1000000;

                if(otherMgr){
                    otherMgr->setExposure = exp*1000000;
                }

                pthread_mutex_unlock(&aeMutex);
            }
        } else {
            M_ERROR("Camera: %s failed to get valid exposure value from control pipe\n\tShould follow format: \"%s 25\"\n",
            name, CmdStrings[SET_EXP]);
        }
    } else
    /**************************
     *
     * SET Gain
     *
     */
    if(strncmp(cmd, CmdStrings[SET_GAIN], strlen(CmdStrings[SET_GAIN])) == 0){

        char buffer[strlen(CmdStrings[SET_GAIN])+1];
        int gain = -1;

        if(sscanf(cmd, "%s %d", buffer, &gain) == 2){
            if(gain < MIN_GAIN || gain > MAX_GAIN){
                M_ERROR("Invalid Control Pipe Gain: %d,\n\tShould be between %d and %d\n", gain, MIN_GAIN, MAX_GAIN);
            } else {
                pthread_mutex_lock(&aeMutex);

                if(ae_mode != AE_OFF) {
                    ae_mode = AE_OFF;
                    ConstructDefaultRequestSettings();

                    if(otherMgr){
                        otherMgr->ae_mode = AE_OFF;
                        otherMgr->ConstructDefaultRequestSettings();
                    }
                }

                M_DEBUG("Camera: %s Received new gain value: %d\n", name, gain);
                setGain = gain;

                if(otherMgr){
                    otherMgr->setGain = gain;
                }

                pthread_mutex_unlock(&aeMutex);
            }
        } else {
            M_ERROR("Camera: %s failed to get valid gain value from control pipe\n\tShould follow format: \"%s 350\"\n",
            name, CmdStrings[SET_GAIN]);
        }
    } else
    /**************************
     *
     * START Auto Exposure
     *
     */
    if(strncmp(cmd, CmdStrings[START_AE], strlen(CmdStrings[START_AE])) == 0){

        pthread_mutex_lock(&aeMutex);

        if(ae_mode != configInfo.ae_mode) {
            ae_mode = configInfo.ae_mode;
            ConstructDefaultRequestSettings();

            if(otherMgr){
                otherMgr->ae_mode = configInfo.ae_mode;
                otherMgr->ConstructDefaultRequestSettings();
            }

            M_DEBUG("Camera: %s starting to use Auto Exposure\n", name);
        }
        pthread_mutex_unlock(&aeMutex);

    } else
    /**************************
     *
     * STOP Auto Exposure
     *
     */
    if(strncmp(cmd, CmdStrings[STOP_AE], strlen(CmdStrings[STOP_AE])) == 0){

        pthread_mutex_lock(&aeMutex);

        if(ae_mode != AE_OFF) {
            ae_mode = AE_OFF;
            ConstructDefaultRequestSettings();

            if(otherMgr){
                otherMgr->ae_mode = AE_OFF;
                otherMgr->ConstructDefaultRequestSettings();
            }
            M_DEBUG("Camera: %s ceasing to use Auto Exposure\n", name);
        }
        pthread_mutex_unlock(&aeMutex);

    } else

    /**************************
     *
     * Take snapshot without saving
     *
     */
    if(strncmp(cmd, CmdStrings[SNAPSHOT_NS], strlen(CmdStrings[SNAPSHOT_NS])) == 0 ||
       strncmp(cmd, "snapshot-no-save",      strlen(CmdStrings[SNAPSHOT_NS])) == 0){
        if(!en_snapshot){
            M_ERROR("Camera: %s declining to take snapshot, mode not enabled\n", name);
        }
        else{
            M_PRINT("Camera: %s taking snapshot for pipe only (not saving it)\n", name);
            numNeededSnapshots++;
        }
    } else

    /**************************
     *
     * Take snapshot to save with filename
     *
     */
    if(strncmp(cmd, CmdStrings[SNAPSHOT], strlen(CmdStrings[SNAPSHOT])) == 0){
        if(!en_snapshot){
            M_ERROR("Camera: %s declining to take snapshot, mode not enabled\n", name);
        }
        else{
            char buffer[strlen(CmdStrings[SET_EXP_GAIN])+1];
            char *filename = (char *)malloc(256);

            if(sscanf(cmd, "%s %s", buffer, filename) != 2){
                // We weren't given a proper file, generate one from the date and time
                // fetch time to use for video filename
                time_t rawtime = time(NULL);
                char time_formatted[100];
                strftime(time_formatted, sizeof(time_formatted)-1, "%Y-%m-%d_%T", localtime(&rawtime));

                // construct the video filename
                sprintf(filename,"/data/snapshots/%s-%s.jpg", name, time_formatted);

            }

            M_PRINT("Camera: %s taking snapshot (destination: %s)\n", name, filename);

            snapshotQueue.push(filename);
            numNeededSnapshots++;
        }
    }
    /**************************
     *
     *  ¯\_(ツ)_/¯
     *
     */
    else {
        M_ERROR("Camera: %s got unknown Command: %s\n", name, cmd);
    }
}



void PerCameraMgr::EStop(){

    EStopped = true;
    stopped = true;
    pthread_cond_broadcast(&stereoCond);
    pthread_cond_broadcast(&resultCond);

    if(partnerMode == MODE_STEREO_MASTER){
        otherMgr->EStop();
    }
}


