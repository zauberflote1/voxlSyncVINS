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
#include <OMX_Component.h>
#include <OMX_IndexExt.h>
#include <cstring>
#include <dlfcn.h>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <errno.h>
#include <system/graphics.h>
// #include <media/hardware/HardwareAPI.h>
#include <OMX_QCOMExtns.h>
#include <modal_journal.h>
#include <OMX_VideoExt.h>
#include <voxl_cutils.h>
#include <modal_pipe.h>

#include "omx_video_encoder.h"
#include "buffer_manager.h"
#include "common_defs.h"
#include "voxl_camera_server.h"

// stream gets more I frames per second
#define N_I_FRAMES_STREAM 2
#define NUM_OUTPUT_BUFFERS 16

#ifdef APQ8096
    const char* OMX_LIB_NAME = "/usr/lib/libOmxCore.so";
    const OMX_COLOR_FORMATTYPE   OMX_COLOR_FMT = OMX_QCOM_COLOR_FormatYVU420SemiPlanar;
#elif QRB5165
    const char* OMX_LIB_NAME = "/usr/lib/libmm-omxcore.so";
    const OMX_COLOR_FORMATTYPE   OMX_COLOR_FMT = OMX_COLOR_FormatYUV420SemiPlanar;
#endif

///<@todo Make these functions
#define Log2(number, power) {OMX_U32 temp = number; power = 0; while ((0 == (temp & 0x1)) && power < 16) {temp >>=0x1; power++;}}
#define FractionToQ16(q,num,den) { OMX_U32 power; Log2(den,power); q = num << (16 - power); }

static const int32_t OMXSpecVersion = 0x00000101;

// Helper MACRO to reset the size, version of any OMX structure
#define OMX_RESET_STRUCT_SIZE_VERSION(_structPointer_, _structName_)    \
    (_structPointer_)->nSize = sizeof(_structName_);                    \
    (_structPointer_)->nVersion.nVersion = OMXSpecVersion

// Helper MACRO to reset any OMX structure to its default valid state
#define OMX_RESET_STRUCT(_structPointer_, _structName_)     \
    memset((_structPointer_), 0x0, sizeof(_structName_));   \
    (_structPointer_)->nSize = sizeof(_structName_);        \
    (_structPointer_)->nVersion.nVersion = OMXSpecVersion

// Main thread functions for providing input buffer to the OMX input port and processing encoded buffer on the OMX output port
void* ThreadProcessOMXInputPort(void* data);
void* ThreadProcessOMXOutputPort(void* data);

// Function called by the OMX component for event handling
OMX_ERRORTYPE OMXEventHandler(OMX_IN OMX_HANDLETYPE hComponent,
                              OMX_IN OMX_PTR        pAppData,
                              OMX_IN OMX_EVENTTYPE  eEvent,
                              OMX_IN OMX_U32        nData1,
                              OMX_IN OMX_U32        nData2,
                              OMX_IN OMX_PTR        pEventData);

// Function called by the OMX component to indicate the input buffer we passed to it has been consumed
OMX_ERRORTYPE OMXEmptyBufferHandler(OMX_IN OMX_HANDLETYPE        hComponent,
                                    OMX_IN OMX_PTR               pAppData,
                                    OMX_IN OMX_BUFFERHEADERTYPE* pBuffer);

// Function called by the OMX component to hand over the encoded output buffer
OMX_ERRORTYPE OMXFillHandler(OMX_OUT OMX_HANDLETYPE        hComponent,
                             OMX_OUT OMX_PTR               pAppData,
                             OMX_OUT OMX_BUFFERHEADERTYPE* pBuffer);

// Investigate build-time linking to omx instead of this mess
typedef OMX_ERRORTYPE (*OMXGetHandleFunc)(OMX_OUT OMX_HANDLETYPE* handle,
                                          OMX_IN OMX_STRING componentName,
                                          OMX_IN OMX_PTR appData,
                                          OMX_IN OMX_CALLBACKTYPE* callBacks);
typedef OMX_ERRORTYPE (*OMXFreeHandleFunc)(OMX_IN OMX_HANDLETYPE hComp);

static OMX_ERRORTYPE (*OMXInit)(void);
static OMX_ERRORTYPE (*OMXDeinit)(void);
static OMXGetHandleFunc OMXGetHandle;
static OMXFreeHandleFunc OMXFreeHandle;

static void __attribute__((constructor)) setupOMXFuncs()
{

    void *OmxCoreHandle = dlopen(OMX_LIB_NAME, RTLD_NOW);

    OMXInit =   (OMX_ERRORTYPE (*)(void))dlsym(OmxCoreHandle, "OMX_Init");
    OMXDeinit = (OMX_ERRORTYPE (*)(void))dlsym(OmxCoreHandle, "OMX_Deinit");
    OMXGetHandle =     (OMXGetHandleFunc)dlsym(OmxCoreHandle, "OMX_GetHandle");
    OMXFreeHandle =   (OMXFreeHandleFunc)dlsym(OmxCoreHandle, "OMX_FreeHandle");
}


// -----------------------------------------------------------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------------------------------------------------------
VideoEncoder::VideoEncoder(VideoEncoderConfig* PVideoEncoderConfig)
{
    pthread_mutex_init(&out_mutex, NULL);

    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&out_cond, &attr);
    pthread_condattr_destroy(&attr);

    m_VideoEncoderConfig = *PVideoEncoderConfig;
    m_outputPipe        = m_VideoEncoderConfig.outputPipe;
    m_inputBufferSize   = 0;
    m_inputBufferCount  = 0;
    m_outputBufferSize  = 0;
    m_outputBufferCount = 0;

    m_nextInputBufferIndex  = 0;
    m_nextOutputBufferIndex = 0;

    if(OMXInit()){
        M_ERROR("OMX Init failed!\n");
        throw -EINVAL;
    }

    if(SetConfig()){
        M_ERROR("OMX Set config failed!\n");
        throw -EINVAL;
    }

    if(OMX_SendCommand(m_OMXHandle, OMX_CommandStateSet, (OMX_U32)OMX_StateExecuting, NULL)){
        M_ERROR("OMX Set state executing failed!\n");
        throw -EINVAL;
    }

    // We do this so that the OMX component has output buffers to fill the encoded frame data. The output buffers are
    // recycled back to the OMX component once they are returned to us and after we write the frame content to the
    // video file
    for(uint32_t i = 0;  i < m_outputBufferCount; i++){
        if(OMX_FillThisBuffer(m_OMXHandle, m_ppOutputBuffers[i])){
            M_ERROR("OMX Fill buffer: %d failed!\n", i);
            throw -EINVAL;
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------------------------------------------------------
VideoEncoder::~VideoEncoder()
{
   return;

}


// -----------------------------------------------------------------------------------------------------------------------------
// This configures the OMX component i.e. the video encoder's input, output ports and all its parameters and gets it into a
// ready to use state. After this function we can start sending input buffers to the video encoder and it will start sending
// back the encoded frames
// -----------------------------------------------------------------------------------------------------------------------------
OMX_ERRORTYPE VideoEncoder::SetConfig(void)
{
    OMX_ERRORTYPE ret;
    char* pComponentName;
    OMX_CALLBACKTYPE                 callbacks = {OMXEventHandler, OMXEmptyBufferHandler, OMXFillHandler};
    OMX_COLOR_FORMATTYPE             omxFormat = OMX_COLOR_FormatMax;
    OMX_VIDEO_PARAM_PORTFORMATTYPE   videoPortFmt;

    m_pHALInputBuffers = m_VideoEncoderConfig.inputBuffers;
    OMX_RESET_STRUCT(&videoPortFmt, OMX_VIDEO_PARAM_PORTFORMATTYPE);

    if(m_VideoEncoderConfig.venc_config.mode == VENC_H265){
        pComponentName = (char *)"OMX.qcom.video.encoder.hevc";
    }else{
        pComponentName = (char *)"OMX.qcom.video.encoder.avc";
    }

    ret = OMXGetHandle(&m_OMXHandle, pComponentName, this, &callbacks);
    if(ret){
        M_ERROR("OMX Get handle failed!\n");
        _print_omx_error(ret);
        return OMX_ErrorUndefined;
    }

    if(m_VideoEncoderConfig.format != HAL_PIXEL_FORMAT_YCbCr_420_888){
        M_ERROR("OMX Unknown video recording format!\n");
        return OMX_ErrorBadParameter;
    }

    omxFormat = OMX_COLOR_FMT;
    bool isFormatSupported = false;
    // Check if OMX component supports the input frame format
    OMX_S32 index = 0;

    M_DEBUG("Available color formats for OMX:\n");

    while (!OMX_GetParameter(m_OMXHandle, OMX_IndexParamVideoPortFormat, (OMX_PTR)&videoPortFmt)){
        videoPortFmt.nPortIndex = PortIndexIn;
        videoPortFmt.nIndex     = index;
        if (videoPortFmt.eColorFormat == omxFormat) isFormatSupported = true;
        M_DEBUG("\t%s (0x%x)\n", colorFormatStr(videoPortFmt.eColorFormat), videoPortFmt.eColorFormat);
        index++;
    }

    if (!isFormatSupported){
        M_ERROR("OMX unsupported video input format: %s\n", colorFormatStr(omxFormat));
        return OMX_ErrorBadParameter;
    }


    // Set/Get input port parameters
    if (SetPortParams((OMX_U32)PortIndexIn,
                      (OMX_U32)(m_VideoEncoderConfig.inputBuffers->totalBuffers),
                      (OMX_U32*)&m_inputBufferSize,
                      (OMX_U32*)&m_inputBufferCount,
                      omxFormat))
    {
        M_ERROR("OMX SetPortParams of PortIndexIn failed!\n");
        return OMX_ErrorUndefined;
    }

    // Set/Get output port parameters
    if (SetPortParams((OMX_U32)PortIndexOut,
                      (OMX_U32)NUM_OUTPUT_BUFFERS,
                      (OMX_U32*)&m_outputBufferSize,
                      (OMX_U32*)&m_outputBufferCount,
                      omxFormat))
    {
        M_ERROR("OMX SetPortParams of PortIndexOut failed!\n");
        return OMX_ErrorUndefined;
    }






    ////////////////////////////////////////////////////////////////////////////
    // set h264/h265 params
    ////////////////////////////////////////////////////////////////////////////
    if(m_VideoEncoderConfig.venc_config.mode == VENC_H264){

        ////////////////////////////////////////////////////////////////////////
        // set OMX_VIDEO_PARAM_AVCTYPE
        // start with platform defaults and work from there, don't erase
        ////////////////////////////////////////////////////////////////////////
        OMX_VIDEO_PARAM_AVCTYPE avc;
        OMX_RESET_STRUCT(&avc, OMX_VIDEO_PARAM_AVCTYPE);
        avc.nPortIndex = PortIndexOut;
        ret = OMX_GetParameter(m_OMXHandle, OMX_IndexParamVideoAvc, (OMX_PTR)&avc);
        if(ret){
            M_ERROR("OMX Get parameter of OMX_IndexParamVideoAvc failed\n");
            _print_omx_error(ret);
            return OMX_ErrorUndefined;
        }

        // TODO see if this has any benefit from being configurable
        avc.eProfile = OMX_VIDEO_AVCProfileMain;
        avc.eLevel   = OMX_VIDEO_AVCLevel5;

        // defualts from qcom open source omx/v4l2 android implementation
        avc.nPFrames = m_VideoEncoderConfig.venc_config.nPframes;
        avc.nBFrames = 0;
        avc.bUseHadamard = OMX_FALSE;
        avc.nRefIdx10ActiveMinus1 = 1;
        avc.nRefIdx11ActiveMinus1 = 0;
        avc.bEnableUEP = OMX_FALSE;
        avc.bEnableFMO = OMX_FALSE;
        avc.bEnableASO = OMX_FALSE;
        avc.bEnableRS = OMX_FALSE;
        avc.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
        avc.bFrameMBsOnly = OMX_FALSE;
        avc.bMBAFF = OMX_FALSE;
        avc.bEntropyCodingCABAC = OMX_FALSE;
        avc.bWeightedPPrediction = OMX_FALSE;
        avc.nWeightedBipredicitonMode = 0;
        avc.bconstIpred = OMX_FALSE;
        avc.bDirect8x8Inference = OMX_FALSE;
        avc.bDirectSpatialTemporal = OMX_FALSE;
        avc.nCabacInitIdc = 0;
        avc.eLoopFilterMode = OMX_VIDEO_AVCLoopFilterEnable;
        avc.nSliceHeaderSpacing       = 1024; // don't know where this came from

        // other defaults from somewhere, probably thundercomm example
        // avc.nBFrames                  = 0;
        // avc.bUseHadamard              = OMX_TRUE;
        // avc.nRefFrames                = 2;
        // avc.nRefIdx10ActiveMinus1     = 0;
        // avc.nRefIdx11ActiveMinus1     = 0;
        // avc.bEnableUEP                = OMX_FALSE;
        // avc.bEnableFMO                = OMX_FALSE;
        // avc.bEnableASO                = OMX_FALSE;
        // avc.bEnableRS                 = OMX_FALSE;
        // avc.nAllowedPictureTypes      =
        // avc.bFrameMBsOnly             = OMX_TRUE;
        // avc.bMBAFF                    = OMX_FALSE;
        // avc.bWeightedPPrediction      = OMX_TRUE;
        // avc.bconstIpred               = OMX_TRUE;
        // avc.bDirect8x8Inference       = OMX_TRUE;
        // avc.bDirectSpatialTemporal    = OMX_TRUE;
        // avc.eLoopFilterMode           = OMX_VIDEO_AVCLoopFilterEnable;
        // avc.bEntropyCodingCABAC       = OMX_TRUE;
        // avc.nCabacInitIdc             = 1;

        ret = OMX_SetParameter(m_OMXHandle, OMX_IndexParamVideoAvc, (OMX_PTR)&avc);
        if(ret){
            M_ERROR("OMX_SetParameter of OMX_IndexParamVideoAvc failed!\n");
            _print_omx_error(ret);
            return OMX_ErrorUndefined;
        }
    }

    // Configure for H265
    else if(m_VideoEncoderConfig.venc_config.mode == VENC_H265)
    {
        ////////////////////////////////////////////////////////////////////////
        // set OMX_VIDEO_PARAM_HEVCTYPE
        ////////////////////////////////////////////////////////////////////////
        OMX_VIDEO_PARAM_HEVCTYPE hevc;
        OMX_RESET_STRUCT(&hevc, OMX_VIDEO_PARAM_HEVCTYPE);
        hevc.nPortIndex = PortIndexOut;
        ret = OMX_GetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_IndexParamVideoHevc, (OMX_PTR)&hevc);
        if(ret){
            M_ERROR("OMX_GetParameter of OMX_IndexParamVideoHevc failed!\n");
            _print_omx_error(ret);
            return OMX_ErrorUndefined;
        }

        //nKeyFrameInterval;        // distance between consecutive I-frames (including one
                                      // of the I frames). 0 means interval is unspecified and
                                      // can be freely chosen by the codec. 1 means a stream of
                                      // only I frames.
        hevc.nKeyFrameInterval = m_VideoEncoderConfig.venc_config.nPframes+1;
        hevc.eProfile = OMX_VIDEO_HEVCProfileMain;
        hevc.eLevel   = OMX_VIDEO_HEVCHighTierLevel5;

        // TODO more params here?

        ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_IndexParamVideoHevc, (OMX_PTR)&hevc);
        if(ret){
            M_ERROR("OMX_SetParameter of OMX_IndexParamVideoHevc failed!\n");
            return OMX_ErrorUndefined;
        }

    }
    else{
        M_ERROR("Unsupported coding type!\n");
        return OMX_ErrorBadParameter;
    }


    ////////////////////////////////////////////////////////////////////////////
    // OMX_CONFIG_FRAMERATETYPE
    ////////////////////////////////////////////////////////////////////////////
    OMX_CONFIG_FRAMERATETYPE framerate;
    OMX_RESET_STRUCT_SIZE_VERSION(&framerate, OMX_CONFIG_FRAMERATETYPE);
    framerate.nPortIndex = PortIndexOut;
    // FractionToQ16(framerate.xEncodeFramerate, (int)(m_VideoEncoderConfig.frameRate * 2), 2);
    framerate.xEncodeFramerate = m_VideoEncoderConfig.frameRate << 16;
    ret = OMX_SetConfig(m_OMXHandle, OMX_IndexConfigVideoFramerate, (OMX_PTR)&framerate);
    if(ret){
        M_ERROR("OMX_SetConfig of OMX_IndexConfigVideoFramerate failed!\n");
        _print_omx_error(ret);
        return OMX_ErrorUndefined;
    }


    ////////////////////////////////////////////////////////////////////////
    // set QOMX_EXTNINDEX_VIDEO_INITIALQP
    // bitrate still seems to fluctuate a lot on startup, but does it
    // slightly less when starting with the max compression
    ////////////////////////////////////////////////////////////////////////
    QOMX_EXTNINDEX_VIDEO_INITIALQP initqp;
    OMX_RESET_STRUCT_SIZE_VERSION(&initqp, QOMX_EXTNINDEX_VIDEO_INITIALQP);
    initqp.nPortIndex = PortIndexOut;
    int qstart;
    if(m_VideoEncoderConfig.venc_config.br_ctrl==VENC_CONTROL_CQP){
        qstart = m_VideoEncoderConfig.venc_config.Qfixed;
    }else{
        qstart = (m_VideoEncoderConfig.venc_config.Qmin + m_VideoEncoderConfig.venc_config.Qmax)/2;
    }
    initqp.nQpI = qstart;
    initqp.nQpP = qstart;
    initqp.nQpB = qstart;
    initqp.bEnableInitQp = 0x7; // apply to all I, P, and B
    ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)QOMX_IndexParamVideoInitialQp,(OMX_PTR)&initqp);
    if(ret){
        M_ERROR("%s Failed to set Initial QP parameter\n", __func__);
        _print_omx_error(ret);
        return ret;
    }


    ////////////////////////////////////////////////////////////////////////
    // set OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE
    // standard OMX_VIDEO_PARAM_QUANTIZATIONTYPE doesn't work
    // this is the main quantization range that's respected!
    ////////////////////////////////////////////////////////////////////////
#ifndef APQ8096
    OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE qp_range;
    OMX_RESET_STRUCT_SIZE_VERSION(&qp_range, OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE);
    qp_range.nPortIndex = PortIndexOut;
    int qmin, qmax;
    if(m_VideoEncoderConfig.venc_config.br_ctrl==VENC_CONTROL_CQP){
        qmin = m_VideoEncoderConfig.venc_config.Qfixed;
        qmax = m_VideoEncoderConfig.venc_config.Qfixed;
    }else{
        qmin = m_VideoEncoderConfig.venc_config.Qmin;
        qmax = m_VideoEncoderConfig.venc_config.Qmax;
    }
    qp_range.minIQP = qmin;
    qp_range.maxIQP = qmax;
    qp_range.minPQP = qmin;
    qp_range.maxPQP = qmax;
    qp_range.minBQP = qmin;
    qp_range.maxBQP = qmax;
    ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_QcomIndexParamVideoIPBQPRange, (OMX_PTR)&qp_range);
    if(ret){
        M_ERROR("%s Failed to set Q Range parameter\n", __func__);
        _print_omx_error(ret);
        return ret;
    }
#endif



    ////////////////////////////////////////////////////////////////////////////
    // OMX_VIDEO_PARAM_BITRATETYPE && OMX_VIDEO_CONFIG_BITRATETYPE
    // both need to be set
    ////////////////////////////////////////////////////////////////////////////
    OMX_VIDEO_PARAM_BITRATETYPE paramBitRate;
    OMX_RESET_STRUCT_SIZE_VERSION(&paramBitRate, OMX_VIDEO_PARAM_BITRATETYPE);
    paramBitRate.nPortIndex = PortIndexOut;
    OMX_U32 bps = m_VideoEncoderConfig.venc_config.mbps*1000000;
    // constant Q mode
    if(m_VideoEncoderConfig.venc_config.br_ctrl == VENC_CONTROL_CQP){
        paramBitRate.eControlRate = OMX_Video_ControlRateDisable;
        paramBitRate.nTargetBitrate = 0;
    }
    // constant bitrate mode
    else if(m_VideoEncoderConfig.venc_config.br_ctrl == VENC_CONTROL_CBR){
        paramBitRate.eControlRate = OMX_Video_ControlRateConstant;
        paramBitRate.nTargetBitrate = bps;
    }
    else{
        M_ERROR("unknown control rate %d\n", m_VideoEncoderConfig.venc_config.br_ctrl);
        return OMX_ErrorUndefined;
    }

    // Other modes to try:
    // paramBitRate.eControlRate = OMX_Video_ControlRateVariable;
    // paramBitRate.eControlRate = QOMX_Video_ControlRateMaxBitrate;
    // paramBitRate.eControlRate = QOMX_Video_ControlRateMaxBitrateSkipFrames;

    ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_IndexParamVideoBitrate, (OMX_PTR)&paramBitRate);
    if(ret){
        M_ERROR("OMX_SetParameter of OMX_IndexParamVideoBitrate failed for out index!\n");
        _print_omx_error(ret);
        return OMX_ErrorUndefined;
    }

    // also set it as an in param
    paramBitRate.nPortIndex = PortIndexIn;
     ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_IndexParamVideoBitrate, (OMX_PTR)&paramBitRate);
    if(ret){
        M_ERROR("OMX_SetParameter of OMX_IndexParamVideoBitrate failed for in index!\n");
        _print_omx_error(ret);
        return OMX_ErrorUndefined;
    }

    // also do the config equivalent
    OMX_VIDEO_CONFIG_BITRATETYPE configBitRate;
    OMX_RESET_STRUCT_SIZE_VERSION(&configBitRate, OMX_VIDEO_CONFIG_BITRATETYPE);
    configBitRate.nPortIndex = PortIndexOut;
    configBitRate.nEncodeBitrate = bps;
    ret = OMX_SetConfig(m_OMXHandle, (OMX_INDEXTYPE)(OMX_IndexConfigVideoBitrate), (OMX_PTR)&configBitRate);
    if(ret){
        M_ERROR("OMX_SetParameter of OMX_IndexConfigVideoBitrate failed!\n");
        _print_omx_error(ret);
        return OMX_ErrorUndefined;
    }




    ////////////////////////////////////////////////////////////////////////////
    // QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE
    // https://android.googlesource.com/platform/hardware/qcom/sm7250/media/+/0aef9b5a7fda17e3fac441b565cd1fb4e37df0ff/mm-video-v4l2/vidc/venc/src/omx_video_extensions.hpp
    ////////////////////////////////////////////////////////////////////////////
#ifndef APQ8096
    QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE lowLatency;
    OMX_RESET_STRUCT_SIZE_VERSION(&lowLatency, QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE);
    lowLatency.bEnableLowLatencyMode = OMX_TRUE;
    ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_QTIIndexParamLowLatencyMode, (OMX_PTR)&lowLatency);
    if(ret){
        M_ERROR("%s Failed to set Low Latency parameter\n", __func__);
        _print_omx_error(ret);
        return ret;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////
    // OMX_QCOM_VIDEO_CONFIG_PERF_LEVEL
    // https://android.googlesource.com/platform/hardware/qcom/sm7250/media/+/0aef9b5a7fda17e3fac441b565cd1fb4e37df0ff/mm-video-v4l2/vidc/venc/src/omx_video_extensions.hpp
    ////////////////////////////////////////////////////////////////////////////
    OMX_QCOM_VIDEO_CONFIG_PERF_LEVEL perf;
    OMX_RESET_STRUCT_SIZE_VERSION(&perf, OMX_QCOM_VIDEO_CONFIG_PERF_LEVEL);
    perf.ePerfLevel = OMX_QCOM_PerfLevelTurbo;
    //perf.ePerfLevel = OMX_QCOM_PerfLevelNominal;
    ret = OMX_SetConfig(m_OMXHandle, (OMX_INDEXTYPE)(OMX_QcomIndexConfigPerfLevel), (OMX_PTR)&perf);
    if(ret){
        M_ERROR("%s Failed to set OMX Turbo Config\n", __func__);
        _print_omx_error(ret);
        return OMX_ErrorUndefined;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Configure Bitrate Savings (CAC)
    // https://android.googlesource.com/platform/hardware/qcom/sm7250/media/+/0aef9b5a7fda17e3fac441b565cd1fb4e37df0ff/mm-video-v4l2/vidc/venc/src/omx_video_extensions.hpp
    ////////////////////////////////////////////////////////////////////////////
#ifndef APQ8096
    OMX_U32 adaptive_coding = 0;
    ret = OMX_SetConfig(m_OMXHandle, (OMX_INDEXTYPE)OMX_QTIIndexConfigContentAdaptiveCoding, (OMX_PTR)&adaptive_coding);
    if(ret){
        M_ERROR("%s Failed to set OMX adaptive coding\n", __func__);
        _print_omx_error(ret);
        return OMX_ErrorUndefined;
    }


    QOMX_ENABLETYPE enInputQueue;
    OMX_RESET_STRUCT_SIZE_VERSION(&enInputQueue, QOMX_ENABLETYPE);
    enInputQueue.bEnable = OMX_TRUE;
    ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_QcomIndexParamVencControlInputQueue, (OMX_PTR)&enInputQueue);
    if(ret){
        M_ERROR("%s Failed to set enable input queue parameter\n", __func__);
        _print_omx_error(ret);
        return ret;
    }
#endif

    // more things to try:
    // OMX_QTIIndexParamColorSpaceConversion
    // OMX_QTIIndexConfigContentAdaptiveCoding



/*
    ////////////////////////////////////////////////////////////////////////////
    // OMX_VIDEO_PARAM_QUANTIZATIONTYPE
    // standard OMX quantization struct, doesn't seem to work
    // returns OMX_ErrorUnsupportedIndex
    ////////////////////////////////////////////////////////////////////////////
    OMX_VIDEO_PARAM_QUANTIZATIONTYPE quant;
    OMX_RESET_STRUCT_SIZE_VERSION(&quant, OMX_VIDEO_PARAM_QUANTIZATIONTYPE);
    initqp.nPortIndex = PortIndexOut;
    quant.nQpI = 35;
    quant.nQpP = 35;
    quant.nQpB = 35;
    ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_IndexParamQuantizationTable, (OMX_PTR)&quant);
    if(ret){
        M_ERROR("%s Failed to set Quantization Parameter\n", __func__);
        _print_omx_error(ret);
        return ret;
    }
*/


/*
    ////////////////////////////////////////////////////////////////////////
    // set OMX_QCOM_VIDEO_PARAM_PEAK_BITRATE
    // not working, returns OMX_ErrorUnsupportedIndex
    ////////////////////////////////////////////////////////////////////////
    OMX_QCOM_VIDEO_PARAM_PEAK_BITRATE peak_br;
    OMX_RESET_STRUCT_SIZE_VERSION(&peak_br, OMX_QCOM_VIDEO_PARAM_PEAK_BITRATE);
    peak_br.nPeakBitrate = m_VideoEncoderConfig.targetBitRate;
    ret = OMX_SetParameter(m_OMXHandle, (OMX_INDEXTYPE)OMX_QcomIndexParamPeakBitrate, (OMX_PTR)&peak_br);
    if(ret){
        M_ERROR("%s Failed to set Peak Bitrate parameter\n", __func__);
        _print_omx_error(ret);
        return ret;
    }

*/



    // Set Color aspect parameters
    // android::DescribeColorAspectsParams colorParams;
    // OMX_RESET_STRUCT(&colorParams, android::DescribeColorAspectsParams);
    // colorParams.nPortIndex = PortIndexIn;

    // if (OMX_GetConfig(m_OMXHandle, (OMX_INDEXTYPE)OMX_QTIIndexConfigDescribeColorAspects, (OMX_PTR)&colorParams))
    // {
    //     M_ERROR("OMX_GetConfig of OMX_QTIIndexConfigDescribeColorAspects failed!\n");
    //     return OMX_ErrorUndefined;
    // }
    // colorParams.sAspects.mPrimaries    = android::ColorAspects::PrimariesBT709_5;
    // colorParams.sAspects.mTransfer     = android::ColorAspects::TransferSMPTE170M;
    // colorParams.sAspects.mMatrixCoeffs = android::ColorAspects::MatrixBT709_5;

    // OMX_RESET_STRUCT_SIZE_VERSION(&colorParams, android::DescribeColorAspectsParams);

    // if (OMX_SetConfig(m_OMXHandle, (OMX_INDEXTYPE)OMX_QTIIndexConfigDescribeColorAspects, (OMX_PTR)&colorParams))
    // {
    //     M_ERROR("OMX_SetConfig of OMX_QTIIndexConfigDescribeColorAspects failed!\n");
    //     return OMX_ErrorUndefined;
    // }








    ////////////////////////////////////////////////////////////////////////////
    // Enable the port!
    ////////////////////////////////////////////////////////////////////////////
    ret = OMX_SendCommand(m_OMXHandle, OMX_CommandPortEnable, PortIndexIn, NULL);
    if(ret){
        M_ERROR("OMX failed to send enable command to in port\n");
        _print_omx_error(ret);
        return ret;
    }

    // Allocate input / output port buffers
    m_ppInputBuffers  = (OMX_BUFFERHEADERTYPE **)malloc(sizeof(OMX_BUFFERHEADERTYPE *) * m_inputBufferCount);
    m_ppOutputBuffers = (OMX_BUFFERHEADERTYPE **)malloc(sizeof(OMX_BUFFERHEADERTYPE *) * m_outputBufferCount);

    if ((m_ppInputBuffers == NULL) || (m_ppOutputBuffers == NULL))
    {
        M_ERROR("OMX Allocate OMX_BUFFERHEADERTYPE ** failed\n");
        return OMX_ErrorUndefined;
    }

    for (uint32_t i = 0; i < m_inputBufferCount; i++)
    {
        if(!m_VideoEncoderConfig.inputBuffers->bufferBlocks[i].vaddress)
        {
            M_WARN("Encoder expecting(%d) more buffers than module allocated(%d)\n", m_inputBufferCount, i);
            return OMX_ErrorUndefined;
        }
        // The OMX component i.e. the video encoder allocates the block, gets the memory from hal
        if (int ret = OMX_UseBuffer (m_OMXHandle, &m_ppInputBuffers[i], PortIndexIn, this, m_inputBufferSize,
                (OMX_U8*)m_VideoEncoderConfig.inputBuffers->bufferBlocks[i].vaddress))
        {
            M_ERROR("OMX_UseBuffer on input buffer: %d failed\n", i);
            OMXEventHandler(NULL, NULL, OMX_EventError, ret, 1, NULL);
            return OMX_ErrorUndefined;
        }
    }

    for (uint32_t i = 0; i < m_outputBufferCount; i++)
    {
        // The OMX component i.e. the video encoder allocates the memory residing behind these buffers
        if (OMX_AllocateBuffer (m_OMXHandle, &m_ppOutputBuffers[i], PortIndexOut, this, m_outputBufferSize))
        {
            M_ERROR("OMX_AllocateBuffer on output buffer: %d failed\n", i);
            return OMX_ErrorUndefined;
        }
    }

    if (OMX_SendCommand(m_OMXHandle, OMX_CommandStateSet, (OMX_U32)OMX_StateIdle, NULL))
    {
        M_ERROR("------voxl-camera-server ERROR: OMX_SendCommand OMX_StateIdle failed\n");\
        return OMX_ErrorUndefined;
    }

    return OMX_ErrorNone;
}

// -----------------------------------------------------------------------------------------------------------------------------
// This function sets the input or output port parameters and gets the input or output port buffer sizes and count to allocate
// -----------------------------------------------------------------------------------------------------------------------------
OMX_ERRORTYPE VideoEncoder::SetPortParams(OMX_U32  portIndex,               ///< In or Out port
                                          OMX_U32  bufferCountMin,          ///< Minimum number of buffers
                                          OMX_U32* pBufferSize,             ///< Returned buffer size
                                          OMX_U32* pBufferCount,            ///< Returned number of buffers
                                          OMX_COLOR_FORMATTYPE inputFormat) ///< Image format on the input port
{
    OMX_ERRORTYPE ret;
    OMX_PARAM_PORTDEFINITIONTYPE sPortDef;
    OMX_RESET_STRUCT(&sPortDef, OMX_PARAM_PORTDEFINITIONTYPE);
    if ((pBufferSize == NULL) || (pBufferCount == NULL)){
        M_ERROR("OMX Buffer error : NULL pointer\n");
        return OMX_ErrorBadParameter;
    }

    // get the default parameters for this index (in or out)
    sPortDef.nPortIndex = portIndex;
    ret = OMX_GetParameter(m_OMXHandle, OMX_IndexParamPortDefinition, (OMX_PTR)&sPortDef);
    if(ret){
        M_ERROR("OMX_GetParameter OMX_IndexParamPortDefinition failed!\n");
        return ret;
    }

    // change port definition settings that are for both in and out
    sPortDef.format.video.xFramerate = m_VideoEncoderConfig.frameRate << 16;
    sPortDef.format.video.nFrameWidth  = m_VideoEncoderConfig.width;
    sPortDef.format.video.nFrameHeight = m_VideoEncoderConfig.height;
    // sPortDef.format.video.nStride      = m_VideoEncoderConfig.width;
    // cheap hack to get stride from GBM data
#ifdef QRB5165
    sPortDef.format.video.nStride = m_VideoEncoderConfig.inputBuffers->bufferBlocks[0].stride;
#else
    sPortDef.format.video.nStride = m_VideoEncoderConfig.width;
#endif
    sPortDef.bEnabled = OMX_TRUE;
    sPortDef.format.video.nSliceHeight = 1024; // TODO play with this


    // set input/output specific settings
    if(portIndex == PortIndexIn){
        sPortDef.format.video.eColorFormat = inputFormat;
        sPortDef.bBuffersContiguous = OMX_TRUE; // ???
        sPortDef.format.video.eCompressionFormat =  OMX_VIDEO_CodingUnused;
    }
    else{
        sPortDef.format.video.eColorFormat =  OMX_COLOR_FormatUnused;
        // if you set this to 0 it's forced into constant Q mode nomatter what
        // we set up quality and bitrate later
        if(m_VideoEncoderConfig.venc_config.br_ctrl == VENC_CONTROL_CQP){
            sPortDef.format.video.nBitrate = 0;
        }else{
            sPortDef.format.video.nBitrate = m_VideoEncoderConfig.venc_config.mbps * 1000000;
        }
        sPortDef.bEnabled = OMX_TRUE;
        if(m_VideoEncoderConfig.venc_config.mode == VENC_H265){
            sPortDef.format.video.eCompressionFormat =  OMX_VIDEO_CodingHEVC;
        }else{
            sPortDef.format.video.eCompressionFormat =  OMX_VIDEO_CodingAVC;
        }
    }


    // now set our updated port definition
    ret = OMX_SetParameter(m_OMXHandle, OMX_IndexParamPortDefinition, (OMX_PTR)&sPortDef);
    if(ret){
        M_ERROR("OMX_SetParameter OMX_IndexParamPortDefinition failed!\n");
        return ret;
    }




    // now read them back and check the buffer count
    ret = OMX_GetParameter(m_OMXHandle, OMX_IndexParamPortDefinition, (OMX_PTR)&sPortDef);
    if (ret != OMX_ErrorNone) {
        M_ERROR("Error: GET OMX_IndexParamPortDefinition (second get)\n");
        return ret;
    }

    if (bufferCountMin < sPortDef.nBufferCountMin){
        bufferCountMin = sPortDef.nBufferCountMin;
    }

    sPortDef.nBufferCountActual = bufferCountMin;
    // sPortDef.nBufferCountMin    = bufferCountMin;
    M_DEBUG("Buffer Count Expected: %d\n", sPortDef.nBufferCountActual);

    // now write it back with the updated buffer count
    if (OMX_SetParameter(m_OMXHandle, OMX_IndexParamPortDefinition, (OMX_PTR)&sPortDef)){
        M_ERROR("OMX_SetParameter OMX_IndexParamPortDefinition failed!\n");
        return OMX_ErrorUndefined;
    }

    if (OMX_GetParameter(m_OMXHandle, OMX_IndexParamPortDefinition, (OMX_PTR)&sPortDef)){
        M_ERROR("------voxl-camera-server ERROR: OMX_GetParameter OMX_IndexParamPortDefinition failed!\n");
        return OMX_ErrorUndefined;
    }
    M_DEBUG("Buffer Count Actual: %d\n", sPortDef.nBufferCountActual);

    if(bufferCountMin != sPortDef.nBufferCountActual){
        M_ERROR("Failed to get correct number of buffers from OMX module, expected: %d got: %d\n", bufferCountMin, sPortDef.nBufferCountActual);
        return OMX_ErrorUndefined;
    }

    M_WARN("Port Def %d:\n\tCount Min: %d\n\tCount Actual: %d\n\tSize: 0x%x\n\tBuffers Contiguous: %s\n\tBuffer Alignment: %d\n",
        portIndex,
        sPortDef.nBufferCountMin,
        sPortDef.nBufferCountActual,
        sPortDef.nBufferSize,
        sPortDef.bBuffersContiguous ? "Yes" : "No",
        sPortDef.nBufferAlignment
        );

    *pBufferCount = sPortDef.nBufferCountActual;
    *pBufferSize  = sPortDef.nBufferSize;

    return OMX_ErrorNone;
}

int VideoEncoder::ItemsInQueue()
{
    return out_metaQueue.size();
}

// -----------------------------------------------------------------------------------------------------------------------------
// The client calls this interface function to pass in a YUV image frame to be encoded
// -----------------------------------------------------------------------------------------------------------------------------
void VideoEncoder::ProcessFrameToEncode(camera_image_metadata_t meta, BufferBlock* buffer)
{
    pthread_mutex_lock(&out_mutex);
    // Queue up work for thread "ThreadProcessOMXOutputPort"
    out_metaQueue.push_back(meta);
    //pthread_cond_signal(&out_cond);
    pthread_mutex_unlock(&out_mutex);

    OMX_BUFFERHEADERTYPE* OMXBuffer = NULL;
    for(unsigned int i = 0; (OMXBuffer = m_ppInputBuffers[i])->pBuffer != buffer->vaddress; i++) {
        //M_VERBOSE("Encoder Buffer Miss\n");
        if(i == m_pHALInputBuffers->totalBuffers - 1){
            M_ERROR("Encoder did not find omx-ready buffer for buffer: 0x%lx, skipping encoding\n", buffer->vaddress);
            return;
        }
    }
    M_VERBOSE("Encoder Buffer Hit\n");

    // QRB Testing
    // 4096x2160
    // OMXBuffer->nFilledLen = buffer->width * (buffer->height + 267) * 3 / 2;
    // 2048x1536
    // OMXBuffer->nFilledLen = buffer->width * buffer->height * 3 / 2;
    // 1024x768
    // OMXBuffer->nFilledLen = buffer->width * (buffer->height + 171) * 3 / 2;

    #ifdef QRB5165
        OMXBuffer->nFilledLen = buffer->size;
    #else
        int offset = 0;
        switch (buffer->height) {
            case 720 : offset = 16; break;
            case 1080: offset = 8;  break;
        }
        OMXBuffer->nFilledLen = buffer->width * (buffer->height + offset) * 3 / 2;
    // bufferMakeYUVContiguous(buffer);
    #endif

    OMXBuffer->nTimeStamp = meta.timestamp_ns;

    if (OMX_EmptyThisBuffer(m_OMXHandle, OMXBuffer))
    {
        M_ERROR("OMX_EmptyThisBuffer failed for framebuffer: %d\n", meta.frame_id);
    }

    M_VERBOSE("OMX emptied a buffer\n");
}

// -----------------------------------------------------------------------------------------------------------------------------
// This function performs any work necessary to start receiving encoding frames from the client
// -----------------------------------------------------------------------------------------------------------------------------
void VideoEncoder::Start()
{
    pthread_attr_t resultAttr;
    pthread_attr_init(&resultAttr);
    pthread_attr_setdetachstate(&resultAttr, PTHREAD_CREATE_JOINABLE);
    pthread_create(&out_thread,
                   &resultAttr,
                   [](void* data){return ((VideoEncoder*)data)->ThreadProcessOMXOutputPort();},
                   this);

    pthread_attr_destroy(&resultAttr);
}

// -----------------------------------------------------------------------------------------------------------------------------
// This function is called by the client to indicate that no more frames will be sent for encoding
// -----------------------------------------------------------------------------------------------------------------------------
void VideoEncoder::Stop()
{
    stop  = true;
    usleep(500000);
    stop_read = true;
    // The thread wont finish and the "join" call will not return till the last expected encoded frame is received from
    // the encoder OMX component
    pthread_cond_signal(&out_cond);

    pthread_join(out_thread, NULL);

    pthread_mutex_destroy(&out_mutex);
    pthread_cond_destroy(&out_cond);

     unsigned int i;

    if(OMX_SendCommand(m_OMXHandle, OMX_CommandStateSet, (OMX_U32)OMX_StateIdle, NULL)){
        M_ERROR("OMX Set state idle failed!\n");
        throw -EINVAL;
    }

    for(i=0; i<m_inputBufferCount; i++){
        OMX_FreeBuffer(m_OMXHandle, PortIndexIn, m_ppInputBuffers[i]);
    }

    free(m_ppInputBuffers);

    for (i=0; i<m_outputBufferCount; i++){
        OMX_FreeBuffer(m_OMXHandle, PortIndexOut, m_ppOutputBuffers[i]);
    }

    free(m_ppOutputBuffers);

    //OMX_FreeHandle(m_OMXHandle);
    OMXDeinit();

    // if (m_OMXHandle != NULL)
    // {
    //     OMXFreeHandle(m_OMXHandle);
    //     m_OMXHandle = NULL;
    // }

}



// -----------------------------------------------------------------------------------------------------------------------------
// Function called by the OMX component for event handling
// -----------------------------------------------------------------------------------------------------------------------------
OMX_ERRORTYPE OMXEventHandler(OMX_IN OMX_HANDLETYPE hComponent,     ///< OMX component handle
                              OMX_IN OMX_PTR        pAppData,       ///< Any private app data
                              OMX_IN OMX_EVENTTYPE  eEvent,         ///< Event identifier
                              OMX_IN OMX_U32        nData1,         ///< Data 1
                              OMX_IN OMX_U32        nData2,         ///< Data 2
                              OMX_IN OMX_PTR        pEventData)     ///< Event data
{
    switch (eEvent) {
        case OMX_EventCmdComplete:
            M_DEBUG("OMX_EventCmdComplete\n");
            break;
        case OMX_EventError:
            M_DEBUG("OMX_EventError: ");
           _print_omx_error((OMX_ERRORTYPE)nData1);
            break;
        case OMX_EventMark:
            M_DEBUG("OMX Event: OMX_EventMark\n");
            break;
        case OMX_EventPortSettingsChanged:
            M_DEBUG("OMX Event: OMX_EventPortSettingsChanged\n");
            break;
        case OMX_EventBufferFlag:
            M_DEBUG("OMX Event: OMX_EventBufferFlag\n");
            break;
        case OMX_EventResourcesAcquired:
            M_DEBUG("OMX Event: OMX_EventResourcesAcquired\n");
            break;
        case OMX_EventComponentResumed:
            M_DEBUG("OMX Event: OMX_EventComponentResumed\n");
            break;
        case OMX_EventDynamicResourcesAvailable:
            M_DEBUG("OMX Event: OMX_EventDynamicResourcesAvailable\n");
            break;
        case OMX_EventPortFormatDetected:
            M_DEBUG("OMX Event: OMX_EventPortFormatDetected\n");
            break;
        case OMX_EventKhronosExtensions:
            M_DEBUG("OMX Event: OMX_EventKhronosExtensions\n");
            break;
        case OMX_EventVendorStartUnused:
            M_DEBUG("OMX Event: OMX_EventVendorStartUnused\n");
            break;
        case OMX_EventMax:
            M_DEBUG("OMX Event: OMX_EventMax\n");
            break;
        default:
            M_DEBUG("OMX Event: Unknown\n");
            break;

    }
    return OMX_ErrorNone;
}

// -----------------------------------------------------------------------------------------------------------------------------
// Function called by the OMX component indicating it has completed consuming our YUV frame for encoding
// -----------------------------------------------------------------------------------------------------------------------------
OMX_ERRORTYPE OMXEmptyBufferHandler(OMX_IN OMX_HANDLETYPE        hComponent,    ///< OMX component handle
                                    OMX_IN OMX_PTR               pAppData,      ///< Any private app data
                                    OMX_IN OMX_BUFFERHEADERTYPE* pBuffer)       ///< Buffer that has been emptied
{

    VideoEncoder*  pVideoEncoder = (VideoEncoder*)pAppData;
    bufferPushAddress(*pVideoEncoder->m_pHALInputBuffers, pBuffer->pBuffer);

    return OMX_ErrorNone;
}

// -----------------------------------------------------------------------------------------------------------------------------
// Function called by the OMX component to give us the encoded frame. Since this is a callback we dont do much work but simply
// prepare work that will be done by the worker threads
// -----------------------------------------------------------------------------------------------------------------------------
OMX_ERRORTYPE OMXFillHandler(OMX_OUT OMX_HANDLETYPE        hComponent,  ///< OMX component handle
                             OMX_OUT OMX_PTR               pAppData,    ///< Any private app data
                             OMX_OUT OMX_BUFFERHEADERTYPE* pBuffer)     ///< Buffer that has been filled by OMX component
{
    VideoEncoder*  pVideoEncoder = (VideoEncoder*)pAppData;
    if(pVideoEncoder->stop) return OMX_ErrorNone;
    pthread_mutex_lock(&pVideoEncoder->out_mutex);
    // Queue up work for thread "ThreadProcessOMXOutputPort"
    pVideoEncoder->out_msgQueue.push_back(pBuffer);
    pthread_cond_signal(&pVideoEncoder->out_cond);
    pthread_mutex_unlock(&pVideoEncoder->out_mutex);

    return OMX_ErrorNone;
}

// -----------------------------------------------------------------------------------------------------------------------------
// This thread function processes the encoded buffers available on the OMX component's output port
// -----------------------------------------------------------------------------------------------------------------------------
void* VideoEncoder::ThreadProcessOMXOutputPort()
{
    pthread_setname_np(pthread_self(), "omx_out");

    int64_t frameNumber = -1;

    // The condition of the while loop is such that this thread will not terminate till it receives the last expected encoded
    // frame from the OMX component
    while (!stop_read)
    {
        pthread_mutex_lock(&out_mutex);

        if (out_msgQueue.empty())
        {
            pthread_cond_wait(&out_cond, &out_mutex);
            pthread_mutex_unlock(&out_mutex);
            continue;
        }

        if(out_metaQueue.empty()){
            M_WARN("Trying to process omx output with missing metadata\n");
            //pthread_cond_wait(&out_cond, &out_mutex);
            pthread_mutex_unlock(&out_mutex);
            continue;
        }

        if(m_outputPipe==NULL){
            M_WARN("Trying to process omx output without initialized pipe\n");
            pthread_cond_wait(&out_cond, &out_mutex);
            pthread_mutex_unlock(&out_mutex);
            continue;
        }


        // Coming here means we have a encoded frame to process
        OMX_BUFFERHEADERTYPE* pOMXBuffer = out_msgQueue.front();
        out_msgQueue.pop_front();


        camera_image_metadata_t meta     = out_metaQueue.front();
        // h264 metadata packet, don't associate it with a frame
        if(m_VideoEncoderConfig.venc_config.mode == VENC_H265){
            if(pOMXBuffer->pBuffer[4] != 0x40){
                out_metaQueue.pop_front();
            } else {
                meta.frame_id = -1;
            }
        } else {
            if(pOMXBuffer->pBuffer[4] != 0x67){
                out_metaQueue.pop_front();
            } else {
                meta.frame_id = -1;
            } 
        }

        pthread_mutex_unlock(&out_mutex);
        frameNumber = meta.frame_id;

        meta.size_bytes = pOMXBuffer->nFilledLen;
        if(m_VideoEncoderConfig.venc_config.mode == VENC_H265){
            meta.format = IMAGE_FORMAT_H265;
        }else{
            meta.format = IMAGE_FORMAT_H264;
        }

        pipe_server_write_camera_frame(*m_outputPipe, meta, pOMXBuffer->pBuffer);
        M_VERBOSE("Sent encoded frame: %d\n", frameNumber);

        // Since we processed the OMX buffer we can immediately recycle it by
        // sending it to the output port of the OMX component
        // reset filled len
        pOMXBuffer->nFilledLen=0;
        if (OMX_FillThisBuffer(m_OMXHandle, pOMXBuffer))
        {
            M_ERROR("OMX_FillThisBuffer resulted in error for frame %d\n", frameNumber);
        }
    }

    M_DEBUG("------ Last frame encoded: %d\n", frameNumber);

    return NULL;
}
