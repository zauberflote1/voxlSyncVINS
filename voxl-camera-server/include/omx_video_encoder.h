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
#ifndef VOXL_CAMERA_SERVER_VIDEO_ENCODER
#define VOXL_CAMERA_SERVER_VIDEO_ENCODER

#include <list>
#include <OMX_Core.h>
#include <OMX_IVCommon.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <buffer_manager.h>
#include "common_defs.h"



// -----------------------------------------------------------------------------------------------------------------------------
// Video encoder config data
// -----------------------------------------------------------------------------------------------------------------------------
typedef struct VideoEncoderConfig
{
    uint32_t width;                 ///< Image width
    uint32_t height;                ///< Image height
    uint32_t format;                ///< Image format
    venc_config_t venc_config;
    int32_t  frameRate;             ///< Frame rate
    BufferGroup* inputBuffers;      ///< Input buffers coming from hal3
    int*     outputPipe;            ///< Pre-configured MPA output pipe
} VideoEncoderConfig;

//------------------------------------------------------------------------------------------------------------------------------
// Main interface class that interacts with the OMX Encoder component and the Camera Manager class. At the crux of it, this
// class takes the YUV frames from the camera and passes it to the OMX component for encoding. It gets the final encoded frames
// in h264 or h265 format
//------------------------------------------------------------------------------------------------------------------------------
class VideoEncoder
{
public:
    VideoEncoder(VideoEncoderConfig* pVideoEncoderConfig);
    ~VideoEncoder();

    // Do any necessary work to start receiving encoding frames from the client
    void Start();
    // This call indicates that no more frames will be sent for encoding
    void Stop();
    // Client of this encoder class calls this function to pass in the YUV video frame to be encoded
    void ProcessFrameToEncode(camera_image_metadata_t meta, BufferBlock* buffer);

    void* ThreadProcessOMXOutputPort();

    int ItemsInQueue(); // return how many frames are still in the process queue

    // Set the OMX component configuration
    OMX_ERRORTYPE SetConfig(void);
    // Set input / output port parameters
    OMX_ERRORTYPE SetPortParams(OMX_U32  portIndex,
                                OMX_U32  bufferCountMin,
                                OMX_U32* pBufferSize,
                                OMX_U32* pBufferCount,
                                OMX_COLOR_FORMATTYPE format);

    static const OMX_U32  PortIndexIn          = 0;
    static const OMX_U32  PortIndexOut         = 1;

    pthread_t              out_thread;              ///< Out thread
    pthread_mutex_t        out_mutex;               ///< Out thread Mutex for list access
    pthread_cond_t         out_cond;                ///< Out thread Condition variable for wake up
    std::list<OMX_BUFFERHEADERTYPE*>      out_msgQueue;            ///< Out thread Message queue
    std::list<camera_image_metadata_t>    out_metaQueue;           ///< Out thread Message queue

    volatile bool          stop = false;            ///< Thread terminate indicator
    volatile bool          stop_read = false;            ///< read thread terminate indicator

    VideoEncoderConfig     m_VideoEncoderConfig;
    int*                   m_outputPipe;
    uint32_t               m_inputBufferSize;       ///< Input buffer size
    uint32_t               m_inputBufferCount;      ///< Input buffer count
    uint32_t               m_outputBufferSize;      ///< Output buffer size
    uint32_t               m_outputBufferCount;     ///< Output buffer count
    OMX_HANDLETYPE         m_OMXHandle = NULL;      ///< OMX component handle
    BufferGroup*           m_pHALInputBuffers;
    OMX_BUFFERHEADERTYPE** m_ppInputBuffers;        ///< Input buffers
    uint32_t               m_nextInputBufferIndex;  ///< Next input buffer to use
    OMX_BUFFERHEADERTYPE** m_ppOutputBuffers;       ///< Output buffers
    uint32_t               m_nextOutputBufferIndex; ///< Next input buffer to use
};



static void _print_omx_error(OMX_ERRORTYPE e)
{
    switch(e) {
        case OMX_ErrorNone:
            M_ERROR("OMX_ErrorNone\n");
            break;
        case OMX_ErrorInsufficientResources:
            M_ERROR("OMX_ErrorInsufficientResources\n");
            break;
        case OMX_ErrorUndefined:
            M_ERROR("OMX_ErrorUndefined\n");
            break;
        case OMX_ErrorInvalidComponentName:
            M_ERROR("OMX_ErrorInvalidComponentName\n");
            break;
        case OMX_ErrorComponentNotFound:
            M_ERROR("OMX_ErrorComponentNotFound\n");
            break;
        case OMX_ErrorInvalidComponent:
            M_ERROR("OMX_ErrorInvalidComponent\n");
            break;
        case OMX_ErrorBadParameter:
            M_ERROR("OMX_ErrorBadParameter\n");
            break;
        case OMX_ErrorNotImplemented:
            M_ERROR("OMX_ErrorNotImplemented\n");
            break;
        case OMX_ErrorUnderflow:
            M_ERROR("OMX_ErrorUnderflow\n");
            break;
        case OMX_ErrorOverflow:
            M_ERROR("OMX_ErrorOverflow\n");
            break;
        case OMX_ErrorHardware:
            M_ERROR("OMX_ErrorHardware\n");
            break;
        case OMX_ErrorInvalidState:
            M_ERROR("OMX_ErrorInvalidState\n");
            break;
        case OMX_ErrorStreamCorrupt:
            M_ERROR("OMX_ErrorStreamCorrupt\n");
            break;
        case OMX_ErrorPortsNotCompatible:
            M_ERROR("OMX_ErrorPortsNotCompatible\n");
            break;
        case OMX_ErrorResourcesLost:
            M_ERROR("OMX_ErrorResourcesLost\n");
            break;
        case OMX_ErrorNoMore:
            M_ERROR("OMX_ErrorNoMore\n");
            break;
        case OMX_ErrorVersionMismatch:
            M_ERROR("OMX_ErrorVersionMismatch\n");
            break;
        case OMX_ErrorNotReady:
            M_ERROR("OMX_ErrorNotReady\n");
            break;
        case OMX_ErrorTimeout:
            M_ERROR("OMX_ErrorTimeout\n");
            break;
        case OMX_ErrorSameState:
            M_ERROR("OMX_ErrorSameState\n");
            break;
        case OMX_ErrorResourcesPreempted:
            M_ERROR("OMX_ErrorResourcesPreempted\n");
            break;
        case OMX_ErrorPortUnresponsiveDuringAllocation:
            M_ERROR("OMX_ErrorPortUnresponsiveDuringAllocation\n");
            break;
        case OMX_ErrorPortUnresponsiveDuringDeallocation:
            M_ERROR("OMX_ErrorPortUnresponsiveDuringDeallocation\n");
            break;
        case OMX_ErrorPortUnresponsiveDuringStop:
            M_ERROR("OMX_ErrorPortUnresponsiveDuringStop\n");
            break;
        case OMX_ErrorIncorrectStateTransition:
            M_ERROR("OMX_ErrorIncorrectStateTransition\n");
            break;
        case OMX_ErrorIncorrectStateOperation:
            M_ERROR("OMX_ErrorIncorrectStateOperation\n");
            break;
        case OMX_ErrorUnsupportedSetting:
            M_ERROR("OMX_ErrorUnsupportedSetting\n");
            break;
        case OMX_ErrorUnsupportedIndex:
            M_ERROR("OMX_ErrorUnsupportedIndex\n");
            break;
        case OMX_ErrorBadPortIndex:
            M_ERROR("OMX_ErrorBadPortIndex\n");
            break;
        case OMX_ErrorPortUnpopulated:
            M_WARN("OMX_ErrorPortUnpopulated\n");
            break;
        case OMX_ErrorComponentSuspended:
            M_ERROR("OMX_ErrorComponentSuspended\n");
            break;
        case OMX_ErrorDynamicResourcesUnavailable:
            M_ERROR("OMX_ErrorDynamicResourcesUnavailable\n");
            break;
        case OMX_ErrorMbErrorsInFrame:
            M_ERROR("OMX_ErrorMbErrorsInFrame\n");
            break;
        case OMX_ErrorFormatNotDetected:
            M_ERROR("OMX_ErrorFormatNotDetected\n");
            break;
        case OMX_ErrorContentPipeOpenFailed:
            M_ERROR("OMX_ErrorContentPipeOpenFailed\n");
            break;
        case OMX_ErrorContentPipeCreationFailed:
            M_ERROR("OMX_ErrorContentPipeCreationFailed\n");
            break;
        case OMX_ErrorSeperateTablesUsed:
            M_ERROR("OMX_ErrorSeperateTablesUsed\n");
            break;
        case OMX_ErrorTunnelingUnsupported:
            M_ERROR("OMX_ErrorTunnelingUnsupported\n");
            break;
        case OMX_ErrorKhronosExtensions:
            M_ERROR("OMX_ErrorKhronosExtensions\n");
            break;
        case OMX_ErrorVendorStartUnused:
            M_ERROR("OMX_ErrorVendorStartUnused\n");
            break;
        case OMX_ErrorMax:
            M_ERROR("OMX_ErrorMax\n");
            break;
    }
    return;
}




static const char * colorFormatStr(OMX_COLOR_FORMATTYPE fmt) {
    switch (fmt){
        case OMX_COLOR_FormatUnused:
            return "OMX_COLOR_FormatUnused";
        case OMX_COLOR_FormatMonochrome:
            return "OMX_COLOR_FormatMonochrome";
        case OMX_COLOR_Format8bitRGB332:
            return "OMX_COLOR_Format8bitRGB332";
        case OMX_COLOR_Format12bitRGB444:
            return "OMX_COLOR_Format12bitRGB444";
        case OMX_COLOR_Format16bitARGB4444:
            return "OMX_COLOR_Format16bitARGB4444";
        case OMX_COLOR_Format16bitARGB1555:
            return "OMX_COLOR_Format16bitARGB1555";
        case OMX_COLOR_Format16bitRGB565:
            return "OMX_COLOR_Format16bitRGB565";
        case OMX_COLOR_Format16bitBGR565:
            return "OMX_COLOR_Format16bitBGR565";
        case OMX_COLOR_Format18bitRGB666:
            return "OMX_COLOR_Format18bitRGB666";
        case OMX_COLOR_Format18bitARGB1665:
            return "OMX_COLOR_Format18bitARGB1665";
        case OMX_COLOR_Format19bitARGB1666:
            return "OMX_COLOR_Format19bitARGB1666";
        case OMX_COLOR_Format24bitRGB888:
            return "OMX_COLOR_Format24bitRGB888";
        case OMX_COLOR_Format24bitBGR888:
            return "OMX_COLOR_Format24bitBGR888";
        case OMX_COLOR_Format24bitARGB1887:
            return "OMX_COLOR_Format24bitARGB1887";
        case OMX_COLOR_Format25bitARGB1888:
            return "OMX_COLOR_Format25bitARGB1888";
        case OMX_COLOR_Format32bitBGRA8888:
            return "OMX_COLOR_Format32bitBGRA8888";
        case OMX_COLOR_Format32bitARGB8888:
            return "OMX_COLOR_Format32bitARGB8888";
        case OMX_COLOR_FormatYUV411Planar:
            return "OMX_COLOR_FormatYUV411Planar";
        case OMX_COLOR_FormatYUV411PackedPlanar:
            return "OMX_COLOR_FormatYUV411PackedPlanar";
        case OMX_COLOR_FormatYUV420Planar:
            return "OMX_COLOR_FormatYUV420Planar";
        case OMX_COLOR_FormatYUV420PackedPlanar:
            return "OMX_COLOR_FormatYUV420PackedPlanar";
        case OMX_COLOR_FormatYUV420SemiPlanar:
            return "OMX_COLOR_FormatYUV420SemiPlanar";
        case OMX_COLOR_FormatYUV422Planar:
            return "OMX_COLOR_FormatYUV422Planar";
        case OMX_COLOR_FormatYUV422PackedPlanar:
            return "OMX_COLOR_FormatYUV422PackedPlanar";
        case OMX_COLOR_FormatYUV422SemiPlanar:
            return "OMX_COLOR_FormatYUV422SemiPlanar";
        case OMX_COLOR_FormatYCbYCr:
            return "OMX_COLOR_FormatYCbYCr";
        case OMX_COLOR_FormatYCrYCb:
            return "OMX_COLOR_FormatYCrYCb";
        case OMX_COLOR_FormatCbYCrY:
            return "OMX_COLOR_FormatCbYCrY";
        case OMX_COLOR_FormatCrYCbY:
            return "OMX_COLOR_FormatCrYCbY";
        case OMX_COLOR_FormatYUV444Interleaved:
            return "OMX_COLOR_FormatYUV444Interleaved";
        case OMX_COLOR_FormatRawBayer8bit:
            return "OMX_COLOR_FormatRawBayer8bit";
        case OMX_COLOR_FormatRawBayer10bit:
            return "OMX_COLOR_FormatRawBayer10bit";
        case OMX_COLOR_FormatRawBayer8bitcompressed:
            return "OMX_COLOR_FormatRawBayer8bitcompressed";
        case OMX_COLOR_FormatL2:
            return "OMX_COLOR_FormatL2";
        case OMX_COLOR_FormatL4:
            return "OMX_COLOR_FormatL4";
        case OMX_COLOR_FormatL8:
            return "OMX_COLOR_FormatL8";
        case OMX_COLOR_FormatL16:
            return "OMX_COLOR_FormatL16";
        case OMX_COLOR_FormatL24:
            return "OMX_COLOR_FormatL24";
        case OMX_COLOR_FormatL32:
            return "OMX_COLOR_FormatL32";
        case OMX_COLOR_FormatYUV420PackedSemiPlanar:
            return "OMX_COLOR_FormatYUV420PackedSemiPlanar";
        case OMX_COLOR_FormatYUV422PackedSemiPlanar:
            return "OMX_COLOR_FormatYUV422PackedSemiPlanar";
        case OMX_COLOR_Format18BitBGR666:
            return "OMX_COLOR_Format18BitBGR666";
        case OMX_COLOR_Format24BitARGB6666:
            return "OMX_COLOR_Format24BitARGB6666";
        case OMX_COLOR_Format24BitABGR6666:
            return "OMX_COLOR_Format24BitABGR6666";
        case OMX_COLOR_FormatKhronosExtensions:
            return "OMX_COLOR_FormatKhronosExtensions";
        case OMX_COLOR_FormatVendorStartUnused:
            return "OMX_COLOR_FormatVendorStartUnused";
        case OMX_COLOR_FormatAndroidOpaque:
            return "OMX_COLOR_FormatAndroidOpaque";
    // QRB only???
    #ifdef QRB5165
        case OMX_COLOR_Format32BitRGBA8888:
            return "OMX_COLOR_Format32BitRGBA8888";
        case OMX_COLOR_FormatYUV420Flexible:
            return "OMX_COLOR_FormatYUV420Flexible";
        case OMX_COLOR_FormatYUV420Planar16:
            return "OMX_COLOR_FormatYUV420Planar16";
        case OMX_COLOR_FormatYUV444Y410:
            return "OMX_COLOR_FormatYUV444Y410";
    #endif
        case OMX_TI_COLOR_FormatYUV420PackedSemiPlanar:
            return "OMX_TI_COLOR_FormatYUV420PackedSemiPlanar";
        case OMX_QCOM_COLOR_FormatYVU420SemiPlanar:
            return "OMX_QCOM_COLOR_FormatYVU420SemiPlanar";
        case OMX_QCOM_COLOR_FormatYUV420PackedSemiPlanar64x32Tile2m8ka:
            return "OMX_QCOM_COLOR_FormatYUV420PackedSemiPlanar64x32Tile2m8ka";
        case OMX_SEC_COLOR_FormatNV12Tiled:
            return "OMX_SEC_COLOR_FormatNV12Tiled";
        case OMX_QCOM_COLOR_FormatYUV420PackedSemiPlanar32m:
            return "OMX_QCOM_COLOR_FormatYUV420PackedSemiPlanar32m";
        case OMX_COLOR_FormatMax:
            return "OMX_COLOR_FormatMax";
        default:
            return "Unknown";
    }
}



#endif // VOXL_CAMERA_SERVER_VIDEO_ENCODER
