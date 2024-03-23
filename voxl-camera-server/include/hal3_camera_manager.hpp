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

#ifndef VOXL_HAL3_CAMERA_MANAGER
#define VOXL_HAL3_CAMERA_MANAGER

#include <camera/CameraMetadata.h>
#include <hardware/camera3.h>
#include <list>
#include <queue>
#include <string>
#include <modal_pipe.h>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "ringbuffer.h"
#include "buffer_manager.h"
#include "common_defs.h"
#include "exposure-hist.h"
#include "exposure-msv.h"
#include "omx_video_encoder.h"
#include "tof_interface.hpp"


//HAL3 will lag the framerate if we attempt autoexposure any more frequently than this
#define NUM_SKIPPED_FRAMES 4

using namespace std;
using std::queue;


#ifdef APQ8096
using namespace modalai;
#endif

// Forward Declaration
class BufferManager;
class PerCameraMgr;

//------------------------------------------------------------------------------------------------------------------------------
// Everything needed to handle a single camera
//------------------------------------------------------------------------------------------------------------------------------
class PerCameraMgr : public IRoyaleDataListener
{
public:
    PerCameraMgr() ;
    PerCameraMgr(PerCameraInfo perCameraInfo);
    ~PerCameraMgr();

    // Callback function for the TOF bridge to provide the post processed TOF data
    bool RoyaleDataDone(const void*             pData,
                        uint32_t                size,
                        int64_t                 timestamp,
                        RoyaleListenerType      dataType);

    // Start the camera so that it starts streaming frames
    void Start();
    // Stop the camera and stop sending any more requests to the camera module
    void Stop();
    void EStop();


    const PerCameraInfo        configInfo;                     ///< Per camera config information
    const int32_t              cameraId;                       ///< Camera id
          char                 name[MAXNAMELEN];
    const bool                 en_preview;
    const bool                 en_small_video;
    const bool                 en_large_video;
    const bool                 en_snapshot;
    const int32_t              fps;                              ///< FPS
    const int32_t              pre_width;                        ///< Preview Width
    const int32_t              pre_height;                       ///< Preview Height
    const int32_t              pre_halfmt;                       ///< Preview HAL format
    const int32_t              vid_halfmt;                       ///< hal format for video
    const int32_t              small_video_width;                ///< Stream Width
    const int32_t              small_video_height;               ///< Stream Height
    const int32_t              large_video_width;                ///< Record Width
    const int32_t              large_video_height;               ///< Record Height
    const int32_t              snap_width;                       ///< Snapshot Width
    const int32_t              snap_height;                      ///< Snapshot Height
    const int32_t              snap_halfmt;                      ///< Snapshot HAL format
          AE_MODE              ae_mode;                          ///< AE mode

private:

    void* ThreadPostProcessResult();
    void* ThreadIssueCaptureRequests();

    // Call the camera module and pass it the stream configuration
    int  ConfigureStreams();
    // Initialize the MPA pipes
    int  SetupPipes();
    void HandleControlCmd(char* cmd);
    void SetupRawProcessing();

    // Call the camera module to get the default camera settings
    int ConstructDefaultRequestSettings();

    // called by SendOneCaptureRequest to decide if we need to send requests
    // for the preview stream, snapshot, record, and stream streams handled separately
    int HasClientForPreviewFrame();
    int HasClientForSmallVideo();
    int HasClientForLargeVideo();


    // Send one capture request to the camera module
    int  SendOneCaptureRequest(uint32_t* frameNumber);

    typedef std::pair<int, camera3_stream_buffer> image_result;

    void ProcessPreviewFrame (image_result result);
    void ProcessTOFPreviewFrame(BufferBlock* bufferBlockInfo, camera_image_metadata_t meta);
    void publishRAWPreviewFrame(BufferBlock* bufferBlockInfo, camera_image_metadata_t meta);
    void publishISPFrame(BufferBlock const& b, camera_image_metadata_t& meta, int32_t fmt,
                      int grey_pipe, int color_pipe);
    void publishStereoRAWPreviewFrame(camera_image_metadata_t meta);
    void publishStereoISPFrame(BufferBlock* bufferBlockInfo, camera_image_metadata_t meta);
    int  handleStereoSync(camera_image_metadata_t* meta);

    void ProcessSmallVideoFrame  (image_result result);
    void ProcessLargeVideoFrame  (image_result result);
    void ProcessSnapshotFrame(image_result result);
    void runModalExposure(uint8_t* grey_ptr, camera_image_metadata_t meta);

    int getMeta(int frameNumber, camera_image_metadata_t *retMeta){
        for(camera_image_metadata_t c : resultMetaRing){
            // fprintf(stderr, "%s, %d : %d - %d\n", __FUNCTION__, __LINE__, frameNumber, c.frame_id );

            if(c.frame_id == frameNumber){
                *retMeta = c;
                return 0;
            }
        }
        return -1;
    }

    int getAllResult(int frameNumber, image_result result_list[]){
        int count = 0;
        for(image_result r : resultMsgRing){
            // fprintf(stderr, "%s, %d : %d - %d\n", __FUNCTION__, __LINE__, frameNumber, c.frame_id );

            if(r.first == frameNumber) {
                result_list[count] = r;
                count++;
            }
        }
        return count;
    }

    // camera3_callback_ops is returned to us in every result callback. We piggy back any private information we may need at
    // the time of processing the frame result. When we register the callbacks with the camera module, we register the starting
    // address of this structure (which is camera3_callbacks_ops) but followed by our private information. When we receive a
    // pointer to this structure at the time of capture result, we typecast the incoming pointer to this structure type pointer
    // and access our private information
    struct Camera3Callbacks
    {
        camera3_callback_ops cameraCallbacks;
        void* pPrivate;
    };

    void ProcessOneCaptureResult(const camera3_capture_result* pHalResult);
    static void CameraModuleCaptureResult(const camera3_callback_ops_t *cb, const camera3_capture_result* pHalResult);
    static void CameraModuleNotify(const camera3_callback_ops_t *cb, const camera3_notify_msg_t *msg);

    enum PCM_MODE {
        MODE_MONO,
        MODE_STEREO_MASTER,
        MODE_STEREO_SLAVE
    };

    enum STREAM_ID {
        STREAM_PREVIEW,
        STREAM_SMALL_VID,
        STREAM_LARGE_VID,
        STREAM_SNAPSHOT,
        STREAM_INVALID
    };

    STREAM_ID GetStreamId(camera3_stream_t *stream){
        if (stream == &pre_stream) {
            return STREAM_PREVIEW;
        } else if (stream == &small_vid_stream) {
            return STREAM_SMALL_VID;
        } else if (stream == &large_vid_stream) {
            return STREAM_LARGE_VID;
        } else if (stream == &snap_stream) {
            return STREAM_SNAPSHOT;
        } else {
            return STREAM_INVALID;
        }
    }

    BufferGroup *GetBufferGroup(camera3_stream_t *stream){
        return GetBufferGroup(GetStreamId(stream));
    }
    BufferGroup *GetBufferGroup(STREAM_ID stream){
        switch (stream){
            case STREAM_PREVIEW:
                return &pre_bufferGroup;
            case STREAM_SMALL_VID:
                return &small_vid_bufferGroup;
            case STREAM_LARGE_VID:
                return &large_vid_bufferGroup;
            case STREAM_SNAPSHOT:
                return &snap_bufferGroup;
            default:
                return NULL;
        }
    }

    camera_module_t*                    pCameraModule = NULL;        ///< Camera module
    VideoEncoder*                       pVideoEncoderSmall = NULL;
    VideoEncoder*                       pVideoEncoderLarge = NULL;
    ModalExposureHist                   expHistInterface;
    ModalExposureMSV                    expMSVInterface;
    Camera3Callbacks                    cameraCallbacks;             ///< Camera callbacks
    camera3_device_t*                   pDevice = NULL;              ///< HAL3 device
    uint8_t                             num_streams;
    camera3_stream_t                    pre_stream;                  ///< Stream to be used for the preview request
    camera3_stream_t                    small_vid_stream;                  ///< Stream to be used for the stream request
    camera3_stream_t                    large_vid_stream;                  ///< Stream to be used for the record request
    camera3_stream_t                    snap_stream;                 ///< Stream to be used for the snapshots request
    android::CameraMetadata             requestMetadata;             ///< Per request metadata
    BufferGroup                         pre_bufferGroup;             ///< Buffer manager per stream
    BufferGroup                         small_vid_bufferGroup;             ///< Buffer manager per stream
    BufferGroup                         large_vid_bufferGroup;             ///< Buffer manager per stream
    BufferGroup                         snap_bufferGroup;            ///< Buffer manager per stream
    pthread_t                           requestThread;               ///< Request thread private data
    pthread_t                           resultThread;                ///< Result Thread private data
    pthread_mutex_t                     resultMutex;                 ///< Mutex for list access
    pthread_cond_t                      resultCond;                  ///< Condition variable for wake up
    pthread_mutex_t                     aeMutex;                     ///< Mutex for list access
    bool                                is10bit = false;             ///< Marks if a raw preview image is raw10 or raw8
    int64_t                             setExposure = 5259763;       ///< Exposure
    int32_t                             setGain     = 800;           ///< Gain
    queue<image_result>                 resultMsgQueue;
    RingBuffer<camera_image_metadata_t> resultMetaRing;
    RingBuffer<image_result>            resultMsgRing;
    pthread_mutex_t                     stereoMutex;                 ///< Mutex for stereo comms
    pthread_cond_t                      stereoCond;                  ///< Condition variable for wake up
    PerCameraMgr*                       otherMgr = NULL;             ///< Pointer to the partner manager in a stereo pair
    PCM_MODE                            partnerMode;                 ///< Mode for mono/stereo
    uint8_t*                            childFrame = NULL;           ///< Pointer to the child frame, guarded with stereoMutex
    uint8_t*                            childFrame_uvHead = NULL;
    camera_image_metadata_t             childInfo;                   ///< Copy of the child frame info
    bool                                stopped = false;             ///< Indication for the thread to terminate
    bool                                EStopped = false;            ///< Emergency Stop, terminate without any cleanup
    int                                 lastResultFrameNumber = -1;  ///< Last frame the capture result thread should wait for before terminating
    queue<char *>                       snapshotQueue;
    atomic_int                          numNeededSnapshots {0};
    camera_metadata_t*                  pSessionParams = NULL;       ///< Value for encoding type of large stream
    int                                 has_set_preview_thread_affinity = 0;

    ///< TOF Specific members

    // APQ and qrb have different royale APIs, maybe someday we'll backport the
    //     clean api to voxl1 but right now it's baked into the system image
    #ifdef APQ8096
        void*                          tof_interface;                ///< TOF interface to process the TOF camera raw data
    #else
        TOFInterface*                  tof_interface;                ///< TOF interface to process the TOF camera raw data
    #endif

    uint32_t                           TOFFrameNumber = 0;
    int                                tofFrameCounter = 0;



    // these are buffers to hold preview frames that we have processed, e.g
    // converted from raw10 to raw8 or debayered. Preview pipes publish from
    // these buffers when in RAW mode
    uint8_t* previewFrameGrey8 = NULL;
    uint8_t* previewFrameRGB8 = NULL;

    /////////////////////////////////////
    // MPA pipe channels
    ////////////////////////////////////

    // Channels for preview streams
    int     previewPipeBayer = -1;
    int     previewPipeGrey = -1;
    int     previewPipeColor = -1;

    // Channels for stream streams
    int     smallVideoPipeGrey  = -1;
    int     smallVideoPipeColor = -1;
    int     smallVideoPipeEncoded  = -1;

    // Channels for Record streams
    int     largeVideoPipeGrey  = -1;
    int     largeVideoPipeColor = -1;
    int     largeVideoPipeEncoded  = -1;
    
    // Channels for Snapshot
    int     snapshotPipe = -1;

    // Channels for TOF
    int     tofPipeIR    = -1;
    int     tofPipeDepth = -1;
    int     tofPipeConf  = -1;
    int     tofPipePC    = -1;
    int     tofPipeFull  = -1;

    void close_my_pipes(void)
    {
        if(     previewPipeBayer   >=0) pipe_server_close(     previewPipeBayer    );
        if(     previewPipeGrey    >=0) pipe_server_close(     previewPipeGrey     );
        if(     previewPipeColor   >=0) pipe_server_close(     previewPipeColor    );
        if(  smallVideoPipeGrey    >=0) pipe_server_close(  smallVideoPipeGrey     );
        if(  smallVideoPipeColor   >=0) pipe_server_close(  smallVideoPipeColor    );
        if(  smallVideoPipeEncoded >=0) pipe_server_close(  smallVideoPipeEncoded  );
        if(  largeVideoPipeGrey    >=0) pipe_server_close(  largeVideoPipeGrey     );
        if(  largeVideoPipeColor   >=0) pipe_server_close(  largeVideoPipeColor    );
        if(  largeVideoPipeEncoded >=0) pipe_server_close(  largeVideoPipeEncoded  );
        if(    snapshotPipe        >=0) pipe_server_close(    snapshotPipe         );
        if(         tofPipeIR      >=0) pipe_server_close(         tofPipeIR       );
        if(         tofPipeDepth   >=0) pipe_server_close(         tofPipeDepth    );
        if(         tofPipeConf    >=0) pipe_server_close(         tofPipeConf     );
        if(         tofPipePC      >=0) pipe_server_close(         tofPipePC       );
        if(         tofPipeFull    >=0) pipe_server_close(         tofPipeFull     );
        return;
    }

    void setMaster(PerCameraMgr *master) { ///< Tells a camera manager that the passed in pointer is it's master
        partnerMode = MODE_STEREO_SLAVE;
        otherMgr    = master;
    }
};


#endif
