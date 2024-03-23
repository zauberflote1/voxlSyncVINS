/*******************************************************************************************************************************
 *
 * Copyright (c) 2022 ModalAI, Inc.
 *
 ******************************************************************************************************************************/

#ifndef CAMXHAL3BUFFER_H
#define CAMXHAL3BUFFER_H

#include <mutex>
#include <deque>
#include "hardware/camera3.h"

#define  BUFFER_QUEUE_MAX_SIZE  32

#define ALIGN_BYTE(x, a) ((x % a == 0) ? x : x - (x % a) + a)

// Platform Specific Flags
#ifdef APQ8096
    #define HAL3_FMT_YUV  HAL_PIXEL_FORMAT_YCbCr_420_888
#elif QRB5165
    #define HAL3_FMT_YUV  HAL_PIXEL_FORMAT_YCBCR_420_888
#else
    #error "No Platform defined"
#endif

typedef struct _BufferBlock {
    void*             vaddress;
    void*             uvHead;
    uint64_t size;
    uint32_t      width;
    uint32_t      height;
    uint32_t      stride;
    uint32_t      slice;
} BufferBlock;

typedef struct _BufferGroup {
    std::deque<buffer_handle_t*> freeBuffers;
    uint32_t            totalBuffers = 0;
    buffer_handle_t     buffers[BUFFER_QUEUE_MAX_SIZE];
    BufferBlock         bufferBlocks[BUFFER_QUEUE_MAX_SIZE];
    std::mutex bufferMutex;
} BufferGroup;

int bufferAllocateBuffers(
    BufferGroup& bufferGroup,
    uint32_t totalBuffers,
    uint32_t width,
    uint32_t height,
    uint32_t format,
    uint64_t consumerFlags);

void bufferDeleteBuffers(BufferGroup& buffer);
void bufferPush(BufferGroup& bufferGroup, buffer_handle_t* buffer);
void bufferPushAddress(BufferGroup& bufferGroup, void* vaddress);
buffer_handle_t* bufferPop(BufferGroup& bufferGroup);
BufferBlock* bufferGetBufferInfo(BufferGroup* bufferGroup, buffer_handle_t* buffer);
int bufferNumFree(BufferGroup& bufferGroup);

#endif // CAMXHAL3BUFFER_H
