/*******************************************************************************************************************************
 *
 * Copyright (c) 2022 ModalAI, Inc.
 *
 ******************************************************************************************************************************/

#include "buffer_manager.h"
#include <modal_journal.h>

using namespace std;


#define ALIGN_BYTE(x, a) ((x % a == 0) ? x : x - (x % a) + a)

//These two will be implementation-dependent, found in the buffer_impl_*.cpp files
extern int allocateOneBuffer(
        BufferGroup& bufferGroup,
        uint32_t     index,
        uint32_t     width,
        uint32_t     height,
        uint32_t     format,
        uint64_t     consumerFlags,
        buffer_handle_t* pBuffer);

extern void deleteOneBuffer(
        BufferGroup& bufferGroup,
        uint32_t     index);

//
// =============================================================
//

void bufferDeleteBuffers(BufferGroup& bufferGroup)
{

    if (bufferGroup.totalBuffers != bufferGroup.freeBuffers.size()){
        M_WARN("Deleting buffers: %lu of %d still in use\n",
            (bufferGroup.totalBuffers)-(bufferGroup.freeBuffers.size()),
            bufferGroup.totalBuffers);
    }
    for (uint32_t i = 0; i < bufferGroup.totalBuffers; i++) {
        deleteOneBuffer(bufferGroup, i);
    }
}

int bufferAllocateBuffers(
    BufferGroup& bufferGroup,
    uint32_t totalBuffers,
    uint32_t width,
    uint32_t height,
    uint32_t format,
    uint64_t consumerFlags)
{
    bufferGroup.totalBuffers=0;
    for (uint32_t i = 0; i < totalBuffers; i++) {

        if(allocateOneBuffer(bufferGroup, i, width, height, format, consumerFlags, &bufferGroup.buffers[i])) return -1;

        if(bufferGroup.bufferBlocks[i].vaddress == NULL){
            M_ERROR("Buffer was allocated but did not populate the vaddress field\n");
            return -1;
        }

        bufferGroup.totalBuffers++;
        bufferGroup.freeBuffers.push_back(&bufferGroup.buffers[i]);
    }

    return 0;
}

void bufferPush(BufferGroup& bufferGroup, buffer_handle_t* buffer)
{
    unique_lock<mutex> lock(bufferGroup.bufferMutex);

    // push to the back, later we will pop from the front so all buffers get used
    bufferGroup.freeBuffers.push_back(buffer);
}

void bufferPushAddress(BufferGroup& bufferGroup, void* vaddress)
{
    unique_lock<mutex> lock(bufferGroup.bufferMutex);
    for (uint32_t i = 0; i < bufferGroup.totalBuffers; i++){
        if (vaddress == bufferGroup.bufferBlocks[i].vaddress){
            // push to the back, later we will pop from the front so all buffers get used
            bufferGroup.freeBuffers.push_back(&bufferGroup.buffers[i]);
            return;
        }
    }
    M_ERROR("Recieved invalid buffer in %s\n", __FUNCTION__);
    return;
}

buffer_handle_t* bufferPop(BufferGroup& bufferGroup)
{
    unique_lock<mutex> lock(bufferGroup.bufferMutex);
    if (bufferGroup.freeBuffers.size() == 0) {
        return NULL;
    }

    buffer_handle_t* buffer = bufferGroup.freeBuffers.front();
    bufferGroup.freeBuffers.pop_front();
    return buffer;
}

BufferBlock* bufferGetBufferInfo(BufferGroup* bufferGroup, buffer_handle_t* buffer)
{
    unique_lock<mutex> lock(bufferGroup->bufferMutex);
    for (uint32_t i = 0;i < bufferGroup->totalBuffers;i++){
        if (*buffer == bufferGroup->buffers[i]){
            return &(bufferGroup->bufferBlocks[i]);
        }
    }
    M_ERROR("%s wan't able to successfully find the requested buffer\n", __FUNCTION__ );
    return NULL;
}

int bufferNumFree(BufferGroup& bufferGroup)
{
    int ret;
    unique_lock<mutex> lock(bufferGroup.bufferMutex);
    ret = bufferGroup.freeBuffers.size();
    return ret;
}
