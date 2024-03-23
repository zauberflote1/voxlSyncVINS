/*******************************************************************************************************************************
 *
 * Copyright (c) 2022 ModalAI, Inc.
 *
 ******************************************************************************************************************************/

#ifdef APQ8096

#include <stdlib.h>
#include <log/log.h>
#include <hardware/gralloc.h>
#include <errno.h>
#include <map>

#include "buffer_manager.h"
#include "common_defs.h"
#include <modal_journal.h>

using namespace std;

static gralloc_module_t* grallocModule = NULL;
static alloc_device_t*   grallocDevice = NULL;

// -----------------------------------------------------------------------------------------------------------------------------
// Sets up the gralloc interface to be used for making the buffer memory allocation and lock/unlock/free calls
// -----------------------------------------------------------------------------------------------------------------------------
static int SetupGrallocInterface()
{

    hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t**)(&grallocModule));

    if (grallocModule == NULL)
    {
        M_ERROR("Failed to get Gralloc hardware module\n\n");
        return -1;
    }

    gralloc_open((const hw_module_t*)grallocModule, &grallocDevice);

    if (grallocDevice == NULL)
    {
        M_ERROR("Failed to get Gralloc device!\n\n");
        return -1;
    }

    return 0;
}

// -----------------------------------------------------------------------------------------------------------------------------
// Makes buffer contiguous
// -----------------------------------------------------------------------------------------------------------------------------
// void bufferMakeYUVContiguous(BufferBlock* pBufferInfo)
// {

//     const int height = pBufferInfo->height;
//     const int width  = pBufferInfo->width;

//     if((uint8_t*)(pBufferInfo->vaddress) + (width*height) == pBufferInfo->uvHead){
//         M_VERBOSE("Buffer already continuous\n");
//         return;
//     }

//     memcpy((uint8_t*)(pBufferInfo->vaddress) + (width*height), pBufferInfo->uvHead, (width * height / 2));
// }

// -----------------------------------------------------------------------------------------------------------------------------
// Call the Gralloc interface to do the actual memory allocation for one single buffer
// -----------------------------------------------------------------------------------------------------------------------------
int allocateOneBuffer(
        BufferGroup&       bufferGroup,
        uint32_t       index,
        uint32_t       width,
        uint32_t       height,
        uint32_t       format,
        uint64_t  consumerFlags,
        buffer_handle_t*   pBuffer)
{

    // For the TOF camera we have to send the BLOB format buffers to the camera module but these are not jpg images. So we  
    // cant really compute the size for a jpeg image hence just make it twice the size of the (width * height)  
    if (format == HAL_PIXEL_FORMAT_BLOB)
    {   
        width  = width * height * 2;    
        height = 1; 
    }

    //Fail if it's not already open and we fail to open
    if(grallocDevice == NULL && SetupGrallocInterface()) return -1;

    bufferGroup.bufferBlocks[index].width  = width;
    bufferGroup.bufferBlocks[index].height = height;

    // Call gralloc to make the memory allocation
    grallocDevice->alloc(grallocDevice,
                            width,
                            height,
                            format,
                            consumerFlags,
                            pBuffer,
                            (int*)&bufferGroup.bufferBlocks[index].stride);

    // Get the CPU virtual address of the buffer memory allocation
    if (format == HAL_PIXEL_FORMAT_RAW10)
    {
        grallocModule->lock(grallocModule,
                             *pBuffer,
                             0,
                             0,
                             0,
                             width,
                             height,
                             &bufferGroup.bufferBlocks[index].vaddress);

        bufferGroup.bufferBlocks[index].size =
            bufferGroup.bufferBlocks[index].stride * height;

    } else if (format == HAL3_FMT_YUV)
    {
        struct android_ycbcr ycbcr;
        grallocModule->lock_ycbcr(grallocModule,
                                   *pBuffer,
                                   consumerFlags,
                                   0,
                                   0,
                                   width,
                                   height,
                                   &ycbcr);

        bufferGroup.bufferBlocks[index].vaddress  = ycbcr.y;
        if (ycbcr.cr < ycbcr.cb)
        {
            bufferGroup.bufferBlocks[index].uvHead = ycbcr.cb;
        }
        else
        {
            bufferGroup.bufferBlocks[index].uvHead = ycbcr.cr;
        }

        bufferGroup.bufferBlocks[index].size =
                bufferGroup.bufferBlocks[index].stride * height * 1.5; // 1.5 because it is 12 bits / pixel

    } else if (format == HAL_PIXEL_FORMAT_BLOB)
    {

        grallocModule->lock(grallocModule,
                             *pBuffer,
                             GRALLOC_USAGE_SW_READ_OFTEN,
                             0,
                             0,
                             width,
                             height,
                             &bufferGroup.bufferBlocks[index].vaddress);

        bufferGroup.bufferBlocks[index].size =
                bufferGroup.bufferBlocks[index].stride * height;
        bufferGroup.bufferBlocks[index].width          = width;
        bufferGroup.bufferBlocks[index].height         = height;
    } else
    {
        M_ERROR("Unknown pixel format!\n");
        return -1;
    }

    return 0;
}

void deleteOneBuffer(
       BufferGroup&       bufferGroup,
       uint32_t       index)
{
    if (grallocDevice != NULL && bufferGroup.buffers[index] != NULL)
    {
        grallocModule->unlock(grallocModule, bufferGroup.buffers[index]);
        grallocDevice->free(grallocDevice, bufferGroup.buffers[index]);
        bufferGroup.buffers[index] = NULL;
    }
}

#endif
