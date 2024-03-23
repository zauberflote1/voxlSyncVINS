/*******************************************************************************************************************************
 *
 * Copyright (c) 2022 ModalAI, Inc.
 *
 ******************************************************************************************************************************/

#ifdef QRB5165

#include <asm-generic/errno-base.h>
#include <cassert>
#include <cstring>
#include <gbm.h>
#include <gbm_priv.h>
#include <system/graphics.h>
#include <system/window.h>
#include <unordered_map>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/ion.h>
#include <linux/msm_ion.h>
#include <camera/CameraMetadata.h>
#include <libhardware/gralloc_priv.h>

#include "buffer_manager.h"
#include "common_defs.h"
#include <modal_journal.h>

using namespace std;

// manage internal GBM state here
// we need to delete GBM BOs when they're deleted externally, but we don't want
// to delete the GBM device until program exit
// use a cursed singleton
class GBMAllocator {
public:
    static GBMAllocator& getInstance() {
        static GBMAllocator instance;
        return instance;
    }

    // delete copy and move constructors, operator=
    GBMAllocator(GBMAllocator& other) = delete;
    GBMAllocator(GBMAllocator&& other) = delete;
    void operator=(GBMAllocator const& other) = delete;


    /**
     * Allocate one GBM buffer object.
     *
     * @param group      the BufferGroup for which we are allocating a buffer.
     *                   this parameter is only used to generate a key for
     *                   group_buffers_map.
     * @param index      the index of the external BufferBlock, so that we can delete
     *                   the correct GBM buffer when the external BufferBlock is deleted.
     * @param width      the width of the image which will be stored in the buffer
     * @param height     the height of the image which will be stored in the
     *                   buffer
     * @param gbm_format the gbm format of the image (NOT hal3 format -- must
     *                   translate before passing here!)
     * @param gbm_flags  the gbm flags of the buffer (NOT gralloc flags -- must
     *                   translate before passing here!)
     */
    struct gbm_bo* allocateBuffer(BufferGroup const& group, uint32_t index, uint32_t width, uint32_t height,
            uint32_t gbm_format, uint32_t gbm_flags) {

        auto& buffers = group_buffers_map[&group];

        const auto existing = buffers.find(index);
        if (existing != buffers.end()) {
            M_WARN("Attempted to allocate existing index in GBMAllocator, returning existing buffer\n");
            return existing->second;
        }

        struct gbm_bo* out = gbm_bo_create(gbm_dev, width, height, gbm_format, gbm_flags);
        if (out != nullptr) {
            buffers[index] = out;
        }
        return out;
    }

    int deallocateBuffer(BufferGroup const& group, uint32_t index) {
        auto& buffers = group_buffers_map[&group];
        auto existing = buffers.find(index);

        if (existing == buffers.end()) {
            M_WARN("Unable to find buffer to deallocate, group=%p index=%d\n", &group, index);
            return -1;
        }

        struct gbm_bo* bo = existing->second;
        buffers.erase(index);
        gbm_bo_destroy(bo);
        return 0;
    }

private:
    int gbm_fd_;
    struct gbm_device* gbm_dev;
    // keep track of the BOs for each index in each buffer group
    std::unordered_map<BufferGroup const*,
        std::unordered_map<uint32_t, gbm_bo*>> group_buffers_map;


    GBMAllocator() {
        gbm_fd_ = open("/dev/dri/card0", O_RDWR);
        if (gbm_fd_ < 0) {
            M_WARN("Unable to open /dev/dri/card0, falling back to /dev/ion\n");

            gbm_fd_ = open("/dev/ion", O_RDWR);
            if (gbm_fd_ < 0) {
                M_ERROR("Opening /dev/ion also failed\n");
            }
        }
        assert(gbm_fd_ >= 0);
        M_VERBOSE("Opened GBM fd\n");

        gbm_dev = gbm_create_device(gbm_fd_);
        assert(gbm_dev != nullptr);
        M_VERBOSE("Created GBM device\n");
    }


    ~GBMAllocator() {
        // TODO: delete all BOs here. not sure what happens if they persist past
        // the GBM device which allocated them
        gbm_device_destroy(gbm_dev);
        close(gbm_fd_);
    }
};

const std::unordered_map<int32_t, int32_t> gralloc_usage_flag_map = {
  {GRALLOC_USAGE_HW_CAMERA_ZSL,      0                              },
// apparently we don't have these two yet (from source material aka qmmf)
// keeping them in case of updates
//   {GRALLOC_USAGE_PRIVATE_ALLOC_UBWC, GBM_BO_USAGE_UBWC_ALIGNED_QTI  },
//   {GRALLOC_USAGE_PRIVATE_UNCACHED,   GBM_BO_USAGE_UNCACHED_QTI      },
  {GRALLOC_USAGE_PROTECTED,          GBM_BO_USAGE_PROTECTED_QTI     },
  {GRALLOC_USAGE_SW_READ_OFTEN,      GBM_BO_USAGE_CPU_READ_QTI      },
  {GRALLOC_USAGE_SW_WRITE_OFTEN,     GBM_BO_USAGE_CPU_WRITE_QTI     },
  {GRALLOC_USAGE_HW_VIDEO_ENCODER,   GBM_BO_USAGE_VIDEO_ENCODER_QTI },
  {GRALLOC_USAGE_HW_FB,              0                              },
  {GRALLOC_USAGE_HW_TEXTURE,         0                              },
  {GRALLOC_USAGE_HW_RENDER,          GBM_BO_USAGE_HW_RENDERING_QTI  },
  {GRALLOC_USAGE_HW_COMPOSER,        GBM_BO_USAGE_HW_COMPOSER_QTI   },
  {GRALLOC_USAGE_HW_CAMERA_READ,     GBM_BO_USAGE_CAMERA_READ_QTI   },
  {GRALLOC_USAGE_HW_CAMERA_WRITE,    GBM_BO_USAGE_CAMERA_WRITE_QTI  }};


const std::unordered_map<uint32_t, uint32_t> hal_format_flag_map = {
  {HAL_PIXEL_FORMAT_BGRA_8888,               GBM_FORMAT_BGRA8888              },
  {HAL_PIXEL_FORMAT_RGB_565,                 GBM_FORMAT_RGB565                },
  {HAL_PIXEL_FORMAT_RGB_888,                 GBM_FORMAT_RGB888                },
  {HAL_PIXEL_FORMAT_RGBA_1010102,            GBM_FORMAT_RGBA1010102           },
  {HAL_PIXEL_FORMAT_RGBA_8888,               GBM_FORMAT_RGBA8888              },
  {HAL_PIXEL_FORMAT_RGBX_8888,               GBM_FORMAT_RGBX8888              },

  {HAL_PIXEL_FORMAT_BLOB,                    GBM_FORMAT_BLOB                  },
//   {HAL_PIXEL_FORMAT_RAW8,                    0},
  {HAL_PIXEL_FORMAT_RAW10,                   GBM_FORMAT_RAW10                 },
  {HAL_PIXEL_FORMAT_RAW12,                   GBM_FORMAT_RAW12                 },
  {HAL_PIXEL_FORMAT_RAW16,                   GBM_FORMAT_RAW16                 },

  {HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED,  GBM_FORMAT_IMPLEMENTATION_DEFINED},

//  {HAL_PIXEL_FORMAT_NV12_ENCODEABLE,         GBM_FORMAT_NV12_ENCODEABLE},
//  {HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS,      GBM_FORMAT_YCbCr_420_SP_VENUS},
//  {HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC, GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC},

  {HAL_PIXEL_FORMAT_YCBCR_420_888,           GBM_FORMAT_YCbCr_420_888         },
  {HAL_PIXEL_FORMAT_YCBCR_422_SP,            GBM_FORMAT_YCbCr_422_SP          },
  {HAL_PIXEL_FORMAT_YCBCR_422_I,             0                                },
  {HAL_PIXEL_FORMAT_YCRCB_420_SP,            GBM_FORMAT_YCrCb_420_SP          },
  {HAL_PIXEL_FORMAT_YV12,                    0                                },
  {HAL_PIXEL_FORMAT_YCBCR_422_888,           0                                },
//  {HAL_PIXEL_FORMAT_NV21_ZSL,                GBM_FORMAT_NV21_ZSL},
};

const std::unordered_map<uint32_t, const char*> gbm_format_str_map = {
       {GBM_FORMAT_BGRA8888              , "GBM_FORMAT_BGRA8888"},
       {GBM_FORMAT_RGB565                , "GBM_FORMAT_RGB565"},
       {GBM_FORMAT_RGB888                , "GBM_FORMAT_RGB888"},
       {GBM_FORMAT_RGBA1010102           , "GBM_FORMAT_RGBA1010102"},
       {GBM_FORMAT_RGBA8888              , "GBM_FORMAT_RGBA8888"},
       {GBM_FORMAT_RGBX8888              , "GBM_FORMAT_RGBX8888"},
       {GBM_FORMAT_BLOB                  , "GBM_FORMAT_BLOB"},
       {GBM_FORMAT_RAW10                 , "GBM_FORMAT_RAW10"},
       {GBM_FORMAT_RAW12                 , "GBM_FORMAT_RAW12"},
       {GBM_FORMAT_RAW16                 , "GBM_FORMAT_RAW16"},
       {GBM_FORMAT_IMPLEMENTATION_DEFINED, "GBM_FORMAT_IMPLEMENTATION_DEFINED"},
       {GBM_FORMAT_NV12_ENCODEABLE       , "GBM_FORMAT_NV12_ENCODEABLE"},
       {GBM_FORMAT_YCbCr_420_SP_VENUS    , "GBM_FORMAT_YCbCr_420_SP_VENUS"},
       {GBM_FORMAT_YCbCr_420_888         , "GBM_FORMAT_YCbCr_420_888"},
       {GBM_FORMAT_YCbCr_422_SP          , "GBM_FORMAT_YCbCr_422_SP"},
       {GBM_FORMAT_YCrCb_420_SP          , "GBM_FORMAT_YCrCb_420_SP"},
};

const uint32_t usage_arr[] = {
    GBM_BO_USAGE_PROTECTED_QTI		,
    GBM_BO_USAGE_UNCACHED_QTI		,
    GBM_BO_USAGE_CPU_READ_QTI		,
    GBM_BO_USAGE_CPU_WRITE_QTI		,
    GBM_BO_USAGE_NON_CPU_WRITER_QTI	,
    GBM_BO_USAGE_UBWC_ALIGNED_QTI   ,
    GBM_BO_USAGE_CAMERA_READ_QTI	,
    GBM_BO_USAGE_CAMERA_WRITE_QTI	,
    GBM_BO_USAGE_VIDEO_ENCODER_QTI	,
    GBM_BO_USAGE_HW_COMPOSER_QTI	,
    GBM_BO_USAGE_HW_RENDERING_QTI   ,
    GBM_BO_USAGE_10BIT_QTI    	    ,
    GBM_BO_USAGE_10BIT_TP_QTI    	,
};

const string usage_str[] = {
    "GBM_BO_USAGE_PROTECTED_QTI		"    ,
    "GBM_BO_USAGE_UNCACHED_QTI		"    ,
    "GBM_BO_USAGE_CPU_READ_QTI		"    ,
    "GBM_BO_USAGE_CPU_WRITE_QTI		"    ,
    "GBM_BO_USAGE_NON_CPU_WRITER_QTI"	,
    "GBM_BO_USAGE_UBWC_ALIGNED_QTI  "    ,
    "GBM_BO_USAGE_CAMERA_READ_QTI	"    ,
    "GBM_BO_USAGE_CAMERA_WRITE_QTI	"    ,
    "GBM_BO_USAGE_VIDEO_ENCODER_QTI	"    ,
    "GBM_BO_USAGE_HW_COMPOSER_QTI	"    ,
    "GBM_BO_USAGE_HW_RENDERING_QTI  "    ,
    "GBM_BO_USAGE_10BIT_QTI    	    "    ,
    "GBM_BO_USAGE_10BIT_TP_QTI    	"    ,
};

void printGbmFlags(uint32_t flags) {
    M_VERBOSE("Dumping GBM flags\n");
    for (int i = 0; i < 13; i++) {
        if (flags & usage_arr[i]) {
            M_DEBUG("\tFound flag %s\n", usage_str[i].c_str());
        }
    }
}


uint32_t grallocFlagsToGbm(uint32_t gralloc_flags) {
    uint32_t output = 0;
    for (auto const& pair : gralloc_usage_flag_map) {
        if (gralloc_flags & pair.first) {
            output |= pair.second;
        }
    }

    // if we have encoder, then remove camera read/write flags
    // for some reason, camera read/write flags cause GBM to get confused and
    // allocated wrongly-sized buffers in certain cases. this is also documented
    // inside QMMF. if we leave these flags in while also having video encoder
    // usage, then camx will crash because it's expecting larger buffers than
    // GBM will allocate.
    if (output & GBM_BO_USAGE_VIDEO_ENCODER_QTI) {
        M_DEBUG("Found video encoder usage, removing camera read/write usage\n");
        output = output & ~(GBM_BO_USAGE_CAMERA_READ_QTI | GBM_BO_USAGE_CAMERA_WRITE_QTI);

        // sanity check because I'm paranoid
        if (output & GBM_BO_USAGE_CAMERA_WRITE_QTI || output & GBM_BO_USAGE_CAMERA_READ_QTI) {
            M_ERROR("Attempted to remove camera read/write, but still present\n");
        }
    }

    M_DEBUG("Converted gralloc flags 0x%x to GBM flags 0x%x\n", gralloc_flags, output);
    printGbmFlags(output);

    return output;
}

uint32_t hal3FormatToGbm(uint32_t hal3_format) {
    for (auto const& pair : hal_format_flag_map) {
        if (hal3_format == pair.first) {
            return pair.second;
        }
    }
    M_WARN("Got unrecognized hal3 format in %s, defaulting to GBM_FORMAT_IMPLEMENTATION_DEFINED\n", __FUNCTION__);
    return GBM_FORMAT_IMPLEMENTATION_DEFINED;
}


void* getUVStartFromFmt(uint8_t* block_start, uint32_t aligned_width, uint32_t aligned_height,
        uint32_t fmt) {
    switch(fmt) {
        case HAL_PIXEL_FORMAT_YCBCR_420_888:
            return static_cast<void*>(block_start + (aligned_width * aligned_height));

        case HAL_PIXEL_FORMAT_BLOB:
        case HAL_PIXEL_FORMAT_RAW10:
        case HAL_PIXEL_FORMAT_RAW12:
            return nullptr;

        default:
            M_WARN("Got unsupported format in %s, returning nullptr\n", __FUNCTION__);
            return nullptr;
    }
}

int allocateOneBuffer(BufferGroup& bufferGroup, uint32_t index, uint32_t width, uint32_t height,
        uint32_t hal3_format, uint64_t gralloc_flags, buffer_handle_t* pBuffer) {

    uint32_t gbm_flags  = grallocFlagsToGbm(gralloc_flags);
    uint32_t gbm_format = hal3FormatToGbm(hal3_format);

    struct gbm_bo* bo = GBMAllocator::getInstance()
        .allocateBuffer(bufferGroup, index, width, height, gbm_format, gbm_flags);
    if (bo == nullptr) {
        int errsv = errno;
        M_ERROR("GBM allocation failed: %s errno=%d\n", strerror(errsv), errsv);
        return -EINVAL;
    }

    uint32_t stride;
    // workaround/HACK: for ov7251, the driver always returns RAW8 data, but we need to request RAW10 from hal3
    // HAL3 will write RAW8 data into the given buffer as though there was no stride, therefore we will ignore stride
    // and overwrite it with width in that specific case
    if (hal3_format == HAL_PIXEL_FORMAT_RAW10 && width == 640 && height == 480) {
        stride = width;
    }
    else {
        stride = gbm_bo_get_stride(bo);
    }


    uint32_t aligned_h;
    int rval = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_HEIGHT, bo, &aligned_h);
    if (rval != 0) {
        M_ERROR("Could not get buffer stride: error=%d\n", rval);
        return -EINVAL;
    }

    // Note to reader: gbm_priv also contains the action
    // GBM_PERFORMGET_BO_ALIGNED_WIDTH. However, qmmf just uses stride, so we
    // will emulate that behavior here. Experimentally, it seems as though
    // stride >= aligned_width always. leaving the following for reference.
    uint32_t aligned_w = 0;
    rval = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_WIDTH, bo, &aligned_w);

    size_t size;
    rval = gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &size);
    if (rval != 0) {
        M_ERROR("Could not get buffer size: error=%d\n", rval);
        return -EINVAL;
    }

    int fd = gbm_bo_get_fd(bo);
    if (fd < 0) {
        int errsv = errno;
        M_ERROR("Unable to get fd from bo: %s (errno=%d)\n", strerror(errsv), errsv);
        return -EINVAL;
    }

    // map the allocated buffer into our address space
    void* buf_addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (buf_addr == nullptr) {
        int errsv = errno;
        M_ERROR("Unable to mmap buffer object: %s (errno=%d)\n", strerror(errsv), errsv);
        return -EINVAL;
    }

    auto format_str = gbm_format_str_map.find(gbm_format)->second;
    M_DEBUG("Allocated BO with width=%u height=%u stride=%u aligned_w=%u aligned_h=%u size=%u flags=0x%x format=%s\n",
            width, height, stride, aligned_w, aligned_h, size, gralloc_flags, format_str);

    // populate buffer block
    BufferBlock& block = bufferGroup.bufferBlocks[index];
    block.vaddress = buf_addr;
    block.uvHead   = getUVStartFromFmt((uint8_t*) buf_addr, stride, aligned_h, hal3_format);
    block.size     = size;
    block.width    = width;
    block.height   = height;
    block.stride   = stride;
    block.slice    = aligned_h;

    // construct a native_handle_t from our bo
    private_handle_t* priv_handle = new private_handle_t(fd, size, gralloc_flags);
    // private_handle_t is a superclass of native_handle_t which initializes the
    // trailing VLA correctly
    *pBuffer = static_cast<native_handle_t*>(priv_handle);

    return 0;
}

void deleteOneBuffer(BufferGroup& group, uint32_t index) {
    if (group.buffers[index] == NULL) {
        M_WARN("Attempted to delete null buffer\n");
        return;
    }

    BufferBlock& block = group.bufferBlocks[index];

    munmap(block.vaddress, block.size);
    private_handle_t* priv_handle = static_cast<private_handle_t*>(
                                        const_cast<native_handle_t*>(group.buffers[index]));
    delete priv_handle;
    int rval = GBMAllocator::getInstance()
        .deallocateBuffer(group, index);

    if (rval != 0) {
        M_ERROR("Encountered error while deallocating buffer\n");
    }
}

#endif
