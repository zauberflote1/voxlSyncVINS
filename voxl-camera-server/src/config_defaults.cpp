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

#include <string.h>
#include <ctype.h>

#include "config_defaults.h"
#include "common_defs.h"


#define VENC_H264_SMALL_DEFAULT {\
	.mode = VENC_H264,\
	.br_ctrl = VENC_CONTROL_CQP,\
	.Qfixed = 30,\
	.Qmin = 15,\
	.Qmax = 40,\
	.nPframes = 9,\
	.mbps = 2.0,\
}

#define VENC_H264_LARGE_DEFAULT {\
	.mode = VENC_H264,\
	.br_ctrl = VENC_CONTROL_CQP,\
	.Qfixed = 40,\
	.Qmin = 15,\
	.Qmax = 50,\
	.nPframes = 29,\
	.mbps = 40.0,\
}

#define VENC_H265_SMALL_DEFAULT {\
	.mode = VENC_H265,\
	.br_ctrl = VENC_CONTROL_CQP,\
	.Qfixed = 30,\
	.Qmin = 15,\
	.Qmax = 40,\
	.nPframes = 9,\
	.mbps = 2.0,\
}

#define VENC_H265_LARGE_DEFAULT {\
	.mode = VENC_H265,\
	.br_ctrl = VENC_CONTROL_CQP,\
	.Qfixed = 38,\
	.Qmin = 15,\
	.Qmax = 50,\
	.nPframes = 29,\
	.mbps = 30.0,\
}



////////////////////////////////////////////////////////////////////////////////
// These are defaults for the histogram algo, we don't use this anymore
////////////////////////////////////////////////////////////////////////////////
//< Gain Min
//< Gain Max
//< Exposure Min
//< Exposure Max
//< Desired MSV
//< k_p_ns
//< k_i_ns
//< Max i
//< p Good Threshold
//< Exposure Period
//< Gain Period
//< Display Debug
//< Exposure offset

#define AE_HIST_DEFAULTS_OV7251 {\
    98,    \
    833,   \
    20,     \
    33000,  \
    64.0,   \
    8000.0, \
    5.0,    \
    250.0,  \
    3,      \
    1,      \
    2,      \
    0,      \
    8000}

#define AE_HIST_DEFAULTS_OV9782 {\
    54,     \
    835,    \
    20,     \
    33000,  \
    54.0,   \
    8000.0, \
    5.0,    \
    250.0,  \
    3,      \
    1,      \
    2,      \
    0,      \
    8000}

#define AE_HIST_DEFAULTS_AR0144 {\
    54,     \
    835,    \
    20,     \
    33000,  \
    54.0,   \
    8000.0, \
    5.0,    \
    250.0,  \
    3,      \
    1,      \
    2,      \
    0,      \
    8000}


////////////////////////////////////////////////////////////////////////////////
// These are defaults for the MSV algo, this is the new default
// note that the gain min/max are queried from HAL3 at runtime. The values here
// are used as additional contraints to bound the hal3 limits further.
// a typical min value is 100 in qrb5165 from system image 1.7.1 and newer
// the lower limit is set here to 54 to allow backwards compatability with
// older system images that used to have a lower bound of 54.
////////////////////////////////////////////////////////////////////////////////
//< Gain Min
//< Gain Max
//< Exposure Min
//< Exposure Max
//< Soft min exposure
//< Gain Slope
//< Desired MSV
//< Filter Alpha
//< Most saturated ignore frac
//< Exposure update period
//< Gain update period
//< Display Debug
#define AE_MSV_DEFAULTS_OV7251 {\
    54,   \
    8000,  \
    20,   \
    33000,\
    5000, \
    0.05, \
    60.0, \
    0.6,  \
    0.2,  \
    1,    \
    1,    \
    false}

#define AE_MSV_DEFAULTS_OV9782 {\
    54,   \
    8000,  \
    20,   \
    33000,\
    5000, \
    0.05, \
    60.0, \
    0.6,  \
    0.2,  \
    1,    \
    1,    \
    false }

#define AE_MSV_DEFAULTS_AR0144 {\
    54,   \
    8000,  \
    20,   \
    33000,\
    5000, \
    0.05, \
    60.0, \
    0.6,  \
    0.2,  \
    1,    \
    1,    \
    false }




static const PerCameraInfo emptyDefaults =
    {
        "",                         //< Name
        SENSOR_INVALID,             //< Sensor
        BAYER_MONO,                 //< bayer pixel format
        -1,                         //< ID
        -1,                         //< ID2
        0,                          //< Enabled?
        -1,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        0,                          //< Enable Preview Mode?
        -1,                         //< Preview Width of the frame
        -1,                         //< Preview Height of the frame
        1,                          //< en_raw_preview
        0,                          //< Enable Small Video
        -1,                         //< Small Video Width of the frame
        -1,                         //< Small Video Height of the frame
        VENC_H264_SMALL_DEFAULT,
        0,                          //< Enable Large Video
        -1,                         //< Large Video Width of the frame
        -1,                         //< Large Video Height of the frame
        VENC_H264_LARGE_DEFAULT,
        0,                          //< Enable Snapshot mode?
        -1,                         //< Snapshot Width of the frame
        -1,                         //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_OFF,                     //< AE Mode
        AE_HIST_DEFAULTS_OV7251,
        AE_MSV_DEFAULTS_OV7251,
        0,                          //< Standby Enabled
        1                           //< Standby Decimator
    };


static const PerCameraInfo OV7251Defaults = 
    {
        "",                         //< Name
        SENSOR_OV7251,              //< Sensor
        BAYER_MONO,                 //< bayer pixel format
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        30,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        1,                          //< Enable Preview Mode?
        640,                        //< Preview Width of the frame
        480,                        //< Preview Height of the frame
        1,                          //< en_raw_preview
        0,                          //< Enable Small Video
        -1,                         //< Small Video Width of the frame
        -1,                         //< Small Video Height of the frame
        VENC_H264_SMALL_DEFAULT,
        0,                          //< Enable Large Video
        -1,                         //< Large Video Width of the frame
        -1,                         //< Large Video Height of the frame
        VENC_H264_LARGE_DEFAULT,
        0,                          //< Enable Snapshot mode?
        -1,                         //< Snapshot Width of the frame
        -1,                         //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_LME_MSV,                 //< AE Mode
        AE_HIST_DEFAULTS_OV7251,
        AE_MSV_DEFAULTS_OV7251,
        0,                          //< Standby Enabled
        1,                          //< Standby Decimator
    };


static const PerCameraInfo OV9782Defaults =
    {
        "",                         //< Name
        SENSOR_OV9782,              //< Sensor
        BAYER_BGGR,                 //< bayer pixel format
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        30,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        1,                          //< Enable Preview Mode?
        1280,                       //< Preview Width of the frame
        800,                        //< Preview Height of the frame
        1,                          //< en_raw_preview
        0,                          //< Enable Small Video
        -1,                         //< Small Video Width of the frame
        -1,                         //< Small Video Height of the frame
        VENC_H264_SMALL_DEFAULT,
        0,                          //< Enable Large Video
        -1,                         //< Large Video Width of the frame
        -1,                         //< Large Video Height of the frame
        VENC_H264_LARGE_DEFAULT,
        0,                          //< Enable Snapshot mode?
        -1,                         //< Snapshot Width of the frame
        -1,                         //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_LME_MSV,                 //< AE Mode
        AE_HIST_DEFAULTS_OV9782,
        AE_MSV_DEFAULTS_OV9782,
        0,                          //< Standby Enabled
        1                           //< Standby Decimator
    };



// We have the mono version of AR0144
// HFOV 138.6 deg
// VFOV 84.4 deg
// DFOV 162.4 deg
static const PerCameraInfo AR0144Defaults =
    {
        "",                         //< Name
        SENSOR_AR0144,              //< Sensor
        BAYER_MONO,                 //< bayer pixel format
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        30,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        1,                          //< Enable Preview Mode?
        1280,                       //< Preview Width of the frame
        800,                        //< Preview Height of the frame
        1,                          //< en_raw_preview
        0,                          //< Enable Small Video
        -1,                         //< Small Video Width of the frame
        -1,                         //< Small Video Height of the frame
        VENC_H264_SMALL_DEFAULT,
        0,                          //< Enable Large Video
        -1,                         //< Large Video Width of the frame
        -1,                         //< Large Video Height of the frame
        VENC_H264_LARGE_DEFAULT,
        0,                          //< Enable Snapshot mode?
        -1,                         //< Snapshot Width of the frame
        -1,                         //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_LME_MSV,                 //< AE Mode
        AE_HIST_DEFAULTS_AR0144,
        AE_MSV_DEFAULTS_AR0144,
        0,                          //< Standby Enabled
        1                           //< Standby Decimator
    };



#ifdef APQ8096

    static const PerCameraInfo IMX214Defaults =
        {
            "",                         //< Name
            SENSOR_IMX214,              //< Sensor
            BAYER_RGGB,                 //< bayer pixel format
            -1,                         //< ID
            -1,                         //< ID2
            1,                          //< Enabled?
            30,                         //< Framerate
            0,                          //< en_rotate
            0,                          //< en_rotate2
            0,                          //< Enable Preview Mode?
            640,                        //< Preview Width of the frame
            480,                        //< Preview Height of the frame
            0,                          //< en_raw_preview
            1,                          //< Enable Small Video
            1024,                       //< Small Video Width of the frame
            768,                        //< Small Video Height of the frame
            VENC_H264_SMALL_DEFAULT,
            1,                          //< Enable Large Video
            1920,                       //< Large Video Width of the frame
            1080,                       //< Large Video Height of the frame
            VENC_H264_LARGE_DEFAULT,
            1,                          //< Enable Snapshot mode?
            4160,                       //< Snapshot Width of the frame
            3120,                       //< Snapshot Height of the frame
            0,                          //< Independent Exposure
            AE_ISP,                     //< AE Mode
            AE_HIST_DEFAULTS_OV7251,
            AE_MSV_DEFAULTS_OV7251,
            0,                          //< Standby Enabled
            1                           //< Standby Decimator
        };

// all other platforms other than APQ8096 use normal defaults
#else

    static const PerCameraInfo IMX214Defaults =
        {
            "",                         //< Name
            SENSOR_IMX214,              //< Sensor
            BAYER_RGGB,                 //< bayer pixel format
            -1,                         //< ID
            -1,                         //< ID2
            1,                          //< Enabled?
            30,                         //< Framerate
            0,                          //< en_rotate
            0,                          //< en_rotate2
            0,                          //< Enable Preview Mode?
            640,                        //< Preview Width of the frame
            480,                        //< Preview Height of the frame
            0,                          //< en_raw_preview
            1,                          //< Enable Small Video
            1024,                       //< Small Video Width of the frame
            768,                        //< Small Video Height of the frame
            VENC_H265_SMALL_DEFAULT,
            1,                          //< Enable Large Video
            4208,                       //< Large Video Width of the frame
            3120,                       //< Large Video Height of the frame
            VENC_H265_LARGE_DEFAULT,
            1,                          //< Enable Snapshot mode?
            4208,                       //< Snapshot Width of the frame
            3120,                       //< Snapshot Height of the frame
            0,                          //< Independent Exposure
            AE_ISP,                     //< AE Mode
            AE_HIST_DEFAULTS_OV7251,
            AE_MSV_DEFAULTS_OV7251,
            0,                          //< Standby Enabled
            1                           //< Standby Decimator
        };

#endif // end ifdef APQ8096



// this is for SDK1.0.0 release with system image 1.6.4 and prior which is
// before camx was updated to allow full native res of 4208x3120
// after sys image 1.7.1 the struct above is used instead
static const PerCameraInfo IMX214DefaultsOld =
    {
        "",                         //< Name
        SENSOR_IMX214,              //< Sensor
        BAYER_RGGB,                 //< bayer pixel format
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        30,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        0,                          //< Enable Preview Mode?
        640,                        //< Preview Width of the frame
        480,                        //< Preview Height of the frame
        0,                          //< en_raw_preview
#ifdef APQ8096
        1,                          //< Enable Small Video
        1024,                       //< Small Video Width of the frame
        768,                        //< Small Video Height of the frame
        VENC_H264_SMALL_DEFAULT,
        1,                          //< Enable Large Video
        1920,                       //< Large Video Width of the frame
        1080,                       //< Large Video Height of the frame
        VENC_H264_LARGE_DEFAULT,
#else
        1,                          //< Enable Small Video
        1024,                       //< Small Video Width of the frame
        768,                        //< Small Video Height of the frame
        VENC_H265_SMALL_DEFAULT,
        1,                          //< Enable Large Video
        4096,                       //< Large Video Width of the frame
        2160,                       //< Large Video Height of the frame
        VENC_H265_LARGE_DEFAULT,
#endif
        1,                          //< Enable Snapshot mode?
        4160,                       //< Snapshot Width of the frame
        3120,                       //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_ISP,                     //< AE Mode
        AE_HIST_DEFAULTS_OV7251,
        AE_MSV_DEFAULTS_OV7251,
        0,                          //< Standby Enabled
        1                           //< Standby Decimator
    };


static const PerCameraInfo IMX412Defaults =
    {
        "",                         //< Name
        SENSOR_IMX412,              //< Sensor
        BAYER_RGGB,                 //< bayer pixel format TODO CHECK THIS
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        30,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        0,                          //< Enable Preview Mode?
        640,                        //< Preview Width of the frame
        480,                        //< Preview Height of the frame
        0,                          //< en_raw_preview
        1,                          //< Enable Small Video
        1024,                       //< Small Video Width of the frame
        768,                        //< Small Video Height of the frame
        VENC_H265_SMALL_DEFAULT,
        1,                          //< Enable Large Video
        4056,                       //< Large Video Width of the frame
        3040,                       //< Large Video Height of the frame
        VENC_H265_LARGE_DEFAULT,
        1,                          //< Enable Snapshot mode?
        4056,                       //< Snapshot Width of the frame
        3040,                       //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_ISP,                     //< AE Mode
        AE_HIST_DEFAULTS_OV7251,
        AE_MSV_DEFAULTS_OV7251,
        0,                          //< Standby Enabled
        1                           //< Standby Decimator
    };


// this is for SDK1.0.0 release with system image 1.6.4 and prior which is
// before camx was updated to allow full native res of 4208x3120
// after sys image 1.7.1 the struct above is used instead
static const PerCameraInfo IMX412DefaultsOld =
    {
        "",                         //< Name
        SENSOR_IMX412,              //< Sensor
        BAYER_RGGB,                 //< bayer pixel format TODO CHECK THIS
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        30,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        0,                          //< Enable Preview Mode?
        640,                        //< Preview Width of the frame
        480,                        //< Preview Height of the frame
        0,                          //< en_raw_preview
        1,                          //< Enable Small Video
        1024,                       //< Small Video Width of the frame
        768,                        //< Small Video Height of the frame
        VENC_H265_SMALL_DEFAULT,
        1,                          //< Enable Large Video
        2048,                       //< Large Video Width of the frame
        1536,                       //< Large Video Height of the frame
        VENC_H265_LARGE_DEFAULT,
        1,                          //< Enable Snapshot mode?
        4000,                       //< Snapshot Width of the frame
        3000,                       //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_ISP,                     //< AE Mode
        AE_HIST_DEFAULTS_OV7251,
        AE_MSV_DEFAULTS_OV7251,
        0,                          //< Standby Enabled
        1                           //< Standby Decimator
    };


static const PerCameraInfo IMX678Defaults =
    {
        "",                         //< Name
        SENSOR_IMX678,              //< Sensor
        BAYER_RGGB,                 //< bayer pixel format TODO CHECK THIS
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        30,                         //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        0,                          //< Enable Preview Mode?
        640,                        //< Preview Width of the frame
        480,                        //< Preview Height of the frame
        0,                          //< en_raw_preview
        1,                          //< Enable Small Video
        1024,                       //< Small Video Width of the frame
        768,                        //< Small Video Height of the frame
        VENC_H265_SMALL_DEFAULT,
        1,                          //< Enable Large Video
        2048,                       //< Large Video Width of the frame
        1536,                       //< Large Video Height of the frame
        VENC_H265_LARGE_DEFAULT,
        1,                          //< Enable Snapshot mode?
        3840,                       //< Snapshot Width of the frame
        2160,                       //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_ISP,                     //< AE Mode
        AE_HIST_DEFAULTS_OV7251,
        AE_MSV_DEFAULTS_OV7251,
        0,                          //< Standby Enabled
        1                           //< Standby Decimator
    };


static const PerCameraInfo TOFDefaults =
    {
        "",                         //Name
        SENSOR_TOF,                 //< Sensor
        BAYER_MONO,                 //< bayer pixel format
        -1,                         //< ID
        -1,                         //< ID2
        1,                          //< Enabled?
        5,                          //< Framerate
        0,                          //< en_rotate
        0,                          //< en_rotate2
        1,                          //< Enable Preview Mode?
        224,                        //< Preview Width of the frame
        1557,                       //< Preview Height of the frame
        1,                          //< en_raw_preview
        0,                          //< Enable Small Video
        -1,                         //< Small Video Width of the frame
        -1,                         //< Small Video Height of the frame
        VENC_H264_SMALL_DEFAULT,
         0,                          //< Enable Large Video
        -1,                         //< Large Video Width of the frame
        -1,                         //< Large Video Height of the frame
        VENC_H264_LARGE_DEFAULT,
        0,                          //< Enable Snapshot mode?
        -1,                         //< Snapshot Width of the frame
        -1,                         //< Snapshot Height of the frame
        0,                          //< Independent Exposure
        AE_OFF,                     //< AE Mode
        AE_HIST_DEFAULTS_OV7251,
        AE_MSV_DEFAULTS_OV7251,
        0,                          //< Standby Enabled
        5                           //< Standby Decimator
    };


static int _is_sys_image_same_or_newer_than(int majorA, int minorA, int patchA)
{
#ifdef APQ8096
    return 1;
#endif

    char buf[16];
    FILE* fd = fopen("/etc/version", "r");
    if(fd<=0){
        perror("unable to open /etc/version");
        return -1;
    }

    int n = fread(buf, 1, 16, fd);
    fclose(fd);
    if(n<5){
        M_ERROR("failed to read from /etc/version, ret=%d\n", n);
        return -1;
    }

    int majorB;
    int minorB;
    int patchB;

    n = sscanf(buf, "%d.%d.%d", &majorB, &minorB, &patchB);
    if(n!=3){
        M_ERROR("ERROR failed to parse version string %s from /etc/version\n", buf);
    }
    M_PRINT("detected system image %d.%d.%d\n", majorB, minorB, patchB);

    return (majorB > majorA) || \
            (majorB == majorA && (minorB > minorA || (minorB == minorA && patchB >= patchA)));
}



const PerCameraInfo getDefaultCameraInfo(sensor_t t) {
    switch(t){
        case SENSOR_OV7251:
            return OV7251Defaults;
        case SENSOR_OV9782:
            return OV9782Defaults;
         case SENSOR_AR0144:
            return AR0144Defaults;
        case SENSOR_IMX214:
            if(_is_sys_image_same_or_newer_than(1,7,1)){
                printf("using new imx214 defaults\n");
                return IMX214Defaults;
            }
            printf("using old imx214 defaults\n");
            return IMX214DefaultsOld;
        case SENSOR_IMX412:
            if(_is_sys_image_same_or_newer_than(1,7,1)){
                printf("using new imx412 defaults\n");
                return IMX412Defaults;
            }
            printf("using old imx412 defaults\n");
            return IMX412DefaultsOld;
        case SENSOR_IMX678:
            return IMX678Defaults;
        case SENSOR_TOF:
            return TOFDefaults;

        default:
            return emptyDefaults;
    }
}
