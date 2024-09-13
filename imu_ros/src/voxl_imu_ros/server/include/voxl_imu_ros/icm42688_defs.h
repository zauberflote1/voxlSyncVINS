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



#ifndef ICM42688_DEFS_H
#define ICM42688_DEFS_H


// -----------------------------------------------------------------------------------------------------------------------------
// Bank 0 register offsets
// -----------------------------------------------------------------------------------------------------------------------------
enum ICM42688_USER_BANK_0_REG_ADDR
{
	ICM42688_UB0_REG_DEVICE_CONFIG       = 0,     // 0
	ICM42688_UB0_REG_FIFO_CONFIG         = 22,    // 0x16
	ICM42688_UB0_REG_TEMP_DATA1          = 29,    // 0x1D
	ICM42688_UB0_REG_TEMP_DATA0          = 30,    // 0x1E
	ICM42688_UB0_REG_ACCEL_DATA_X1       = 31,    // 0x1F
	ICM42688_UB0_REG_ACCEL_DATA_X0       = 32,    // 0x20
	ICM42688_UB0_REG_ACCEL_DATA_Y1       = 33,    // 0x21
	ICM42688_UB0_REG_ACCEL_DATA_Y0       = 34,    // 0x22
	ICM42688_UB0_REG_GYRO_DATA_X1        = 37,    // 0x25
	ICM42688_UB0_REG_INT_STATUS          = 45,    // 0x2D
	ICM42688_UB0_REG_FIFO_COUNTH         = 46,    // 0x2E
	ICM42688_UB0_REG_FIFO_COUNTL         = 47,    // 0x2F
	ICM42688_UB0_REG_FIFO_DATA           = 48,    // 0x30
	ICM42688_UB0_REG_SIGNAL_PATH_RESET   = 75,    // 0x4B
	ICM42688_UB0_REG_INTF_CONFIG0        = 76,    // 0x4C
	ICM42688_UB0_REG_INTF_CONFIG1        = 77,    // 0x4D
	ICM42688_UB0_REG_PWR_MGMT0           = 78,    // 0x4E
	ICM42688_UB0_REG_GYRO_CONFIG0        = 79,    // 0x4F
	ICM42688_UB0_REG_ACCEL_CONFIG0       = 80,    // 0x50
	ICM42688_UB0_REG_GYRO_CONFIG1        = 81,    // 0x51
	ICM42688_UB0_REG_GYRO_ACCEL_CONFIG0  = 82,    // 0x52
	ICM42688_UB0_REG_ACCEL_CONFIG1       = 83,    // 0x53
	ICM42688_UB0_REG_TMST_CONFIG         = 84,    // 0x54
	ICM42688_UB0_REG_FIFO_CONFIG1        = 95,    // 0x5F
	ICM42688_UB0_REG_FIFO_CONFIG2        = 96,    // 0x60
	ICM42688_UB0_REG_FIFO_CONFIG3        = 97,    // 0x61
	ICM42688_UB0_REG_INT_CONFIG1         = 100,   // 0x64
	ICM42688_UB0_REG_SELF_TEST_CONFIG    = 112,   // 0x70
	ICM42688_UB0_REG_WHOAMI              = 117,   // 0x75
	ICM42688_UB0_REG_BANK_SEL            = 118,   // 0x76
};

// -----------------------------------------------------------------------------------------------------------------------------
// Bank 1 register offsets
// -----------------------------------------------------------------------------------------------------------------------------
enum ICM42688_USER_BANK_1_REG_ADDR
{
	ICM42688_UB1_REG_SENSOR_CONFIG0        = 3,
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC2   = 11,   // 0xB
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC3   = 12,   // 0xC
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC4   = 13,   // 0xD
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC5   = 14,   // 0xE
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC6   = 15,   // 0xF
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC7   = 16,   // 0x10
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC8   = 17,   // 0x11
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC9   = 18,   // 0x12
	ICM42688_UB1_REG_GYRO_CONFIG_STATIC10  = 19,   // 0x13
	ICM42688_UB1_REG_ICXG_ST_DATA          = 95,   // 0x5F
	ICM42688_UB1_REG_ICYG_ST_DATA          = 96,   // 0x60
	ICM42688_UB1_REG_ICZG_ST_DATA          = 97,   // 0x61
	ICM42688_UB1_REG_INTF_CONFIG5          = 123,  // 0x7B
};

// -----------------------------------------------------------------------------------------------------------------------------
// Bank 2 register offsets
// -----------------------------------------------------------------------------------------------------------------------------
enum ICM42688_USER_BANK_2_REG_ADDR
{
	ICM42688_UB1_REG_ACCEL_CONFIG_STATIC2  = 3,
	ICM42688_UB1_REG_ACCEL_CONFIG_STATIC3  = 4,
	ICM42688_UB1_REG_ACCEL_CONFIG_STATIC4  = 5,
	ICM42688_UB1_REG_XA_ST_DATA            = 59,   // 0x3B
	ICM42688_UB1_REG_YA_ST_DATA            = 60,   // 0x3C
	ICM42688_UB1_REG_ZA_ST_DATA            = 61,   // 0x3D
};

// -----------------------------------------------------------------------------------------------------------------------------
// Bank 4 register offsets
// -----------------------------------------------------------------------------------------------------------------------------
enum ICM42688_USER_BANK_4_REG_ADDR
{
	ICM42688_UB4_REG_OFFSET_USER0 = 119,   // 0x77 Lower bits of X-gyro offset. Max value is +/-64dps
	ICM42688_UB4_REG_OFFSET_USER1 = 120,   // 0x78 Upper bits of X+Y-gyro offset. Max value is +/-64dps
	ICM42688_UB4_REG_OFFSET_USER2 = 121,   // 0x79 Lower bits of Y-gyro offset. Max value is +/-64dps
	ICM42688_UB4_REG_OFFSET_USER3 = 122,   // 0x7A Lower bits of Z-gyro offset. Max value is +/-64dps
	ICM42688_UB4_REG_OFFSET_USER4 = 123,   // 0x7B Upper bits of X-accel/Z-gyro offset offset. Max value is +/-1g, +/-64dps
	ICM42688_UB4_REG_OFFSET_USER5 = 124,   // 0x7C Lower bits of X-accel offset offset. Max value is +/-1g
	ICM42688_UB4_REG_OFFSET_USER6 = 125,   // 0x7D Lower bits of Y-accel offset programmed by user. Max value is ±1g,
	ICM42688_UB4_REG_OFFSET_USER7 = 126,   // 0x7E Upper bits of Z-accel, Y-accel. Max value is +/-1g
	ICM42688_UB4_REG_OFFSET_USER8 = 127,   // 0x7F Lower bits of Z-accel. Max value is +/-1g
};


// DEVICE_CONFIG
#define SPI_MODE_03					(0<<4) // default
#define SPI_MODE_12					(1<<4)
#define SOFT_RESET_CONFIG			(1<<0) // wait 1ms for reset

// INTF_CONFIG0 options
#define FIFO_HOLD_LAST_DATA_EN		(1<<7)
#define FIFO_HOLD_LAST_DATA_DIS		(0<<7)
#define FIFO_COUNT_REC_BYTES		(0<<6)
#define FIFO_COUNT_REC_RECORDS		(1<<6)
#define FIFO_COUNT_ENDIAN_BIG		(1<<5)
#define FIFO_COUNT_ENDIAN_LITTLE	(0<<5)
#define SENSOR_DATA_ENDIAN_BIG		(1<<4)
#define SENSOR_DATA_ENDIAN_LITTLE	(0<<4)
#define UI_SIFS_CFG_DISABLE_SPI		(2)
#define UI_SIFS_CFG_DISABLE_I2C		(3)

// Gyro FSR values for GYRO_CONFIG0
#define GYRO_FS_SEL_2000_DPS		(0<<5)
#define GYRO_FS_SEL_1000_DPS		(1<<5)
#define GYRO_FS_SEL_500_DPS			(2<<5)
#define GYRO_FS_SEL_250_DPS			(3<<5)
#define GYRO_FS_SEL_125_DPS			(4<<5)
#define GYRO_FS_SEL_62_5_DPS		(5<<5)
#define GYRO_FS_SEL_31_25_DPS		(6<<5)
#define GYRO_FS_SEL_15_625_DPS		(7<<5)

// ACCEL FSR values for ACCEL_CONFIG0
#define ACCEL_FS_SEL_16_G			(0<<5)
#define ACCEL_FS_SEL_8_G			(1<<5)
#define ACCEL_FS_SEL_4_G			(2<<5)
#define ACCEL_FS_SEL_2_G			(3<<5)

// UI filter order for accel ACCEL_CONFIG1
#define ACCEL_UI_FILT_ORD_1			(0<<3)
#define ACCEL_UI_FILT_ORD_2			(1<<3)
#define ACCEL_UI_FILT_ORD_3			(2<<3)
#define ACCEL_DEC2_M2_ORD_3			(2<<1)

// UI filter order for gyro GYRO_CONFIG1
#define TEMP_FILT_BW_4000			(0<<5)
#define TEMP_FILT_BW_170			(1<<5)
#define TEMP_FILT_BW_82				(2<<5)
#define TEMP_FILT_BW_40				(3<<5)
#define TEMP_FILT_BW_20				(4<<5)
#define TEMP_FILT_BW_10				(5<<5)
#define TEMP_FILT_BW_5				(6<<5)
#define GYRO_UI_FILT_ORD_1			(0<<2)
#define GYRO_UI_FILT_ORD_2			(1<<2)
#define GYRO_UI_FILT_ORD_3			(2<<2)
#define GYRO_DEC2_M2_ORD_3			(2<<0)

// FIFO configuration in FIFO_CONFIG0
#define FIFO_MODE_BYPASS			(0<<6)
#define FIFO_MODE_STREAM			(1<<6)
#define FIFO_MODE_STOP_ON_FULL		(2<<6)

// FIFO configuration in FIFO_CONFIG1
#define FIFO_RESUME_PARTIAL_RD_DIS	(0<<6)
#define FIFO_RESUME_PARTIAL_RD_EN	(1<<6)
#define FIFO_HIRES_EN				(1<<4)
#define FIFO_TEMP_EN				(1<<2)
#define FIFO_GYRO_EN				(1<<1)
#define FIFO_ACCEL_EN				(1<<0)

// PWR_MGMT0
#define TEMP_DIS					(1<<5)
#define TEMP_EN						(0<<5)
#define GYRO_MODE_OFF				(0<<2)
#define GYRO_MODE_STDBY				(1<<2)
#define GYRO_MODE_LOW_NOISE			(3<<2)
#define ACCEL_MODE_OFF				(0<<0)
#define ACCEL_MODE_LOW_POWER		(2<<0)
#define ACCEL_MODE_LOW_NOISE		(3<<0)

// SIGNAL_PATH_RESET
#define DMP_INIT_EN					(1<<6)
#define DMP_MEM_RESET_EN			(1<<5)
#define ABORT_AND_RESET				(1<<3)
#define TMST_STROBE					(1<<2)
#define FIFO_FLUSH					(1<<1)

// TMST_CONFIG
#define TMST_DEFAULT				(0x23)
#define TMST_TO_REGS_EN				(1<<4)
#define TMST_TO_REGS_DIS			(0<<4)
#define TMST_RES_1US				(0<<3) // default
#define TMST_RES_16US				(1<<3)
#define TMST_DELTA_EN				(1<<2)
#define TMST_DELTA_DIS				(0<<2) // default
#define TMST_FSYNC_EN				(1<<1) // default
#define TMST_FSYNC_DIS				(0<<0)
#define TMST_EN						(1<<0) // default
#define TMST_DIS					(0<<0)

// INT_STATUS
#define UI_FSYNC_INT				(1<<6)
#define PLL_RDY_INT					(1<<5)
#define RESET_DONE_INT				(1<<4)
#define DATA_RDY_INT				(1<<3)
#define FIFO_THS_INT				(1<<2)
#define FIFO_FULL_INT				(1<<1)
#define AGC_RDY_INT					(1<<0)



////////////////////////////////////////////////////////////////////////////////
// Self test stuff
////////////////////////////////////////////////////////////////////////////////

#define INV_ABS(x) (((x) < 0) ? -(x) : (x))

// Formula to get ST_OTP based on FS and ST_code
#define INV_ST_OTP_EQUATION(FS, ST_code) (uint32_t)((2620/pow(2,3-FS))*pow(1.01, ST_code-1)+0.5)

// ModalAI derived Pass/Fail criteria based on our own testing
#define MIN_RATIO_GYRO 0.95f
#define MAX_RATIO_GYRO 1.05f
#define MIN_RATIO_ACCL 0.90f
#define MAX_RATIO_ACCL 1.05f
#define MAX_ST_GYRO_OFFSET_DPS	20	// expected offset within +-20 dps

// the following limits should never be used in practice.
// these are test criteria for the case where no factory data is present in
// the imu registers to compare against
#define MIN_ST_GYRO_RESPONSE	10000
#define MAX_ST_GYRO_RESPONSE	20000
#define MIN_ST_ACCEL_RESPONSE	1000
#define MAX_ST_ACCEL_RESPONSE	10000







#endif // ICM42688_DEFS_H