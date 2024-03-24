/*
See	MPU-9250 Register Map and Descriptions, Revision 4.0,
RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
above document; the MPU9250 and MPU9150 are virtually
identical but the latter has a different register map
*/

#ifndef ICM20948_DEFS_H
#define ICM20948_DEFS_H


////////////////////////////////////////////////////////////////////////////////
// bank 0 register map
////////////////////////////////////////////////////////////////////////////////
#define REG_WHO_AM_I				0x00
#define REG_LPF						0x01

#define REG_USER_CTRL				0x03
#define BIT_DMP_EN					0x80
#define BIT_FIFO_EN					0x40
#define BIT_I2C_MST_EN				0x20
#define BIT_I2C_IF_DIS				0x10
#define BIT_DMP_RST					0x08
#define BIT_DIAMOND_DMP_RST			0x04

#define REG_LP_CONFIG				0x05
#define BIT_I2C_MST_CYCLE			0x40
#define BIT_ACCEL_CYCLE				0x20
#define BIT_GYRO_CYCLE				0x10

#define REG_PWR_MGMT_1				0x06
#define BIT_H_RESET					0x80
#define BIT_SLEEP					0x40
#define BIT_LP_EN					0x20
#define BIT_CLK_PLL					0x01

#define REG_PWR_MGMT_2				0x07
#define BIT_PWR_PRESSURE_STBY		0x40
#define BIT_PWR_ACCEL_STBY			0x38
#define BIT_PWR_GYRO_STBY			0x07
#define BIT_PWR_ALL_OFF				0x7f

#define REG_INT_PIN_CFG				0x0F
#define BIT_INT_LATCH_EN			0x20
#define BIT_BYPASS_EN				0x02

#define REG_INT_ENABLE				0x10
#define BIT_DMP_INT_EN				0x02

#define REG_INT_ENABLE_1			0x11
#define BIT_DATA_RDY_3_EN			0x08
#define BIT_DATA_RDY_2_EN			0x04
#define BIT_DATA_RDY_1_EN			0x02
#define BIT_DATA_RDY_0_EN			0x01

#define REG_INT_ENABLE_2			0x12
#define BIT_FIFO_OVERFLOW_EN_0		0x1

#define REG_INT_ENABLE_3			0x13

#define REG_DMP_INT_STATUS			0x18
#define BIT_WAKE_ON_MOTION_INT		0x08
#define BIT_MSG_DMP_INT				0x0002
#define BIT_MSG_DMP_INT_0			0x0100	// CI Command

#define BIT_MSG_DMP_INT_2			0x0200	// CIM Command - SMD
#define BIT_MSG_DMP_INT_3			0x0400	// CIM Command - Pedometer

#define BIT_MSG_DMP_INT_4			0x1000	// CIM Command - Pedometer binning
#define BIT_MSG_DMP_INT_5			0x2000	// CIM Command - Bring To See Gesture
#define BIT_MSG_DMP_INT_6			0x4000	// CIM Command - Look To See Gesture

#define REG_INT_STATUS				0x19
#define BIT_DMP_INT					0x02

#define REG_INT_STATUS_1			0x1A
#define REG_INT_STATUS_2			0x1B

#define REG_SINGLE_FIFO_PRIORITY_SEL	0x26

#define REG_ACCEL_XOUT_H_SH			0x2D
#define REG_ACCEL_XOUT_L_SH			0x2E
#define REG_ACCEL_YOUT_H_SH			0x2F
#define REG_ACCEL_YOUT_L_SH			0x30
#define REG_ACCEL_ZOUT_H_SH			0x31
#define REG_ACCEL_ZOUT_L_SH			0x32
#define REG_GYRO_XOUT_H_SH			0x33
#define REG_GYRO_XOUT_L_SH			0x34
#define REG_GYRO_YOUT_H_SH			0x35
#define REG_GYRO_YOUT_L_SH			0x36
#define REG_GYRO_ZOUT_H_SH			0x37
#define REG_GYRO_ZOUT_L_SH			0x38
#define REG_TEMPERATURE				0x39
#define TEMP_SENSITIVITY			333.87f // degC/LSB


#define REG_EXT_SLV_SENS_DATA_00	0x3B
#define REG_EXT_SLV_SENS_DATA_08	0x43
#define REG_EXT_SLV_SENS_DATA_09	0x44
#define REG_EXT_SLV_SENS_DATA_10	0x45

#define REG_FIFO_EN					0x66
#define BIT_SLV_0_FIFO_EN			0x01

#define REG_FIFO_EN_2				0x67
#define BIT_PRS_FIFO_EN				0x20
#define BIT_ACCEL_FIFO_EN			0x10
#define BITS_GYRO_FIFO_EN			0x0E
#define BIT_TEMP_FIFO_EN			0x01

#define REG_FIFO_RST				0x68

#define REG_FIFO_COUNT_H			0x70
#define REG_FIFO_COUNT_L			0x71
#define REG_FIFO_R_W				0x72

#define REG_HW_FIX_DISABLE			0x75

#define REG_FIFO_CFG				0x76
#define BIT_MULTI_FIFO_CFG			0x01
#define BIT_SINGLE_FIFO_CFG			0x00



#define REG_MEM_START_ADDR			0x7C
#define REG_MEM_R_W					0x7D
#define REG_MEM_BANK_SEL			0x7E



////////////////////////////////////////////////////////////////////////////////
// bank 1 register map
////////////////////////////////////////////////////////////////////////////////
#define REG_TIMEBASE_CORRECTION_PLL	0x28
#define REG_TIMEBASE_CORRECTION_RCOSC 0x29

#define REG_SELF_TEST_X_GYRO		0x02
#define REG_SELF_TEST_Y_GYRO		0x03
#define REG_SELF_TEST_Z_GYRO		0x04
#define REG_SELF_TEST_X_ACCEL		0x0E
#define REG_SELF_TEST_Y_ACCEL		0x0F
#define REG_SELF_TEST_Z_ACCEL		0x10

#define REG_XA_OFFS_H				0x14
#define REG_XA_OFFS_L				0x15
#define REG_YA_OFFS_H				0x17
#define REG_YA_OFFS_L				0x18
#define REG_ZA_OFFS_H				0x1A
#define REG_ZA_OFFS_L				0x1B




////////////////////////////////////////////////////////////////////////////////
// bank 2 register map
////////////////////////////////////////////////////////////////////////////////
#define REG_GYRO_SMPLRT_DIV			0x00

#define REG_GYRO_CONFIG_1			0x01
#define GYRO_FSR_CFG_250			(0x00<<1)
#define GYRO_FSR_CFG_500			(0x01<<1)
#define GYRO_FSR_CFG_1000			(0x02<<1)
#define GYRO_FSR_CFG_2000			(0x03<<1)
#define GYRO_FCHOICE_EN_DLPF		1

#define REG_GYRO_CONFIG_2			0x02
#define GYRO_SELF_TEST_EN			0x38

#define REG_XG_OFFS_USRH			0x03
#define REG_XG_OFFS_USRL			0x04
#define REG_YG_OFFS_USRH			0x05
#define REG_YG_OFFS_USRL			0x06
#define REG_ZG_OFFS_USRH			0x07
#define REG_ZG_OFFS_USRL			0x08

#define REG_ACCEL_SMPLRT_DIV_1		0x10
#define REG_ACCEL_SMPLRT_DIV_2		0x11

#define REG_ACCEL_CONFIG			0x14
#define ACCEL_FSR_CFG_2G			(0x00<<1)
#define ACCEL_FSR_CFG_4G			(0x01<<1)
#define ACCEL_FSR_CFG_8G			(0x02<<1)
#define ACCEL_FSR_CFG_16G			(0x03<<1)
#define ACCEL_FCHOICE_EN_DLPF		1

#define REG_ACCEL_CONFIG_2			0x15
#define ACCEL_SELF_TEST_EN			0x1C

#define REG_PRS_ODR_CONFIG			0x20
#define REG_PRGM_START_ADDRH		0x50

#define REG_TEMP_CONFIG				0x53

#define REG_MOD_CTRL_USR			0x54
#define BIT_ODR_SYNC				0x7

////////////////////////////////////////////////////////////////////////////////
// bank 3 register map
////////////////////////////////////////////////////////////////////////////////
#define REG_I2C_MST_ODR_CONFIG		0x0

#define REG_I2C_MST_CTRL			0x01
#define BIT_I2C_MST_P_NSR			0x10

#define REG_I2C_MST_DELAY_CTRL		0x02
#define BIT_SLV0_DLY_EN				0x01
#define BIT_SLV1_DLY_EN				0x02
#define BIT_SLV2_DLY_EN				0x04
#define BIT_SLV3_DLY_EN				0x08

#define REG_I2C_SLV0_ADDR			0x03
#define REG_I2C_SLV0_REG			0x04
#define REG_I2C_SLV0_CTRL			0x05
#define REG_I2C_SLV0_DO				0x06

#define REG_I2C_SLV1_ADDR			0x07
#define REG_I2C_SLV1_REG			0x08
#define REG_I2C_SLV1_CTRL			0x09
#define REG_I2C_SLV1_DO				0x0A

#define REG_I2C_SLV2_ADDR			0x0B
#define REG_I2C_SLV2_REG			0x0C
#define REG_I2C_SLV2_CTRL			0x0D
#define REG_I2C_SLV2_DO				0x0E

#define REG_I2C_SLV3_ADDR			0x0F
#define REG_I2C_SLV3_REG			0x10
#define REG_I2C_SLV3_CTRL			0x11
#define REG_I2C_SLV3_DO				0x12

#define REG_I2C_SLV4_CTRL			0x15

#define INV_MPU_BIT_SLV_EN			0x80
#define INV_MPU_BIT_BYTE_SW			0x40
#define INV_MPU_BIT_REG_DIS			0x20
#define INV_MPU_BIT_GRP				0x10
#define INV_MPU_BIT_I2C_READ		0x80

/* register for all banks */
#define REG_BANK_SEL				0x7F










// Table for list of results for factory self-test value equation
// st_otp = 2620/2^FS * 1.01^(st_value - 1)
// for gyro and accel FS = 0 so 2620 * 1.01^(st_value - 1)
// st_value = 1 => 2620
// st_value = 2 => 2620 * 1.01 = 2646
// etc../
static const uint16_t sSelfTestEquation[256] = {
	2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
	2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
	3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
	3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
	3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
	3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
	4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
	4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
	4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
	5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
	5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
	6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
	6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
	7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
	7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
	8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
	9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
	10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
	10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
	11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
	12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
	13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
	15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
	16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
	17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
	19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
	22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
	24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
	26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
	28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
	30903, 31212, 31524, 31839, 32157, 32479, 32804
};
/* full scale and LPF setting */
#define SELFTEST_GYRO_FS            ((0 << 3) | 1) // 250dps
#define SELFTEST_ACCEL_FS           ((7 << 3) | 1) // 16g

#define INV_ABS(x) (((x) < 0) ? -(x) : (x))

// Formula to get ST_OTP based on FS and ST_code
#define INV_ST_OTP_EQUATION(FS, ST_code) (uint32_t)((2620/pow(2,3-FS))*pow(1.01, ST_code-1)+0.5)

// ModalAI derived Pass/Fail criteria based on our own testing
#define MIN_RATIO_GYRO 0.95f
#define MAX_RATIO_GYRO 1.05f
#define MIN_RATIO_ACCL 0.95f
#define MAX_RATIO_ACCL 1.05f
#define MAX_ST_GYRO_OFFSET_DPS	20	// expected offset within +-20 dps

// the following limits should never be used in practice.
// these are test criteria for the case where no factory data is present in
// the imu registers to compare against
#define MIN_ST_GYRO_RESPONSE	10000
#define MAX_ST_GYRO_RESPONSE	25000
#define MIN_ST_ACCEL_RESPONSE	5000
#define MAX_ST_ACCEL_RESPONSE	11000


#endif // ICM20948_DEFS_H
