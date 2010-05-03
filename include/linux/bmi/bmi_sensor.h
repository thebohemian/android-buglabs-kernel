/*
 * File:         include/linux/bmi/bmi_sensor.h
 * Author:       Peter Giacomini <p.giacomini@encadis.com>
 *
 * 		This is the application header file for the BMI bus sensor plug-in
 * 		module on the MX31 BUG platform.
 */

#ifndef BMI_SENSOR_H
#define BMI_SENSOR_H

#include <linux/types.h>
#include <linux/bmi/bmi_ioctl.h>

// GPIO
#define SENSOR_GPIO_RED_LED	3	// output
#define SENSOR_GPIO_GREEN_LED	2	// output
#define SENSOR_GPIO_PDOUT	1	// input - aproximity detector state
#define SENSOR_GPIO_MOT_DET	0	// input - motion detector state

// I2C
// I2C Slave Addresses
#define BMI_MEE_I2C_ADDRESS	0x51	// 7-bit address - Module specific EEPROM
#define BMI_IOX_I2C_ADDRESS	0x74	// 7-bit address - 2 banks I2C IO expander
#define BMI_ADC_I2C_ADDRESS	0x48	// 7-bit address - ADC - humidity/acompass/sound/alight/aproximity
#define BMI_PL_I2C_ADDRESS	0x44	// 7-bit address - digital proximity/light
#define BMI_DLIGHT_I2C_ADDRESS	0x44	// 7-bit address - digital light
#define BMI_TEMP_I2C_ADDRESS	0x4C	// 7-bit address - temperature
#define BMI_ACC_I2C_ADDRESS	0x1D	// 7-bit address - accelerometer
#define BMI_DCOMP_I2C_ADDRESS	0x1C	// 7-bit address - digital compass

// I2C IOX register addresses
#define IOX_INPUT0_REG		0x0
#define IOX_INPUT1_REG		0x1
#define IOX_OUTPUT0_REG		0x2
#define IOX_OUTPUT1_REG		0x3
#define IOX_POLARITY0_REG	0x4
#define IOX_POLARITY1_REG	0x5
#define IOX_CONTROL0_REG	0x6
#define IOX_CONTROL1_REG	0x7

// IOX bit definitions
// bank 0
#define	SENSOR_IOX_ACC_INT1	0	// Input - Accelerometer interrupt 1
#define	SENSOR_IOX_ACC_INT2	1	// Input - Accelerometer interrupt 2
#define	SENSOR_IOX_USB_FL_N	2	// Input - USB power interrupt
#define	SENSOR_IOX_USB_EN	3	// Output - USB power enable
#define	SENSOR_IOX_HUM_EN	4	// Output - Humidity sensor power enable
#define	SENSOR_IOX_MOT_DET	5	// Input - Motion Detector interrupt
#define	SENSOR_IOX_MOT_EN	6	// Output - Motion Detector interrupt enable
#define	SENSOR_IOX_COMP_RS_N	7	// Output - A/D Compass Reset (see Honeywell AN213)
#define SENSOR_IOX_USB_BIT  0x08
#define SENSOR_IOX_MOT_BIT  0x40

// bank 1
#define	SENSOR_IOX_PROX_RST_N	0	// Output - Analog Proximity sensor reset
#define	SENSOR_IOX_PROX_EN_N	1	// Output - Analog Proximity sensor enable
#define	SENSOR_IOX_PROX_OUT	2	// Input - Analog Proximity sensor output
#define	SENSOR_IOX_S_PK_CLR_N	3	// Output - Sound peak detector clear
#define	SENSOR_IOX_TEMP_INT	4	// Input - Termperature interrupt
#define	SENSOR_IOX_PL_INT	5	// Input - Proximity/Light interrupt
#define	SENSOR_IOX_MIC_EN	6	// Output - Sound power enanle
#define	SENSOR_IOX_DCOMP_INT	7	// Input - Digital Compass Interrupt
#define SENSOR_IOX_MIC_BIT  0x40

// EEPROM contents
struct sensor_eeprom_raw
{
	__u8 xsf_msb;			/* byte 0x00  */ // analog and digital compass
	__u8 xsf_lsb;			/* byte 0x01  */ // analog and digital compass
	__u8 ysf_msb;			/* byte 0x02  */ // analog and digital compass
	__u8 ysf_lsb;			/* byte 0x03  */ // analog and digital compass
	__u8 zsf_msb;			/* byte 0x04  */ // analog and digital compass
	__u8 zsf_lsb;			/* byte 0x05  */ // analog and digital compass
	__u8 xoff_msb;			/* byte 0x06  */ // analog and digital compass
	__u8 xoff_lsb;			/* byte 0x07  */ // analog and digital compass
	__u8 yoff_msb;			/* byte 0x08  */ // analog and digital compass
	__u8 yoff_lsb;			/* byte 0x09  */ // analog and digital compass
	__u8 zoff_msb;			/* byte 0x0A  */ // analog and digital compass
	__u8 zoff_lsb;			/* byte 0x0B  */ // analog and digital compass
	__u8 xdac;			/* byte 0x0C  */ // digital compass
	__u8 ydac;			/* byte 0x0D  */ // digital compass
	__u8 zdac;			/* byte 0x0E  */ // digital compass
	__u8 adc_present;		/* byte 0x0F  - 0x1 == present */ // TI/Burr-Brown ADS7828
	__u8 humidity_present;		/* byte 0x10  - 0x1 == present */ // Honeywell HIH3030
	__u8 acompass_present;		/* byte 0x11  - 0x1 == present */ // Honeywell HMC6042/HMC1041Z
	__u8 light_proximity_present;	/* byte 0x12  - 0x1 == present */ // Intersil ISL29018
	__u8 sound_present;		/* byte 0x13  - 0x1 == present */ // discrete components
	__u8 temperature_present;	/* byte 0x14  - 0x1 == present */ // National LM95235
	__u8 motion_present;		/* byte 0x15  - 0x1 == present */ // Panasonic AMN44121
	__u8 acc_present;		/* byte 0x16  - 0x1 == present */ // Analog Devices ADXL345
	__u8 dcompass_present;		/* byte 0x17  - 0x1 == present */ // AsahiKASEI AK8973
	__u8 aproximity_present;	/* byte 0x18  - 0x1 == present */ // Avago APDS-9700
	__u8 alight_present;		/* byte 0x19 - 0x1 == present */  // Avago APDS-9002
	__u8 dlight_present;		/* byte 0x1A - 0x1 == present */  // Intersil ISL29003
	__u8 acc302_present;		/* byte 0x1B - 0x1 == present */  // ST LIS302DL
};
#define	SENSOR_DEVICE_NOT_PRESENT	(0x0)
#define	SENSOR_DEVICE_PRESENT		(0x1)
#define SENSOR_EE_SF_START		(0x00)
#define SENSOR_EE_OFF_START		(0x06)
#define SENSOR_EE_XDAC			(0x0C)
#define SENSOR_EE_YDAC			(0x0D)
#define SENSOR_EE_ZDAC			(0x0E)
#define SENSOR_PRESENT_START		(0x0F)
#define SENSOR_PRESENT_END		(0x1B)

struct sensor_comp_cal
{
	unsigned int xsf;
	unsigned int ysf;
	unsigned int zsf;
	unsigned int xoff;
	unsigned int yoff;
	unsigned int zoff;
};

struct sensor_comp_dac
{
	unsigned char xdac;
	unsigned char ydac;
	unsigned char zdac;
};

// ADC (ADS7828) - humidity/acompass/sound/alight/aproximity
// command write
#define SENSOR_ADC_D10P		(0x00 << 4)	// positive diff - ch0 & ch1
#define SENSOR_ADC_D23P		(0x01 << 4)	// positive diff - ch2 & ch3
#define SENSOR_ADC_D45P		(0x02 << 4)	// positive diff - ch4 & ch5
#define SENSOR_ADC_D67P		(0x03 << 4)	// positive diff - ch6 & ch7
#define SENSOR_ADC_D10N		(0x04 << 4)	// negative diff - ch0 & ch1
#define SENSOR_ADC_D23N		(0x05 << 4)	// negative diff - ch2 & ch3
#define SENSOR_ADC_D45N		(0x06 << 4)	// negative diff - ch4 & ch5
#define SENSOR_ADC_D67N		(0x07 << 4)	// negative diff - ch6 & ch7
#define SENSOR_ADC_CH0		(0x08 << 4)	// single ended ch0
#define SENSOR_ADC_CH2		(0x09 << 4)	// single ended ch2
#define SENSOR_ADC_CH4		(0x0A << 4)	// single ended ch4
#define SENSOR_ADC_CH6		(0x0B << 4)	// single ended ch6
#define SENSOR_ADC_CH1		(0x0C << 4)	// single ended ch1
#define SENSOR_ADC_CH3		(0x0D << 4)	// single ended ch3
#define SENSOR_ADC_CH5		(0x0E << 4)	// single ended ch5
#define SENSOR_ADC_CH7		(0x0F << 4)	// single ended ch7
#define SENSOR_ADC_PD_OFF	(0x00 << 2)	// full power down
#define SENSOR_ADC_PD_IR	(0x01 << 2)	// power down internal reference
#define SENSOR_ADC_PD_ADC	(0x02 << 2)	// power down ADC
#define SENSOR_ADC_PD_ON	(0x03 << 2)	// power up ADC
// data read
#define SENSOR_ADC_DATA_MSB	(0x0F)
#define SENSOR_ADC_DATA_LSB	(0xFF)

// ADC mapping
#define SENSOR_ADC_SOUND_PEAK	SENSOR_ADC_CH7
#define SENSOR_ADC_SOUND_AVG	SENSOR_ADC_CH6
#define SENSOR_ADC_APROXIMITY	SENSOR_ADC_CH5	// Analog proximity
#define SENSOR_ADC_HUMIDITY	SENSOR_ADC_CH4
#define SENSOR_ADC_LIGHT	SENSOR_ADC_CH3	// Analog light
#define SENSOR_ADC_ACOMPASS_Z	SENSOR_ADC_CH2	// Analog compass
#define SENSOR_ADC_ACOMPASS_Y	SENSOR_ADC_CH1	// Analog compass
#define SENSOR_ADC_ACOMPASS_X	SENSOR_ADC_CH0	// Analog compass

// Light/Proximity
#define SENSOR_PL_CMD1				(0x00)		// command I
	#define SENSOR_PL_CMD1_PD		(0x00 << 5)	// power down
	#define SENSOR_PL_CMD1_ALS_1X		(0x01 << 5)	// ALS once
	#define SENSOR_PL_CMD1_IR_1X		(0x02 << 5)	// IR once
	#define SENSOR_PL_CMD1_PROX_1X		(0x03 << 5)	// Proximity once
	#define SENSOR_PL_CMD1_ALS_CONT		(0x05 << 5)	// ALS continuous
	#define SENSOR_PL_CMD1_IR_CONT		(0x06 << 5)	// IR continuous
	#define SENSOR_PL_CMD1_PROX_CONT	(0x07 << 5)	// Proximity continuous
	#define SENSOR_PL_CMD1_INT_TIMING	(0x00)		// Proximity continuous
	#define SENSOR_PL_CMD1_EXT_TIMING	(0x10)		// Proximity continuous
	#define SENSOR_PL_CMD1_DATA_ADC		(0x00)		// data is ADC value
	#define SENSOR_PL_CMD1_DATA_TIMING	(0x08)		// data is ADC value
	#define SENSOR_PL_CMD1_INT_STAT		(0x04)		// interrupt status
	#define SENSOR_PL_CMD1_INT_1MS		(0x00)		// interrupt persist = 1 ms
	#define SENSOR_PL_CMD1_INT_4MS		(0x01)		// interrupt persist = 4 ms
	#define SENSOR_PL_CMD1_INT_8MS		(0x02)		// interrupt persist = 8 ms
	#define SENSOR_PL_CMD1_INT_16MS		(0x03)		// interrupt persist = 16 ms
#define SENSOR_PL_CMD2				(0x01)		// command II
	#define SENSOR_PL_CMD2_IR_LED_A		(0x00)		// sense IR from LED and ambient
	#define SENSOR_PL_CMD2_IR_LED		(0x80)		// sense IR from LED only
	#define SENSOR_PL_CMD2_MOD_DC		(0x00)		// IR LED modulation = DC
	#define SENSOR_PL_CMD2_MOD_327K		(0x40)		// IR LED modulation = 327.7 kHz
	#define SENSOR_PL_CMD2_DRIVE_12M	(0x00 << 4)	// IR drive current = 12.5 mA
	#define SENSOR_PL_CMD2_DRIVE_25M	(0x01 << 4)	// IR drive current = 25 mA
	#define SENSOR_PL_CMD2_DRIVE_50M	(0x02 << 4)	// IR drive current = 50 mA
	#define SENSOR_PL_CMD2_DRIVE_100M	(0x03 << 4)	// IR drive current = 100 mA
	#define SENSOR_PL_CMD2_ADC_RES_16	(0x00 << 2)	// ADC resolution = 16 bits
	#define SENSOR_PL_CMD2_ADC_RES_12	(0x01 << 2)	// ADC resolution = 12 bits
	#define SENSOR_PL_CMD2_ADC_RES_8	(0x02 << 2)	// ADC resolution = 8 bits
	#define SENSOR_PL_CMD2_ADC_RES_4	(0x03 << 2)	// ADC resolution = 4 bits
	#define SENSOR_PL_CMD2_ALS_RNG_1	(0x00)		// ALS sensing = 1000 LUX
	#define SENSOR_PL_CMD2_ALS_RNG_4	(0x01)		// ALS sensing = 4000 LUX
	#define SENSOR_PL_CMD2_ALS_RNG_16	(0x02)		// ALS sensing = 16000 LUX
	#define SENSOR_PL_CMD2_ALS_RNG_64	(0x03)		// ALS sensing = 64000 LUX
#define SENSOR_PL_DATA_LSB			(0x02)		// Data
#define SENSOR_PL_DATA_MSB			(0x03)		// Data
#define SENSOR_PL_INT_LT_LSB			(0x04)		// Low interrupt threshold LSB
#define SENSOR_PL_INT_LT_MSB			(0x05)		// Low interrupt threshold MSB
#define SENSOR_PL_INT_HT_LSB			(0x06)		// High interrupt threshold LSB
#define SENSOR_PL_INT_HT_MSB			(0x07)		// High interrupt threshold MSB
#define SENSOR_PL_EXT_SYNC			(0x80)		// write address to restart ADC integration

struct sensor_pl_rw {	// see the datasheet
	unsigned char cmd1;
	unsigned char cmd2;
	unsigned char dl;
	unsigned char dm;
	unsigned char int_lt_lsb;
	unsigned char int_lt_msb;
	unsigned char int_ht_lsb;
	unsigned char int_ht_msb;
};

// Digital Light
#define SENSOR_DL_CMD				(0x00)		// command
	#define SENSOR_DL_CMD_ADC_EN		(0x80)		// enable ADC core
	#define SENSOR_DL_CMD_ADC_DIS		(0x00)		// disable ADC core
	#define SENSOR_DL_CMD_PD		(0x40)		// power down
	#define SENSOR_DL_CMD_EXT_SYNC		(0x20)		// external sync
	#define SENSOR_DL_CMD_MODE_D1		(0x00)		// ADC work mode = Diode 1, 16 bits
	#define SENSOR_DL_CMD_MODE_D2		(0x04)		// ADC work mode = Diode 2, 16 bits
	#define SENSOR_DL_CMD_MODE_DIFF		(0x08)		// ADC work mode = I1-I2, 15 bits
	#define SENSOR_DL_CMD_W16		(0x00)		// 2^16 cycles
	#define SENSOR_DL_CMD_W12		(0x01)		// 2^12 cycles
	#define SENSOR_DL_CMD_W8		(0x02)		// 2^8 cycles
	#define SENSOR_DL_CMD_W4		(0x03)		// 2^4 cycles
#define SENSOR_DL_CONT				(0x01)		// control
	#define SENSOR_DL_CONT_INT		(0x20)		// interrupt status
	#define SENSOR_DL_G1			(0x00)		// gain < 1000 LUX
	#define SENSOR_DL_G4			(0x04)		// gain < 4000 LUX
	#define SENSOR_DL_G16			(0x08)		// gain < 16000 LUX
	#define SENSOR_DL_G64			(0x0C)		// gain < 64000 LUX
	#define SENSOR_DL_IP1			(0x00)		// interrupt persistence = 1 cycle
	#define SENSOR_DL_IP4			(0x01)		// interrupt persistence = 4 cycle
	#define SENSOR_DL_IP8			(0x02)		// interrupt persistence = 8 cycle
	#define SENSOR_DL_IP16			(0x03)		// interrupt persistence = 16 cycle
#define SENSOR_DL_INT_THI			(0x02)		// 
#define SENSOR_DL_INT_TLO			(0x03)		// 
#define SENSOR_DL_SENSOR_LSB			(0x04)		// 
#define SENSOR_DL_SENSOR_MSB			(0x05)		// 
#define SENSOR_DL_TIMER_LSB			(0x06)		// 
#define SENSOR_DL_TIMER_MSB			(0x07)		// 
#define SENSOR_DL_EXT_SYNC			(0x80)		// 
#define SENSOR_DL_INT_CLR			(0x40)		// 

struct sensor_dl_rw {	// see the datasheet
	unsigned char cmd;
	unsigned char control;
	unsigned char int_thi;
	unsigned char int_tlo;
	unsigned int sensor_data;
};

// Temperature
#define SENSOR_TEMP_LOC_MSB			(0x00)		// Local temperature MSB
#define SENSOR_TEMP_ROFF_HIGH			(0x11)		// Remote offset high
	// 10-bit plus sign format
	#define SENSOR_TEMP_LOC_MSB_10B_SIGN	(0x80)		// Sign
	#define SENSOR_TEMP_LOC_MSB_10B_64	(0x40)
	#define SENSOR_TEMP_LOC_MSB_10B_32	(0x20)
	#define SENSOR_TEMP_LOC_MSB_10B_16	(0x10)
	#define SENSOR_TEMP_LOC_MSB_10B_8	(0x08)
	#define SENSOR_TEMP_LOC_MSB_10B_4	(0x04)
	#define SENSOR_TEMP_LOC_MSB_10B_2	(0x02)
	#define SENSOR_TEMP_LOC_MSB_10B_1	(0x01)
#define SENSOR_TEMP_LOC_LSB			(0x30)		// Local temperature LSB
#define SENSOR_TEMP_ROFF_LOW			(0x12)		// Remote offset low
	// 10-bit plus sign format
	#define SENSOR_TEMP_LOC_LSB_10B_P5	(0x80)
	#define SENSOR_TEMP_LOC_LSB_10B_P25	(0x40)
	#define SENSOR_TEMP_LOC_LSB_10B_P125	(0x20)
#define SENSOR_TEMP_REM_MSB			(0x01)		// Remote temperature MSB
	// 12-bit plus sign format
	#define SENSOR_TEMP_REM_MSB_12B_SIGN	(0x80)		// Sign
	#define SENSOR_TEMP_REM_MSB_12B_64	(0x40)
	#define SENSOR_TEMP_REM_MSB_12B_32	(0x20)
	#define SENSOR_TEMP_REM_MSB_12B_16	(0x10)
	#define SENSOR_TEMP_REM_MSB_12B_8	(0x08)
	#define SENSOR_TEMP_REM_MSB_12B_4	(0x04)
	#define SENSOR_TEMP_REM_MSB_12B_2	(0x02)
	#define SENSOR_TEMP_REM_MSB_12B_1	(0x01)
#define SENSOR_TEMP_REM_LSB			(0x10)		// Remote temperature LSB
	// 12-bit plus sign format with filter off
	#define SENSOR_TEMP_REM_LSB_12B_P5	(0x80)
	#define SENSOR_TEMP_REM_LSB_12B_P25	(0x40)
	#define SENSOR_TEMP_REM_LSB_12B_P125	(0x20)
	// 12-bit plus sign format with filter on
	#define SENSOR_TEMP_REM_LSB_12B_P0625	(0x10)
	#define SENSOR_TEMP_REM_LSB_12B_P03125	(0x04)
#define SENSOR_TEMP_UREM_MSB			(0x31)		// Remote unsigned temperature MSB
	// 13-bit usigned format
	#define SENSOR_TEMP_UREM_MSB_12B_128	(0x80)
	#define SENSOR_TEMP_UREM_MSB_12B_64	(0x40)
	#define SENSOR_TEMP_UREM_MSB_12B_32	(0x20)
	#define SENSOR_TEMP_UREM_MSB_12B_16	(0x10)
	#define SENSOR_TEMP_UREM_MSB_12B_8	(0x08)
	#define SENSOR_TEMP_UREM_MSB_12B_4	(0x04)
	#define SENSOR_TEMP_UREM_MSB_12B_2	(0x02)
	#define SENSOR_TEMP_UREM_MSB_12B_1	(0x01)
#define SENSOR_TEMP_UREM_LSB			(0x32)		// Remote unsigned temperature LSB
	// 13-bit usigned format with filter off
	#define SENSOR_TEMP_UREM_LSB_12B_P5	(0x80)
	#define SENSOR_TEMP_UREM_LSB_12B_P25	(0x40)
	#define SENSOR_TEMP_UREM_LSB_12B_P125	(0x20)
	// 13-bit usigned format with filter on
	#define SENSOR_TEMP_UREM_LSB_12B_P0625	(0x10)
	#define SENSOR_TEMP_UREM_LSB_12B_P03125	(0x04)
#define SENSOR_TEMP_CONF2			(0xBF)		// Diode configuration
	#define SENSOR_TEMP_CONF2_A0		(0x40)		// A0 pin function
	#define SENSOR_TEMP_CONF2_OS		(0x00)		// A0 pin function
	#define SENSOR_TEMP_CONF2_OS_ON		(0x20)		// OS fault mask on
	#define SENSOR_TEMP_CONF2_OS_OFF	(0x00)		// OS fault mask off
	#define SENSOR_TEMP_CONF2_TCRIT_ON	(0x10)		// TCRIT fault mask on
	#define SENSOR_TEMP_CONF2_TCRIT_OFF	(0x00)		// TCRIT fault mask off
	#define SENSOR_TEMP_CONF2_DMOD1		(0x00)		// Diode model 1
	#define SENSOR_TEMP_CONF2_DMOD2		(0x08)		// Diode model 1
	#define SENSOR_TEMP_CONF2_FILT_OFF	(0x00 << 1)	// Filter off
	#define SENSOR_TEMP_CONF2_FILT_ON	(0x03 << 1)	// Filter on
#define SENSOR_TEMP_CONF1_RD			(0x03)		// General configuration
#define SENSOR_TEMP_CONF1_WR			(0x09)		// General configuration
	#define SENSOR_TEMP_CONF1_RUN		(0x00)		// Active/Converting
	#define SENSOR_TEMP_CONF1_STOP		(0x40)		// Standby
	#define SENSOR_TEMP_R_TCRIT_MASK_OFF	(0x00)		// Remote TCRIT
	#define SENSOR_TEMP_R_TCRIT_MASK_ON	(0x10)		// Remote TCRIT
	#define SENSOR_TEMP_R_OS_MASK_OFF	(0x00)		// Remote OS
	#define SENSOR_TEMP_R_OS_MASK_ON	(0x08)		// Remote OS
	#define SENSOR_TEMP_L_TCRIT_MASK_OFF	(0x00)		// Local TCRIT
	#define SENSOR_TEMP_L_TCRIT_MASK_ON	(0x04)		// Local TCRIT
	#define SENSOR_TEMP_L_OS_MASK_OFF	(0x00)		// Local OS
	#define SENSOR_TEMP_L_OS_MASK_ON	(0x02)		// Local OS
#define SENSOR_TEMP_CONV_RD			(0x04)		// Conversion rate
#define SENSOR_TEMP_CONV_WR			(0x0A)		// Conversion rate
	#define SENSOR_TEMP_CONV_CONT		(0x00)		// Continuous
	#define SENSOR_TEMP_CONV_P364		(0x01)		// .364 seconds
	#define SENSOR_TEMP_CONV_1		(0x02)		// 1 second
	#define SENSOR_TEMP_CONV_2P5		(0x03)		// 2.5 seconds
#define SENSOR_TEMP_ONE_SHOT			(0x0F)		// Remote offset low
#define SENSOR_TEMP_STAT1			(0x02)		// Status 1
	#define SENSOR_TEMP_STAT1_BUSY		(0x80)		// Converting
	#define SENSOR_TEMP_STAT1_ROS		(0x10)		// Remote OS
	#define SENSOR_TEMP_STAT1_DFAULT	(0x04)		// Diode Fault
	#define SENSOR_TEMP_STAT1_RTCRIT	(0x02)		// Remote TCRIT
	#define SENSOR_TEMP_STAT1_LOC		(0x01)		// Local OS & TCRIT
#define SENSOR_TEMP_STAT2			(0x33)		// Status 2
	#define SENSOR_TEMP_STAT2_NR		(0x80)		// Not Ready - 30 ms power-up
	#define SENSOR_TEMP_STAT2_TT		(0x40)		// TruTherm Diode detected
#define SENSOR_TEMP_REM_OS_LIM_RD		(0x07)		// Remote OS limit
#define SENSOR_TEMP_REM_OS_LIM_WR		(0x0D)		// Remote OS limit
#define SENSOR_TEMP_LOC_OS_LIM			(0x20)		// Local OS limit
#define SENSOR_TEMP_REM_TCRIT_LIM		(0x19)		// Remote T_Crit limit
	#define SENSOR_TEMP_LIM_128		(0x80)
	#define SENSOR_TEMP_LIM_64		(0x40)
	#define SENSOR_TEMP_LIM_32		(0x20)
	#define SENSOR_TEMP_LIM_16		(0x10)
	#define SENSOR_TEMP_LIM_8		(0x08)
	#define SENSOR_TEMP_LIM_4		(0x04)
	#define SENSOR_TEMP_LIM_2		(0x02)
	#define SENSOR_TEMP_LIM_1		(0x01)
#define SENSOR_TEMP_HYSTERESIS			(0x21)		// Common hysteresis
	#define SENSOR_TEMP_HYS_16		(0x10)
	#define SENSOR_TEMP_HYS_8		(0x08)
	#define SENSOR_TEMP_HYS_4		(0x04)
	#define SENSOR_TEMP_HYS_2		(0x02)
	#define SENSOR_TEMP_HYS_1		(0x01)
#define SENSOR_TEMP_MAN_ID			(0xFE)		// Manufacture ID
	#define SENSOR_TEMP_MAN_ID_DATA		(0x01)		// Manufacture ID
#define SENSOR_TEMP_REV_ID			(0xFF)		// Revision ID
	#define SENSOR_TEMP_REV_ID_DATA		(0xB1)		// Revision ID

struct sensor_temp_rw {	// see the datasheet
	unsigned char address;
	unsigned char d1;
};

// accelerometer
// ADXL345
#define SENSOR_ACC_ID				(0x00)		// Device ID
	#define SENSOR_ACC_ID_DATA		(0xE5)		// Device ID
#define SENSOR_ACC_TT				(0x1D)		// Tap threshold (62.5 mg/LSB)
#define SENSOR_ACC_OFSX				(0x1E)		// X axis offset (15.6 mg/LSB)
#define SENSOR_ACC_OFSY				(0x1F)		// Y axis offset (15.6 mg/LSB)
#define SENSOR_ACC_OFSZ				(0x20)		// Z axis offset (15.6 mg/LSB)
#define SENSOR_ACC_DUR				(0x21)		// Tap duration (625 us/LSB)
#define SENSOR_ACC_LAT				(0x22)		// Tap latency (1.25 ms/LSB)
#define SENSOR_ACC_WIN				(0x23)		// Tap window (1.25 ms/LSB)
#define SENSOR_ACC_TAT				(0x24)		// Activity threshold (62.5 mg/LSB)
#define SENSOR_ACC_TINAT			(0x25)		// Inactivity threshold (62.5 mg/LSB)
#define SENSOR_ACC_TIM_INAT			(0x26)		// Inactivity time (1 s/LSB)
#define SENSOR_ACC_AT_CONTROL			(0x27)		// Activity/Inactivity control
	#define SENSOR_ACC_ATC_DC		(0x00)		// Active DC coupled
	#define SENSOR_ACC_ATC_AC		(0x80)		// Active AC coupled
	#define SENSOR_ACC_ATC_XE		(0x40)		// Active X enable
	#define SENSOR_ACC_YTC_XE		(0x20)		// Active X enable
	#define SENSOR_ACC_ZTC_XE		(0x10)		// Active X enable
	#define SENSOR_ACC_ITC_DC		(0x00)		// Inactive DC coupled
	#define SENSOR_ACC_ITC_AC		(0x08)		// Inactive AC coupled
	#define SENSOR_ACC_ITC_XE		(0x04)		// Inactive X enable
	#define SENSOR_ACC_ITC_YE		(0x02)		// Inactive Y enable
	#define SENSOR_ACC_ITC_ZE		(0x01)		// Inactive Z enable
#define SENSOR_ACC_T_FF				(0x28)		// Freefall Threshold (62.5 mg/LSB)
#define SENSOR_ACC_TIM_FF			(0x29)		// Freefall Time (5 ms/LSB)
#define SENSOR_ACC_TAP_AXES			(0x2A)		// Tap Axis control
	#define SENSOR_ACC_TA_SUPRESS		(0x08)		// Supress Double Tap
	#define SENSOR_ACC_TA_XYZE		(0x07)		// X,Y,Z Tap Enable
	#define SENSOR_ACC_TA_XE		(0x04)		// X Tap Enable
	#define SENSOR_ACC_TA_YE		(0x02)		// Y Tap Enable
	#define SENSOR_ACC_TA_ZE		(0x01)		// Z Tap Enable
#define SENSOR_ACC_TAP_STAT			(0x2B)		// Tap Status
	#define SENSOR_ACC_TS_XA		(0x40)		// X Activity
	#define SENSOR_ACC_TS_YA		(0x20)		// Y Activity
	#define SENSOR_ACC_TS_ZA		(0x10)		// Z Activity
	#define SENSOR_ACC_TS_XT		(0x04)		// X Tap
	#define SENSOR_ACC_TS_YT		(0x02)		// Y Tap
	#define SENSOR_ACC_TS_ZT		(0x01)		// Z Tap
#define SENSOR_ACC_RATE				(0x2C)		// Data Rate control
	#define SENSOR_ACC_RATE_LP		(0x10)		// Low Power Mode
	#define SENSOR_ACC_RC_3200_1600		(0x0F)		// _OUTPUT-DATA_BANDWIDTH
	#define SENSOR_ACC_RC_1600_800		(0x0E)
	#define SENSOR_ACC_RC_800_400		(0x0D)
	#define SENSOR_ACC_RC_400_200		(0x0C)
	#define SENSOR_ACC_RC_200_100		(0x0B)
	#define SENSOR_ACC_RC_100_50		(0x0A)
	#define SENSOR_ACC_RC_50_25		(0x09)
	#define SENSOR_ACC_RC_25_12P5		(0x08)
	#define SENSOR_ACC_RC_12P5_6P25		(0x07)
	#define SENSOR_ACC_RC_6P25_3P125	(0x06)
	#define SENSOR_ACC_RC_3P125_1P563	(0x05)
	#define SENSOR_ACC_RC_1P563_P782	(0x04)
	#define SENSOR_ACC_RC_P782_P39		(0x03)
	#define SENSOR_ACC_RC_P39_P195		(0x02)
	#define SENSOR_ACC_RC_P195_P098		(0x01)
	#define SENSOR_ACC_RC_P098_P048		(0x00)
#define SENSOR_ACC_POWER			(0x2D)		// Power control
	#define SENSOR_ACC_P_LINK		(0x20)		// Activity/Inactivity Link mode
	#define SENSOR_ACC_P_APM		(0x10)		// Auto Low Power
	#define SENSOR_ACC_P_SM			(0x00)		// Standby
	#define SENSOR_ACC_P_NORM		(0x08)		// Powered Up
	#define SENSOR_ACC_P_SLEEP_NORM		(0x00)		// Not Sleep
	#define SENSOR_ACC_P_SLEEP		(0x04)		// Sleep
	#define SENSOR_ACC_P_W8			(0x00)		// wakeup Frequency = 8 Hz
	#define SENSOR_ACC_P_W4			(0x01)		// wakeup Frequency = 4 Hz
	#define SENSOR_ACC_P_W2			(0x02)		// wakeup Frequency = 2 Hz
	#define SENSOR_ACC_P_W1			(0x03)		// wakeup Frequency = 1 Hz
#define SENSOR_ACC_IE				(0x2E)		// Interrupt Enable
#define SENSOR_ACC_IM				(0x2F)		// Interrupt Map
#define SENSOR_ACC_IS				(0x30)		// Interrupt Source
	#define SENSOR_ACC_I_DR			(0x80)		// Data Ready
	#define SENSOR_ACC_I_ST			(0x40)		// Single Tap
	#define SENSOR_ACC_I_DT			(0x20)		// Double Tap
	#define SENSOR_ACC_I_A			(0x10)		// Activity
	#define SENSOR_ACC_I_I			(0x08)		// Inactivity
	#define SENSOR_ACC_I_FF			(0x04)		// Freefall
	#define SENSOR_ACC_I_WM			(0x02)		// Watermark
	#define SENSOR_ACC_I_OR			(0x01)		// Overrun
#define SENSOR_ACC_DF				(0x31)		// Data Format
	#define SENSOR_ACC_DF_SELF_TEST		(0x80)		// Self Test
	#define SENSOR_ACC_DF_SPI_MODE4		(0x00)		// SPI 4-Wire
	#define SENSOR_ACC_DF_SPI_MODE3		(0x40)		// SPI 3-Wire
	#define SENSOR_ACC_DF_INT_INVERT	(0x20)		// Interrupt Active Low
	#define SENSOR_ACC_DF_LENGTH		(0x08)		// 13-bit, 16g Enable
	#define SENSOR_ACC_DF_POS		(0x04)		// MSB Left Justified
	#define SENSOR_ACC_DF_R2		(0x00)		// 2g
	#define SENSOR_ACC_DF_R4		(0x01)		// 4g
	#define SENSOR_ACC_DF_R8		(0x02)		// 8g
	#define SENSOR_ACC_DF_R16		(0x03)		// 16g
#define SENSOR_ACC_DX0				(0x32)		// Data X axis 0 (LSB)
#define SENSOR_ACC_DX1				(0x33)		// Data X axis 1 (MSB)
#define SENSOR_ACC_DY0				(0x34)		// Data Y axis 0 (LSB)
#define SENSOR_ACC_DY1				(0x35)		// Data Y axis 1 (MSB)
#define SENSOR_ACC_DZ0				(0x36)		// Data Z axis 0 (LSB)
#define SENSOR_ACC_DZ1				(0x37)		// Data Z axis 1 (MSB)
#define SENSOR_ACC_FC				(0x38)		// FIFO Control
	#define SENSOR_ACC_FC_BYP		(0x00 << 6)	// Bypass
	#define SENSOR_ACC_FC_HOLD		(0x01 << 6)	// Hold after 32
	#define SENSOR_ACC_FC_OF		(0x02 << 6)	// Discard after 32
	#define SENSOR_ACC_FC_TRIG		(0x03 << 6)	// Hold on TRIGGER
	#define SENSOR_ACC_FC_TRIG1		(0x00)		// TRIGGER = INT1
	#define SENSOR_ACC_FC_TRIG2		(0x20)		// TRIGGER = INT2
	#define SENSOR_ACC_FC_SAMP(x)		(x)		// See ADXL345 datasheet
#define SENSOR_ACC_FS				(0x39)		// FIFO Status
	#define SENSOR_ACC_FS_TRIG		(0x80)		// TRIGGER occurred
	#define SENSOR_ACC_FS_ENTRIES_MSK	(0x1F)		// See ADXL345 datasheet

// ST LIS302DL
#define SENSOR_A3_WAI				(0x0F)		// Device ID
	#define SENSOR_A3_WAI_ID		(0x3B)		// Device ID
#define SENSOR_A3_CTRL1				(0x20)		// Control Register
	#define SENSOR_A3_CTRL1_DR100		(0x00)		// sample data rate = 100 Hz
	#define SENSOR_A3_CTRL1_DR400		(0x80)		// sample data rate = 400 Hz
	#define SENSOR_A3_CTRL1_PD		(0x00)		// power down
	#define SENSOR_A3_CTRL1_PU		(0x40)		// power up
	#define SENSOR_A3_CTRL1_FS		(0x20)		// See data sheet
	#define SENSOR_A3_CTRL1_STP		(0x10)		// See data sheet
	#define SENSOR_A3_CTRL1_STM		(0x08)		// See data sheet
	#define SENSOR_A3_CTRL1_XYZEN		(0x07)		// X,Y,Z axis enable
	#define SENSOR_A3_CTRL1_ZEN		(0x04)		// Z axis enable
	#define SENSOR_A3_CTRL1_YEN		(0x02)		// Y axis enable
	#define SENSOR_A3_CTRL1_XEN		(0x01)		// X axis enable
#define SENSOR_A3_CTRL2				(0x21)		// Control Register
	#define SENSOR_A3_CTRL2_SIM		(0x80)		// SPI mode
	#define SENSOR_A3_CTRL2_BOOT		(0x40)		// copy calibration from FLASH
	#define SENSOR_A3_CTRL2_FILT_OFF	(0x00)		// internal filter bypassed
	#define SENSOR_A3_CTRL2_FILT_ON		(0x10)		// internal filter enabled
	#define SENSOR_A3_CTRL2_F2		(0x08)		// WU2 filter enable
	#define SENSOR_A3_CTRL2_F1		(0x04)		// WU1 filter enable
	#define SENSOR_A3_CTRL2_COEFF(x)	(x & 0x3)	// See data sheet
#define SENSOR_A3_CTRL3				(0x22)		// Control Register
	#define SENSOR_A3_CTRL3_IH		(0x00)		// Interrupt active high
	#define SENSOR_A3_CTRL3_IL		(0x80)		// Interrupt active low
	#define SENSOR_A3_CTRL3_IPP		(0x00)		// Interrupt push/pull
	#define SENSOR_A3_CTRL3_IOD		(0x40)		// Interrupt open drain
	#define SENSOR_A3_CTRL3_I2C(x)		((x&0x7) << 3)	// I2 config - See data sheet
	#define SENSOR_A3_CTRL3_I1C(x)		((x&0x7))	// I1 config - See data sheet
#define SENSOR_A3_HPF_RST			(0x23)		// High Pass Filter Reset - See data sheet
#define SENSOR_A3_STAT				(0x27)		// Status
	#define SENSOR_A3_STAT_ZYXOR		(0x80)		// ZYX overrun
	#define SENSOR_A3_STAT_ZOR		(0x40)		// Z overrun
	#define SENSOR_A3_STAT_YOR		(0x20)		// Y overrun
	#define SENSOR_A3_STAT_XOR		(0x10)		// X overrun
	#define SENSOR_A3_STAT_ZYXDA		(0x08)		// ZYX data available
	#define SENSOR_A3_STAT_ZDA		(0x04)		// Z data available
	#define SENSOR_A3_STAT_YDA		(0x02)		// Y data available
	#define SENSOR_A3_STAT_XDA		(0x01)		// X data available
#define SENSOR_A3_OUTX				(0x29)		// X Output
#define SENSOR_A3_OUTY				(0x2B)		// Y Output
#define SENSOR_A3_OUTZ				(0x2D)		// Z Output
#define SENSOR_A3_CFG1				(0x30)		// Configuration
#define SENSOR_A3_CFG2				(0x34)		// Configuration
	#define SENSOR_A3_CFG_AOI		(0x80)		// AND/OR interrupts
	#define SENSOR_A3_CFG_LIR		(0x40)		// latch interrupts into SRC
	#define SENSOR_A3_CFG_ZHIE		(0x20)		// Z high enable
	#define SENSOR_A3_CFG_ZLIE		(0x10)		// Z low enable
	#define SENSOR_A3_CFG_YHIE		(0x08)		// Y high enable
	#define SENSOR_A3_CFG_YLIE		(0x04)		// Y low enable
	#define SENSOR_A3_CFG_XHIE		(0x02)		// X high enable
	#define SENSOR_A3_CFG_XLIE		(0x01)		// X low enable
#define SENSOR_A3_SRC1				(0x31)		// Source
#define SENSOR_A3_SRC2				(0x35)		// Source
	#define SENSOR_A3_SRC_IA		(0x40)		// interrupt active
	#define SENSOR_A3_SRC_ZH		(0x20)		// Z high
	#define SENSOR_A3_SRC_ZL		(0x10)		// Z low
	#define SENSOR_A3_SRC_YH		(0x08)		// Y high
	#define SENSOR_A3_SRC_YL		(0x04)		// Y low
	#define SENSOR_A3_SRC_XH		(0x02)		// X high
	#define SENSOR_A3_SRC_XL		(0x01)		// X low
#define SENSOR_A3_THS1				(0x32)		// Threshold
#define SENSOR_A3_THS2				(0x36)		// Threshold
	#define SENSOR_A3_THS_DCRM		(0x80)		// Resetting mode - See data sheet
	#define SENSOR_A3_THS_THS(x)		(x & 0x7F)	// FF/wakeup threshold
#define SENSOR_A3_DUR1				(0x33)		// Duration - See data sheet
#define SENSOR_A3_DUR2				(0x37)		// Duration - See data sheet
#define SENSOR_A3_CCFG				(0x38)		// Click Configuration
#define SENSOR_A3_CSRC				(0x39)		// Click Source
	#define SENSOR_A3_CCS_LIR		(0x40)		// latch interrupt into SRC
	#define SENSOR_A3_CCS_DZ		(0x20)		// double Z enable
	#define SENSOR_A3_CCS_SZ		(0x10)		// single Z enable
	#define SENSOR_A3_CCS_DY		(0x08)		// double Y enable
	#define SENSOR_A3_CCS_SY		(0x04)		// single Y enable
	#define SENSOR_A3_CCS_DX		(0x02)		// double X enable
	#define SENSOR_A3_CCS_SX		(0x01)		// single X enable
#define SENSOR_A3_CTHXY				(0x3B)		// Click X, Y Threshold
	#define SENSOR_A3_CTHYX_Y(x)		((x&0xF) << 4)	// Y Threshold
	#define SENSOR_A3_CTHYX_X(x)		(x&0xF)		// X Threshold
#define SENSOR_A3_CTHZ				(0x3C)		// Click Z Threshold
	#define SENSOR_A3_CTHYX_Z(x)		(x&0xF)		// Z Threshold
#define SENSOR_A3_CTL				(0x3D)		// Click Time Limit
#define SENSOR_A3_CLAT				(0x3E)		// Click Latency
#define SENSOR_A3_CWIN				(0x3F)		// Click Window

// count always = 1 for LIS302DL
struct sensor_acc_rw {	// see the datasheets
	unsigned char address;
	unsigned int count;	// number of bytes to read (1 or 2) 
	unsigned char data[2];
};

// digital compass
#define SENSOR_DCOMP_ST				(0xC0)		// Status (RO)
	#define SENSOR_DCOMP_ST_INT		(0x01)		// Interrupt
	#define SENSOR_DCOMP_ST_EERW		(0x02)		// EEPROM R/W
#define SENSOR_DCOMP_TMPS			(0xC1)		// Temperature(C) = 35+(120-TMPS)/1.6
#define SENSOR_DCOMP_H1X			(0xC2)		// X Heading
#define SENSOR_DCOMP_H1Y			(0xC3)		// Y Heading
#define SENSOR_DCOMP_H1Z			(0xC4)		// Z Heading
#define SENSOR_DCOMP_MS1			(0xE0)		// Mode
	#define SENSOR_DCOMP_MS1_SENSOR		(0x0)		// sensor mode
	#define SENSOR_DCOMP_MS1_EEPROM		(0x2)		// EEPROM R/W
	#define SENSOR_DCOMP_MS1_PD		(0x3)		// power down
	#define SENSOR_DCOMP_MS1_EEWEN		(0xA8)		// EEPROM Write Enable
#define SENSOR_DCOMP_HXDA			(0xE1)		// X DAC offset - see table 3 in datasheet
#define SENSOR_DCOMP_HYDA			(0xE2)		// Y DAC offset - see table 3 in datasheet
#define SENSOR_DCOMP_HZDA			(0xE3)		// Z DAC offset - see table 3 in datasheet
#define SENSOR_DCOMP_HXGA			(0xE4)		// X gain - see table 4 in datasheet
#define SENSOR_DCOMP_HYGA			(0xE5)		// Y gain - see table 4 in datasheet
#define SENSOR_DCOMP_HZGA			(0xE6)		// Z gain - see table 4 in datasheet
#define SENSOR_DCOMP_TS1			(0x5D)		// FACTORY TEST - DO NOT USE
#define SENSOR_DCOMP_EE_WRAL1			(0x60)		// EE - batch write adress
#define SENSOR_DCOMP_EE_ETS			(0x62)		// EE - temperature offset
#define SENSOR_DCOMP_EE_EVIR			(0x63)		// EE - VREF/IREF adjustment
#define SENSOR_DCOMP_EE_EIHE			(0x64)		// EE - HE drive/OSC
#define SENSOR_DCOMP_EE_ETST			(0x65)		// EE - test
#define SENSOR_DCOMP_EE_EHXGA			(0x66)		// EE - X gain adjustment
#define SENSOR_DCOMP_EE_EHYGA			(0x67)		// EE - Y gain adjustment
#define SENSOR_DCOMP_EE_EHZGA			(0x68)		// EE - Z gain adjustment

// generic address/data byte R/W
struct sensor_rw {
	unsigned char address;
	unsigned char data;
};

// Sensor driver ioctl definitions
#define BMI_SENSOR_ON	(1)
#define BMI_SENSOR_OFF	(0)
#define BMI_SENSOR_RLEDOFF	_IOW(BMI_SENSOR_IOCTL, 0x1, unsigned int)		// Turn off red LED
#define BMI_SENSOR_RLEDON	_IOW(BMI_SENSOR_IOCTL, 0x2, unsigned int)		// Turn on red LED
#define BMI_SENSOR_GLEDOFF	_IOW(BMI_SENSOR_IOCTL, 0x3, unsigned int)		// Turn off green LED
#define BMI_SENSOR_GLEDON	_IOW(BMI_SENSOR_IOCTL, 0x4, unsigned int)		// Turn on green LED
#define BMI_SENSOR_GETSTAT	_IOR(BMI_SENSOR_IOCTL, 0x5, unsigned int *)		// Read IOX/GPIO (== GPIO<<16 | IOX1<<8 | IOX0)
#define BMI_SENSOR_ADCWR	_IOW(BMI_SENSOR_IOCTL, 0x6, unsigned int)		// write ADC
#define BMI_SENSOR_ADCRD	_IOR(BMI_SENSOR_IOCTL, 0x7, unsigned int *)		// read ADC
#define BMI_SENSOR_HUMRD	_IOR(BMI_SENSOR_IOCTL, 0x8, unsigned int *)		// read ADC - Humidity sensor
#define BMI_SENSOR_ACOMPRST	_IO(BMI_SENSOR_IOCTL, 0x9)				// analog Compass reset (toggle off/on)
#define BMI_SENSOR_ACOMPXRD	_IOR(BMI_SENSOR_IOCTL, 0xa, unsigned int *)		// read ADC - Compass X axis
#define BMI_SENSOR_ACOMPYRD	_IOR(BMI_SENSOR_IOCTL, 0xb, unsigned int *)		// read ADC - Compass Y axis
#define BMI_SENSOR_ACOMPZRD	_IOR(BMI_SENSOR_IOCTL, 0xc, unsigned int *)		// read ADC - Compass Z axis
#define BMI_SENSOR_PLWR		_IOW(BMI_SENSOR_IOCTL, 0xd, struct sensor_pl_rw *)	// write Proximity/Light sensor
#define BMI_SENSOR_PLRD		_IOR(BMI_SENSOR_IOCTL, 0xe, struct sensor_pl_rw *)	// read Proximity/Light sensor
#define BMI_SENSOR_PL_SYNC	_IO(BMI_SENSOR_IOCTL, 0xf)				// generate external SYNC for Proximity/Light or Digital Light
#define BMI_SENSOR_PL_IWAIT	_IOR(BMI_SENSOR_IOCTL, 0x10, struct sensor_pl_rw *)	// wait for Proximity/Light interrupt - application sets up INT configuration
#define BMI_SENSOR_SNDARD	_IOR(BMI_SENSOR_IOCTL, 0x11, unsigned int *)		// read ADC - Sound Average level
#define BMI_SENSOR_SNDPRD	_IOR(BMI_SENSOR_IOCTL, 0x12, unsigned int *)		// read ADC - Sound Peak level read/clear
#define BMI_SENSOR_SNDIRD	_IOR(BMI_SENSOR_IOCTL, 0x13, unsigned int *)		// read ADC - Instantaneous level read/clear
#define BMI_SENSOR_TEMPWR	_IOW(BMI_SENSOR_IOCTL, 0x14, struct sensor_temp_rw *)	// write Temperature sensor
#define BMI_SENSOR_TEMPRD	_IOR(BMI_SENSOR_IOCTL, 0x15, struct sensor_temp_rw *)	// read Temperature sensor
#define BMI_SENSOR_TEMPRD_SL	_IOR(BMI_SENSOR_IOCTL, 0x16, unsigned int *)		// Read signed local
#define BMI_SENSOR_TEMPRD_SR	_IOR(BMI_SENSOR_IOCTL, 0x17, unsigned int *)		// Read signed remote
#define BMI_SENSOR_TEMPRD_UR	_IOR(BMI_SENSOR_IOCTL, 0x18, unsigned int *)		// Read unsigned remote
#define BMI_SENSOR_TEMP_IWAIT	_IO(BMI_SENSOR_IOCTL, 0x19)				// wait for Temperature interrupt - application sets up INT configuration
#define BMI_SENSOR_MOTRD	_IOR(BMI_SENSOR_IOCTL, 0x1a, unsigned int *)		// read real-time Motion state
#define BMI_SENSOR_MOT_IWAIT	_IOR(BMI_SENSOR_IOCTL, 0x1b, unsigned int *)		// wait for Motion interrupt
#define BMI_SENSOR_ACCWR	_IOW(BMI_SENSOR_IOCTL, 0x1c, struct sensor_acc_rw *)	// write Accelerometer
#define BMI_SENSOR_ACCRD	_IOR(BMI_SENSOR_IOCTL, 0x1d, struct sensor_acc_rw *)	// read Accelerometer
#define BMI_SENSOR_ACCXRD	_IOR(BMI_SENSOR_IOCTL, 0x1e, unsigned int *)		// read Accelerometer X
#define BMI_SENSOR_ACCYRD	_IOR(BMI_SENSOR_IOCTL, 0x1f, unsigned int *)		// read Accelerometer Y
#define BMI_SENSOR_ACCZRD	_IOR(BMI_SENSOR_IOCTL, 0x20, unsigned int *)		// read Accelerometer Z
#define BMI_SENSOR_ACC_I1WAIT	_IO(BMI_SENSOR_IOCTL, 0x21)				// wait for Accelerometer interrupt 1 - application sets up INT configuration
#define BMI_SENSOR_ACC_I2WAIT	_IO(BMI_SENSOR_IOCTL, 0x22)				// wait for Accelerometer interrupt 2 - application sets up INT configuration
#define BMI_SENSOR_EEWR		_IOW(BMI_SENSOR_IOCTL, 0x23, struct sensor_rw *)	// write EEPROM
#define BMI_SENSOR_EERD		_IOR(BMI_SENSOR_IOCTL, 0x24, struct sensor_rw *)	// read EEPROM
#define BMI_SENSOR_MOT_IE	_IOW(BMI_SENSOR_IOCTL, 0x25, unsigned int)		// Motion interrupt enable (on = BMI_SENSOR_ON)
#define BMI_SENSOR_USB_IWAIT	_IO(BMI_SENSOR_IOCTL, 0x26)				// wait for USB power flag interrupt
#define BMI_SENSOR_USB_PWR_EN	_IOW(BMI_SENSOR_IOCTL, 0x27, unsigned int)		// USB power enable (on = BMI_SENSOR_ON)
#define BMI_SENSOR_HUM_PWR_EN	_IOW(BMI_SENSOR_IOCTL, 0x28, unsigned int)		// Humidity power enable (on = BMI_SENSOR_ON)
#define BMI_SENSOR_DCOM_RST	_IOW(BMI_SENSOR_IOCTL, 0x29, unsigned int)		// Digital Compass Reset (on = BMI_SENSOR_ON)
#define BMI_SENSOR_COM_GCAL	_IOR(BMI_SENSOR_IOCTL, 0x2a, struct sensor_comp_cal *)	// Get compass calibation
#define BMI_SENSOR_COM_SCAL	_IOW(BMI_SENSOR_IOCTL, 0x2b, struct sensor_comp_cal *)	// Set compass calibation
#define BMI_SENSOR_DCWR		_IOW(BMI_SENSOR_IOCTL, 0x2c, struct sensor_rw *)	// write digital compass
#define BMI_SENSOR_DCRD		_IOR(BMI_SENSOR_IOCTL, 0x2d, struct sensor_rw *)	// read digital compass
#define BMI_SENSOR_DC_GDAC	_IOR(BMI_SENSOR_IOCTL, 0x2e, struct sensor_comp_dac *)	// Get digital compass DAC settings
#define BMI_SENSOR_DC_SDAC	_IOW(BMI_SENSOR_IOCTL, 0x2f, struct sensor_comp_dac *)	// Set digital compass DAC settings
#define BMI_SENSOR_DC_IWAIT	_IO(BMI_SENSOR_IOCTL, 0x30)				// wait for digital compass interrupt - application sets up INT configuration
#define BMI_SENSOR_APROX_DUR	_IOW(BMI_SENSOR_IOCTL, 0x31, unsigned int)		// Analog Proximity LED burst time (in ms 2 <= arg <= 100)
#define BMI_SENSOR_APROXRD	_IOR(BMI_SENSOR_IOCTL, 0x32, unsigned int *)		// read Analog proximity = (PDOUT << 16) | ADC_DATA
#define BMI_SENSOR_ALIGHTRD	_IOR(BMI_SENSOR_IOCTL, 0x33, unsigned int *)		// read Analog Light
#define BMI_SENSOR_DLIGHTWR	_IOW(BMI_SENSOR_IOCTL, 0x34, struct sensor_dl_rw *)	// write Digital Light sensor
#define BMI_SENSOR_DLIGHTRD	_IOR(BMI_SENSOR_IOCTL, 0x35, unsigned int)		// read Digital Light sensor
#define BMI_SENSOR_DLIGHT_IC	_IO(BMI_SENSOR_IOCTL, 0x36)				// Digital Light interrupt clear
#define BMI_SENSOR_DLIGHT_IWAIT	_IOR(BMI_SENSOR_IOCTL, 0x37, struct sensor_dl_rw *)	// wait for Digital Light interrupt - application sets up INT configuration
#define BMI_SENSOR_MIC_EN	_IOW(BMI_SENSOR_IOCTL, 0x38, unsigned int)		// Sound power enable (on = BMI_SENSOR_ON)
#define BMI_SENSOR_SUSPEND	_IOW(BMI_SENSOR_IOCTL, 0x39, unsigned int)
#define BMI_SENSOR_RESUME 	_IOW(BMI_SENSOR_IOCTL, 0x40, unsigned int)

#endif	/* BMI_SENSOR_H */

