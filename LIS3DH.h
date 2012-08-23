#ifndef LIS3DH_h
#define LIS3DH_h

#include "Arduino.h"
//#include "Wire.h"

#define LR_MAX_TRIES 12
//specifici lis3dh
#define LR_STATUS_REG_AUX	0x07
#define LR_OUT_ADC1_L	0x08
#define LR_OUT_ADC1_H	0x09
#define LR_OUT_ADC2_L	0x0a
#define LR_OUT_ADC2_H	0x0b
#define LR_OUT_ADC3_L	0x0c
#define LR_OUT_ADC3_H	0x0d
#define LR_INT_COUNTER_REG	0x0e
#define LR_WHO_AM_I		0x0f
#define LR_TEMP_CFG_REG	0x1f
//fine specifici
#define LR_CTRL_REG1 0x20
#define LR_CTRL_REG2 0x21
#define LR_CTRL_REG3 0x22
#define LR_CTRL_REG4 0x23
#define LR_CTRL_REG5 0x24
#define LR_CTRL_REG6 0x25 //diverso, era l'hp filter
#define LR_REFERENCE 0x26
#define LR_STATUS_REG2 0x27
#define LR_OUT_X_L 0x28
#define LR_OUT_X_H 0x29
#define LR_OUT_Y_L 0x2A
#define LR_OUT_Y_H 0x2B
#define LR_OUT_Z_L 0x2C
#define LR_OUT_Z_H 0x2D
//specifici lis3dh
#define LR_FIFO_CTRL_REG	0x2e
#define LR_FIFO_SRC_REG	0x2f
//fine specifici lis3dh
#define LR_INT1_CFG 0x30
#define LR_INT1_SOURCE 0x31
#define LR_INT1_THS 0x32
#define LR_INT1_DURATION 0x33

//specifici lis3dh
#define LR_CLICK_CFG	0x38
#define LR_CLICK_SRC	0x39
#define LR_CLICK_THS	0x3a
#define LR_TIME_LIMIT	0x3b
#define LR_TIME_LATENCY	0x3c
#define LR_TIME_WINDOW	0x3d
#define LR_FIFO_WTM_MASK	0x80
#define LR_FIFO_OVRN_MASK	0x40
#define LR_MULTI_OUT_X_L	0xa8
#define LR_DEV_ID		0x33
#define LR_OUT_MIN_VALUE	(-32768)
#define LR_OUT_MAX_VALUE	(32767)
//fine specifici lis3dh

// Power Modes

#define LR_POWER_OFF B00000000
#define LR_POWER_NORM B00100000 
#define LR_POWER_LOW1 B01000000 
#define LR_POWER_LOW2 B01100000 
#define LR_POWER_LOW3 B10000000 

// Data Rates

#define LR_DATA_RATE_50 B00000000
#define LR_DATA_RATE_100 B00001000
#define LR_DATA_RATE_400 B00010000
#define LR_DATA_RATE_1000 B00011000


// Enable and disable channel.

#define LR_Z_ENABLE B00000100
#define LR_Z_DISABLE B00000000
#define LR_Y_ENABLE B00000010
#define LR_Y_DISABLE B00000000
#define LR_X_ENABLE B00000001
#define LR_X_DISABLE B00000000




//LR_INT1_CFG MODES
#define INT_CFG_XH          0x02
#define INT_CFG_YH          0x08
#define INT_CFG_ZH          0x20
#define INT_CFG_XH_YH       0x0A
#define INT_CFG_XH_ZH       0x22
#define INT_CFG_YH_ZH       0x28
#define INT_CFG_XH_YH_ZH    0x2A

#define INT_CFG_XL_YL_ZL    0x15

//LR_INT1_duration MODES (FOR 25 Hz)
//Duration LSB value for 25Hz = 40ms
#define INT_DURATION_0_MS     0x00
#define INT_DURATION_40_MS    0x01
#define INT_DURATION_160_MS   0x02
#define INT_DURATION_200_MS   0x03
//....

//LR_INT1_THS MODES
//Threshold LSB value for 2G full scale ±= 16mg 
#define INT_THS_2G_16mg      0x01
#define INT_THS_2G_32mg      0x02
#define INT_THS_2G_48mg      0x03
//....
//Threshold LSB value for 4G full scale ±= 31mg 
//....
//Threshold LSB value for 8G full scale ±= 63mg 
//....
//Threshold LSB value for 16G full scale ±= 125mg 
#define INT_THS_16G_125mg      0x01
//....

//filter_config (ctrl reg 2)
#define HP_FILTER_OFF               0x00
#define HP_FILTER_ON                0x01
#define HP_FILTER_ON_DATA_FILTERED  0x09

//scale_config (ctrl reg 4)
//IMPLIVIT SETTINGS: HIGH RESOLUTION MODE, BLOCK DATA UPDATE, Data LSB @ lower address, SPI DISABLED (I2C MODE)
#define RANGE_2G  0x88
#define RANGE_4G  0x98
#define RANGE_8G  0xA8
#define RANGE_16G 0xB8


class LIS3DH
{
	bool writeReg(byte addr, byte val);
	bool readReg(byte addr, byte *val);
	bool getBit(byte b, byte bit);
	public:
		LIS3DH();
		int i2cAddress;
		void begin();
        void config(byte int_cfg, byte int_duration, byte int_ths, byte filter_config, byte scale_config);
        void motion_mode();
        void quiet_detection_mode();
        void impact_mode();
		bool getXValue(int16_t *val);
		bool getZValue(int16_t *val);
		bool getYValue(int16_t *val);
		bool getInterruptSource(int8_t *val);
};

#endif
