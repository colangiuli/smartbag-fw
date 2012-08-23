#include "LIS3DH.h"
#include "Arduino.h"
//#include "Wire.h"
#include "I2C.h"

LIS3DH::LIS3DH(){
	i2cAddress=0x18;
	}


void LIS3DH::begin(){
	
		//Wire.begin();
		I2C_init();
        //config(0x2A, 0x00, 0x02, 0x09, 0x88);
        quiet_detection_mode();
}

void LIS3DH::config(byte int_cfg, byte int_duration, byte int_ths, byte filter_config, byte scale_config){
    
    writeReg(LR_CTRL_REG1,0x37); // x,y,z enabled, 25Hz
    //writeReg(LR_CTRL_REG1,0x67); //x,y,z enabled, 200Hz
    writeReg(LR_CTRL_REG2,filter_config); // High-pass filter enabled
    //writeReg(LR_CTRL_REG2,0x39); // High-pass filter enabled cutoff 11
    writeReg(LR_CTRL_REG3,0x40); // Interrupt driven to INT1 pad
    writeReg(LR_CTRL_REG4,scale_config); // High res enabled, block data update, 2g full scale
    writeReg(LR_CTRL_REG5,0x8); // Interrupt latched
    writeReg(LR_INT1_THS,int_ths); //// Threshold = 10 mg
    writeReg(LR_INT1_DURATION,int_duration); // Duration = 0
    writeReg(LR_INT1_CFG,int_cfg); // Enable XH and YH interrupt generation
}


void LIS3DH::motion_mode(){
   config(INT_CFG_XH_YH_ZH, 0x04, 0x2, HP_FILTER_ON_DATA_FILTERED, RANGE_2G); 
}

void LIS3DH::quiet_detection_mode(){
   config(INT_CFG_XL_YL_ZL, 0x60, 0x2, HP_FILTER_ON_DATA_FILTERED, RANGE_2G); 
}

void LIS3DH::impact_mode(){
   config(INT_CFG_XH_YH_ZH, INT_DURATION_0_MS, 0x03, HP_FILTER_ON_DATA_FILTERED, RANGE_16G); 
}


bool LIS3DH::readReg(byte addr, byte *val){
	I2C_read(i2cAddress,addr,(uint8_t)(1),val);
	return true;
/*	
  Wire.beginTransmission(i2cAddress);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(i2cAddress,1);
  int timeouts=0;
  while(!Wire.available() && timeouts++<=5){
    delay(10);
  }
  if (Wire.available()){
		*val=Wire.read();
		return true;
  }else{
	Serial.println("FAIL");
    return false;
  }
*/

}
bool LIS3DH::writeReg(byte addr, byte val){
	I2C_write(i2cAddress,addr, (uint8_t) val);
/*	
  Wire.beginTransmission(i2cAddress);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();

*/

  return true;

}

  bool LIS3DH::getBit(byte b,byte bit){
	b<<=(bit);
	if (b>=127){
		return true;
	}
	return false;
  }

bool LIS3DH::getZValue(int16_t *val){
	byte high;
	byte low;
	if (!readReg(LR_OUT_Z_L, &low)){
		return false;
	}
	if (!readReg(LR_OUT_Z_H, &high)){
		return false;
	}
	*val=(low|(high << 8));
	return true;
}

bool LIS3DH::getYValue(int16_t *val){
	byte high;
	byte low;
	if (!readReg(LR_OUT_Y_L, &low)){
		return false;
	}
	if (!readReg(LR_OUT_Y_H, &high)){
		return false;
	}
	*val=(low|(high << 8));
	return true;
}

bool LIS3DH::getXValue(int16_t *val){
	byte high;
	byte low;
	if (!readReg(LR_OUT_X_L, &low)){
		return false;
	}
	if (!readReg(LR_OUT_X_H, &high)){
		return false;
	}
	*val=(low|(high << 8));
	return true;
}

bool LIS3DH::getInterruptSource(int8_t *val){
	if (!readReg(LR_INT1_SOURCE, (byte *)(&val))){
		return false;
	}
	return true;
}
	
	


