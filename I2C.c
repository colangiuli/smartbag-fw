//#include <inttypes.h>
#include "I2C.h"
#include "arduino.h"
//#include "timing.h"


uint8_t bytesAvailable = 0;
uint8_t bufferIndex = 0;
uint8_t totalBytes = 0;
uint16_t timeOutDelay = 50;
uint8_t returnStatus;
uint8_t nack;
//uint8_t data[MAX_BUFFER_SIZE];

////////////// Public Methods ////////////////////////////////////////


void I2C_init()
{

    // activate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    //sbi(PORTD, 0);
    //sbi(PORTD, 1);
    sbi(PORTC, 4);
    sbi(PORTC, 5);

  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / 100000) - 16) / 2;
  // enable twi module and acks
  TWCR = _BV(TWEN) | _BV(TWEA); 
}

void I2C_disable()
{
  TWCR = 0;
}

void I2C_timeOut(uint16_t _timeOut)
{
  timeOutDelay = _timeOut;
}

void I2C_setSpeed(uint8_t fast)
{
  if(!fast)
  {
    TWBR = ((F_CPU / 100000) - 16) / 2;
  }
  else
  {
    TWBR = ((F_CPU / 400000) - 16) / 2;
  }
}
  
void I2C_pullup(uint8_t activate)
{
  if(activate)
  {
      // activate internal pull-ups for twi
      // as per note from atmega128 manual pg204
      //sbi(PORTD, 0);
      //sbi(PORTD, 1);
  }
  else
  {
      // deactivate internal pull-ups for twi
      // as per note from atmega128 manual pg204
      //cbi(PORTD, 0);
      //cbi(PORTD, 1);
  }
}

void I2C_scan()
{
  uint16_t tempTime = timeOutDelay;
  I2C_timeOut(80);
  uint8_t totalDevicesFound = 0;
  uint8_t s;
  
  //Serial.println("Scanning for devices...please wait");
  //Serial.println();
  for(s = 0; s <= 0x7F; s++)
  {
    returnStatus = 0;
    returnStatus = I2C_start();
    if(!returnStatus)
    { 
      returnStatus = I2C_sendAddress(SLA_W(s));
    }
    if(returnStatus)
    {
      if(returnStatus == 1)
      {
        //Serial.println("There is a problem with the bus, could not complete scan");
        timeOutDelay = tempTime;
        return;
      }
    }
    else
    {
      //Serial.print("Found device at address - ");
      //Serial.print(" 0x");
      //Serial.println(s,HEX);
      totalDevicesFound++;
    }
    I2C_stop();
  }
  //if(!totalDevicesFound){Serial.println("No devices found");}
  timeOutDelay = tempTime;
}

/*

uint8_t I2C_available()
{
  return(bytesAvailable);
}

uint8_t I2C_receive()
{
  bufferIndex = totalBytes - bytesAvailable;
  if(!bytesAvailable)
  {
    bufferIndex = 0;
    return(0);
  }
  bytesAvailable--;
  return(data[bufferIndex]);
}
*/
  
/*return values for new functions that use the timeOut feature 
  will now return at what point in the transmission the timeout
  occurred. Looking at a full communication sequence between a 
  master and slave (transmit data and then readback data) there
  a total of 7 points in the sequence where a timeout can occur.
  These are listed below and correspond to the returned value:
  1 - Waiting for successful completion of a Start bit
  2 - Waiting for ACK/NACK while addressing slave in transmit mode (MT)
  3 - Waiting for ACK/NACK while sending data to the slave
  4 - Waiting for successful completion of a Repeated Start
  5 - Waiting for ACK/NACK while addressing slave in receiver mode (MR)
  6 - Waiting for ACK/NACK while receiving data from the slave
  7 - Waiting for successful completion of the Stop bit

  All possible return values:
  0           Function executed with no errors
  1 - 7       Timeout occurred, see above list
  8 - 0xFF    See datasheet for exact meaning */ 


/////////////////////////////////////////////////////





uint8_t I2C_write(uint8_t address, uint8_t registerAddress, uint8_t data)
{
  returnStatus = 0;
  returnStatus = I2C_start(); 
  if(returnStatus){return(returnStatus);}
  returnStatus = I2C_sendAddress(SLA_W(address));
  if(returnStatus)
  {
    if(returnStatus == 1){return(2);}
    return(returnStatus);
  }
  returnStatus = I2C_sendByte(registerAddress);
  if(returnStatus)
  {
    if(returnStatus == 1){return(3);}
    return(returnStatus);
  }
  returnStatus = I2C_sendByte(data);
  if(returnStatus)
  {
    if(returnStatus == 1){return(3);}
    return(returnStatus);
  }
  returnStatus = I2C_stop();
  if(returnStatus)
  {
    if(returnStatus == 1){return(7);}
    return(returnStatus);
  }
  return(returnStatus);
}





uint8_t I2C_read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
	uint8_t i = 0;
  bytesAvailable = 0;
  bufferIndex = 0;
  if(numberBytes == 0){numberBytes++;}
  nack = numberBytes - 1;
  returnStatus = 0;
  returnStatus = I2C_start();
  if(returnStatus){return(returnStatus);}
  returnStatus = I2C_sendAddress(SLA_W(address));
  if(returnStatus)
  {
    if(returnStatus == 1){return(2);}
    return(returnStatus);
  }
  returnStatus = I2C_sendByte(registerAddress);
  if(returnStatus)
  {
    if(returnStatus == 1){return(3);}
    return(returnStatus);
  }
  returnStatus = I2C_start();
  if(returnStatus)
  {
    if(returnStatus == 1){return(4);}
    return(returnStatus);
  }
  returnStatus = I2C_sendAddress(SLA_R(address));
  if(returnStatus)
  {
    if(returnStatus == 1){return(5);}
    return(returnStatus);
  }
  for( i = 0; i < numberBytes; i++)
  {
    if( i == nack )
    {
      returnStatus = I2C_receiveByte(0);
      if(returnStatus == 1){return(6);}
      if(returnStatus != MR_DATA_NACK){return(returnStatus);}
    }
    else
    {
      returnStatus = I2C_receiveByte(1);
      if(returnStatus == 1){return(6);}
      if(returnStatus != MR_DATA_ACK){return(returnStatus);}
    }
    dataBuffer[i] = TWDR;
    bytesAvailable = i+1;
    totalBytes = i+1;
  }
  returnStatus = I2C_stop();
  if(returnStatus)
  {
    if(returnStatus == 1){return(7);}
    return(returnStatus);
  }
  return(returnStatus);
}


/////////////// Private Methods ////////////////////////////////////////


uint8_t I2C_start()
{
  unsigned long startingTime = millis();
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)))
  {
    if(!timeOutDelay){continue;}
    if((millis() - startingTime) >= timeOutDelay)
    {
      I2C_lockUp();
      return(1);
    }
       
  }
  if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START))
  {
    return(0);
  }
  if (TWI_STATUS == LOST_ARBTRTN)
  {
    uint8_t bufferedStatus = TWI_STATUS;
    I2C_lockUp();
    return(bufferedStatus);
  }
  return(TWI_STATUS);
}

uint8_t I2C_sendAddress(uint8_t i2cAddress)
{
  TWDR = i2cAddress;
  unsigned long startingTime = millis();
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)))
  {
    if(!timeOutDelay){continue;}
    if((millis() - startingTime) >= timeOutDelay)
    {
      I2C_lockUp();
      return(1);
    }
       
  }
  if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK))
  {
    return(0);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK))
  {
    I2C_stop();
    return(bufferedStatus);
  }
  else
  {
    I2C_lockUp();
    return(bufferedStatus);
  } 
}

uint8_t I2C_sendByte(uint8_t i2cData)
{
  TWDR = i2cData;
  unsigned long startingTime = millis();
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR & (1<<TWINT)))
  {
    if(!timeOutDelay){continue;}
    if((millis() - startingTime) >= timeOutDelay)
    {
      I2C_lockUp();
      return(1);
    }
       
  }
  if (TWI_STATUS == MT_DATA_ACK)
  {
    return(0);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if (TWI_STATUS == MT_DATA_NACK)
  {
    I2C_stop();
    return(bufferedStatus);
  }
  else
  {
    I2C_lockUp();
    return(bufferedStatus);
  } 
}

uint8_t I2C_receiveByte(uint8_t ack)
{
  unsigned long startingTime = millis();
  if(ack)
  {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);

  }
  else
  {
    TWCR = (1<<TWINT) | (1<<TWEN);
  }
  while (!(TWCR & (1<<TWINT)))
  {
    if(!timeOutDelay){continue;}
    if((millis() - startingTime) >= timeOutDelay)
    {
      I2C_lockUp();
      return(1);
    }
  }
  if (TWI_STATUS == LOST_ARBTRTN)
  {
    uint8_t bufferedStatus = TWI_STATUS;
    I2C_lockUp();
    return(bufferedStatus);
  }
  return(TWI_STATUS); 
}

uint8_t I2C_stop()
{
  unsigned long startingTime = millis();
  TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
  while ((TWCR & (1<<TWSTO)))
  {
    if(!timeOutDelay){continue;}
    if((millis() - startingTime) >= timeOutDelay)
    {
      I2C_lockUp();
      return(1);
    }
       
  }
  return(0);
}

void I2C_lockUp()
{
  TWCR = 0; //releases SDA and SCL lines to high impedance
  TWCR = _BV(TWEN) | _BV(TWEA); //reinitialize TWI 
}
