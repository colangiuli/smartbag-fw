
#ifndef I2C_h
#define I2C_h

#include <inttypes.h>
#include <avr/io.h>

#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK	0x18
#define MT_SLA_NACK	0x20
#define MT_DATA_ACK     0x28
#define MT_DATA_NACK    0x30
#define MR_SLA_ACK	0x40
#define MR_SLA_NACK	0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define LOST_ARBTRTN    0x38
#define TWI_STATUS      (TWSR & 0xF8)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

#define MAX_BUFFER_SIZE 32

    void I2C_init();
    void I2C_disable();
    void I2C_timeOut(uint16_t);
    void I2C_setSpeed(uint8_t); 
    void I2C_pullup(uint8_t);
    void I2C_scan();
    uint8_t I2C_available();
    uint8_t I2C_receive();
////////// I2C write /////////////////////
    uint8_t I2C_write(uint8_t address, uint8_t registerAddress, uint8_t data);
////////// I2C read /////////////////////
    uint8_t I2C_read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer);

    uint8_t I2C_start();
    uint8_t I2C_sendAddress(uint8_t);
    uint8_t I2C_sendByte(uint8_t);
    uint8_t I2C_receiveByte(uint8_t);
    uint8_t I2C_stop();
    void I2C_lockUp();



#endif
