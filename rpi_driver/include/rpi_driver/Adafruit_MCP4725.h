#ifndef _ADAFRUIT_MCP4725_H_
#define _ADAFRUIT_MCP4725_H_

#include <iostream>  
#include <wiringPiI2C.h>  
#include <stdio.h>  


#define MCP4725_I2CADDR_DEFAULT (0x60) ///< Default i2c address 0x62
#define MCP4725_CMD_WRITEDAC (0x40)    ///< Writes data to the DAC
#define MCP4725_CMD_WRITEDACEEPROM (0x60) ///< Writes data to the DAC and the EEPROM (persisting the assigned value after reset)

/**************************************************************************/
/*!
    @brief  Class for communicating with an MCP4725 DAC
*/
/**************************************************************************/
class Adafruit_MCP4725 {
public:
    Adafruit_MCP4725(uint8_t i2c_addr=MCP4725_I2CADDR_DEFAULT); 
    ~Adafruit_MCP4725();
    bool setVoltage(uint16_t output, bool writeEEPROM);

private:
    int fd; // device
};

#endif