#include "Adafruit_MCP4725.h"

/**************************************************************************/
/*!
    @brief  Setups the hardware and checks the DAC was found
    @param i2c_address The I2C address of the DAC, defaults to 0x62
*/
/**************************************************************************/
Adafruit_MCP4725::Adafruit_MCP4725(uint8_t i2c_addr) 
{
    if((fd = wiringPiI2CSetup(i2c_addr)) < 0) {
        printf("wiringPiI2CSetup failed\n");
    }
}

Adafruit_MCP4725::~Adafruit_MCP4725(){}

/**************************************************************************/
/*!
    @brief  Sets the output voltage to a fraction of source vref.  (Value
            can be 0..4095)
    @param[in]  output
                The 12-bit value representing the relationship between
                the DAC's input voltage and its output voltage.
    @param[in]  writeEEPROM
                If this value is true, 'output' will also be written
                to the MCP4725's internal non-volatile memory, meaning
                that the DAC will retain the current voltage output
                after power-down or reset.
    @returns True if able to write the value over I2C
*/
/**************************************************************************/
bool Adafruit_MCP4725::setVoltage(uint16_t output, bool writeEEPROM) 
{
    uint8_t packet[3];

    if (writeEEPROM) {
        packet[0] = MCP4725_CMD_WRITEDACEEPROM;
        wiringPiI2CWrite(fd, MCP4725_CMD_WRITEDACEEPROM);
    } else {
        packet[0] = MCP4725_CMD_WRITEDAC;
        wiringPiI2CWrite(fd, MCP4725_CMD_WRITEDAC);
    }

    // limit check value
    output = (output > 4095) ? 4095 : output;

    // MCP4725 expects a 12bit data stream in two bytes (2nd & 3rd of transmission)
    packet[1] = (output >> 8) & 0xFF; // [0 0 0 0 D12 D11 D10 D9 D8] (first bits are modes for our use 0 is fine)
    packet[2] = output; // [D7 D6 D5 D4 D3 D2 D1 D0]
    
    // printf("packet 1 = %X ", packet[1]);
    // printf("packet 2 = %X\n", packet[2]);

    // send our data using the register parameter as our first data byte
    // this ensures the data stream is as the MCP4725 expects
    wiringPiI2CWriteReg8(fd, packet[1], packet[2]);

    return true;
}
