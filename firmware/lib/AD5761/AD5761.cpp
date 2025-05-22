/**************************************************************************/
/*
AD5761
*/
/**************************************************************************/

#include "Arduino.h"
#include "AD5761.hpp"
#include "SPI.h"

/**************************************************************************/
/*
    Set the output of a single DAC channel.
*/
/**************************************************************************/
void AD5761::reset()
{
    // AD5761 software reset
    write(CMD_SW_FULL_RESET, 0);
    // Set the Mode of AD5761
    write(CMD_WR_CTRL_REG, _mode);
}

// SPI Manputaion
void AD5761::write(uint8_t reg_addr_cmd, uint16_t reg_data)
{
    uint8_t data[3];

    digitalWrite(_cs, LOW);
    _spi->beginTransaction(_spi_settings);
    data[0] = reg_addr_cmd;
    data[1] = (reg_data & 0xFF00) >> 8;
    data[2] = (reg_data & 0x00FF) >> 0;
    for (int i = 0; i < 3; i++)
    {
        _spi->transfer(data[i]);
    }
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

void AD5761::write(uint16_t reg_data)
{
    write(CMD_WR_UPDATE_DAC_REG, reg_data);
}

void AD5761::write_volt(float voltage)
{
    int set_val = (int)((voltage / 2.5 + 4) / 8 * 65536);
    write(CMD_WR_UPDATE_DAC_REG, set_val);
}

void AD5761::setMode(uint16_t mode)
{
    _mode = mode;  // Update the internal mode member variable
    write(CMD_WR_CTRL_REG, _mode);  // Write to the control register to apply the new mode
}

void AD5761::read(uint8_t reg_addr_cmd)
{
    digitalWrite(_cs, LOW);
    delay(1);
    _spi->beginTransaction(_spi_settings);
    _spi_buffer[0] = _spi->transfer(reg_addr_cmd);
    _spi_buffer[1] = _spi->transfer(0xFF); // dummy
    _spi_buffer[2] = _spi->transfer(0xFF); // dummy
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
    delay(1);
}