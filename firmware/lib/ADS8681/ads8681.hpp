#ifndef ADS8681_HPP
#define ADS8681_HPP

#include <Arduino.h>
#include <SPI.h>

#define ADS8681_RANGE_SEL_REG 0x14

#define ADS8681_NOP         0b0000000
#define ADS8681_WRITE_FULL  0b1101000

class ADC_ads8681
{
public:
  ADC_ads8681(SPIClass *spi, uint8_t cs_pin, uint16_t range_sel = 0x0004);
  void transmit(uint8_t command, uint16_t address, uint16_t data);
  uint16_t readADC(void);
  void reset(void);

private:
  SPIClass *spi;
  SPISettings _spi_settings = SPISettings(10000000, MSBFIRST, SPI_MODE0);
  uint8_t _cs_pin;
  uint16_t _range_sel;
};

#endif // ADS8681_HPP