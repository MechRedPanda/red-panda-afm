#include "ads8681.hpp"

ADC_ads8681::ADC_ads8681(SPIClass *spi, uint8_t cs_pin, uint16_t range_sel)
  : spi(spi), _cs_pin(cs_pin), _range_sel(range_sel)
{
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);
  spi->begin();
}

void ADC_ads8681::transmit(uint8_t command, uint16_t address, uint16_t data)
{
  uint8_t tx[4];
  tx[0] = (command << 1) | ((address >> 8) & 0x01);
  tx[1] = address & 0xFF;
  tx[2] = (data >> 8) & 0xFF;
  tx[3] = data & 0xFF;

  digitalWrite(_cs_pin, LOW);
  spi->beginTransaction(_spi_settings);
  spi->transfer(tx, 4);
  spi->endTransaction();
  digitalWrite(_cs_pin, HIGH);
}

uint16_t ADC_ads8681::readADC(void)
{
  uint8_t tx[4] = {0, 0, 0, 0};
  uint8_t rx[4];

  digitalWrite(_cs_pin, LOW);
  spi->beginTransaction(_spi_settings);
  spi->transfer(tx, 4);  // First NOP to initiate conversion
  spi->endTransaction();
  digitalWrite(_cs_pin, HIGH);

  delayMicroseconds(5); // wait for conversion to complete

  digitalWrite(_cs_pin, LOW);
  spi->beginTransaction(_spi_settings);
  spi->transfer(rx, 4);  // Second NOP to get result
  spi->endTransaction();
  digitalWrite(_cs_pin, HIGH);

  uint16_t result = ((uint16_t)rx[0] << 8) | rx[1];
  return result;
}

void ADC_ads8681::reset(void)
{
  transmit(ADS8681_WRITE_FULL, ADS8681_RANGE_SEL_REG, _range_sel);
  delayMicroseconds(100);
}
