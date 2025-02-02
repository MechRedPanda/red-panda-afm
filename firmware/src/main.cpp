#include <Arduino.h>
#include "AD5761.hpp"
#include "ads868x.hpp"
#include <SerialTransfer.h>
#include <CmdParser.hpp>
#include <CmdBuffer.hpp>

#define VSPI_SCK 18
#define VSPI_MISO 19
#define VSPI_MOSI 23

SPIClass opu_spi = SPIClass(VSPI);

AD5761 dac_f = AD5761(&opu_spi, 16, 0b0000000101101); // Constructor
AD5761 dac_t = AD5761(&opu_spi, 17, 0b0000000101101); // Constructor

ADC_ads868x adc = ADC_ads868x(&opu_spi, 1, 5);

// Create parser and buffer objects
CmdParser cmdParser;
CmdBuffer<64> myBuffer; // Command buffer with a capacity of 64 characters

SerialTransfer myTransfer;

// Command Handlers
void handleReset()
{
  dac_f.reset();
  dac_t.reset();
  adc.reset();
}

void handleDac()
{
  const char *param1 = cmdParser.getCmdParam(1); // Get first parameter (e.g., "T" or "F")
  const char *param2 = cmdParser.getCmdParam(2); // Get second parameter as string

  if (param1 == nullptr || param2 == nullptr)
  {
    Serial.println("Missing DAC subcommand or value");
    return;
  }

  int value = atoi(param2); // Convert the second parameter to an integer

  if (strcmp(param1, "T") == 0)
  {
    dac_t.write(value);
  }
  else if (strcmp(param1, "F") == 0)
  {
    dac_f.write(value);
  }
  else
  {
    Serial.println("Unknown DAC subcommand");
  }
}

void handleAdc()
{
  Serial.println(adc.readADC());
}

void setup()
{
  // Initialize digital pin D2 as an output
  pinMode(2, OUTPUT);
  opu_spi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, -1); // -1 means no default chip select pin
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  myTransfer.begin(Serial);
}
void loop()
{
  if (myBuffer.readFromSerial(&Serial, 30000))
  { // Timeout set to 30 seconds
    // Parse the command from the buffer
    if (cmdParser.parseCmd(&myBuffer) != CMDPARSER_ERROR)
    {
      const char *cmd = cmdParser.getCommand();

      if (strcmp(cmd, "RESET") == 0)
      {
        handleReset();
      }
      else if (strcmp(cmd, "D") == 0)
      {
        handleDac();
      }
      else if (strcmp(cmd, "A") == 0)
      {
        handleAdc();
      }
      else
      {
        Serial.println("Unknown command");
      }
    }
    else
    {
      Serial.println("Parser error!");
    }
  }
}