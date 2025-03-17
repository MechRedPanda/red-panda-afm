#include <Arduino.h>
#include "AD5761.hpp"
#include "ads868x.hpp"
#include <SerialTransfer.h>
#include <CmdParser.hpp>
#include <CmdBuffer.hpp>

// Define pin connections
#define DIR_PIN 27
#define STEP_PIN1 32
#define STEP_PIN2 33
#define STEP_PIN3 25
#define SLEEP_PIN 26

// Define motor interface type
#define motorInterfaceType 1

#define VSPI_SCK 18
#define VSPI_MISO 19
#define VSPI_MOSI 23

#define HSPI_SCK 14
#define HSPI_MISO 12
#define HSPI_MOSI 13

SPIClass opu_spi = SPIClass(VSPI);
SPIClass driver_spi = SPIClass(HSPI);

AD5761 dac_f = AD5761(&opu_spi, 16, 0b0000000101101); // Constructor
AD5761 dac_t = AD5761(&opu_spi, 17, 0b0000000101101); // Constructor

AD5761 dac_x = AD5761(&driver_spi, 15, 0b0000000101000); // Constructor
AD5761 dac_y = AD5761(&driver_spi, 2, 0b0000000101000);  // Constructor
AD5761 dac_z = AD5761(&driver_spi, 4, 0b0000000101000);  // Constructor

ADC_ads868x adc_fe = ADC_ads868x(&opu_spi, 1, 21);
ADC_ads868x adc_rf = ADC_ads868x(&opu_spi, 1, 5);

// Create parser and buffer objects
CmdParser cmdParser;
CmdBuffer<64> myBuffer; // Command buffer with a capacity of 64 characters

// Command Handlers
void handleReset()
{
  dac_f.reset();
  dac_t.reset();
  dac_x.reset();
  dac_y.reset();
  dac_z.reset();
  adc_fe.reset();
  adc_rf.reset();
  Serial.println("Reset complete");
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
  else if (strcmp(param1, "X") == 0)
  {
    dac_x.write(value);
  }
  else if (strcmp(param1, "Y") == 0)
  {
    dac_y.write(value);
  }
  else if (strcmp(param1, "Z") == 0)
  {
    dac_z.write(value);
  }
  else
  {
    Serial.println("Unknown DAC subcommand");
  }
}

void handleAdc()
{
  const char *param1 = cmdParser.getCmdParam(1); // Get first parameter (e.g., "FE" or "RF")

  if (param1 == nullptr)
  {
    Serial.println("Missing ADC subcommand");
    return;
  }

  if (strcmp(param1, "FE") == 0)
  {
    Serial.println(adc_fe.readADC());
  }
  else if (strcmp(param1, "RF") == 0)
  {
    Serial.println(adc_rf.readADC());
  }
  else
  {
    Serial.println("Unknown ADC subcommand");
  }
}

void handleTriangleWave()
{
  const int frequency = 1000;                      // Hz
  const int period = 1000000 / frequency;       // Period in microseconds
  const int amplitude = 20000;                  // Example amplitude value, adjust as needed
  const int center = 32767;                     // Center point
  const int steps = 100;                        // Number of steps per half-period (adjust for smoothness)
  const int stepDelay = (period / 2) / steps;   // Delay per step
  const int stepSize = (2 * amplitude) / steps; // Value increment per step

  unsigned long startTime = micros();
  while (micros() - startTime < 10000000) // Run for 10 second
  {
    // Rising ramp
    for (int i = 0; i <= steps; i++)
    {
      dac_x.write(center - amplitude + i * stepSize);
      delayMicroseconds(stepDelay);
    }

    // Falling ramp
    for (int i = 0; i <= steps; i++)
    {
      dac_x.write(center + amplitude - i * stepSize);
      delayMicroseconds(stepDelay);
    }
  }
  dac_x.write(center); // Reset to center point
}

void stepMotor(int steps, int delayMicros)
{
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN3, HIGH);
    delayMicroseconds(delayMicros);
    digitalWrite(STEP_PIN3, LOW);
    delayMicroseconds(delayMicros);
  }
}

void testStepMotor(char direction, int steps)
{
  Serial.printf("Testing stepper motor... Direction: %c, Steps: %d\n", direction, steps);

  digitalWrite(SLEEP_PIN, HIGH);

  if (direction == 'F')
  {
    digitalWrite(DIR_PIN, HIGH); // Move Forward
  }
  else if (direction == 'B')
  {
    digitalWrite(DIR_PIN, LOW); // Move Backward
  }
  else
  {
    Serial.println("Invalid direction! Use 'F' for forward or 'B' for backward.");
    return;
  }

  stepMotor(steps, 100); // Adjust speed by modifying delay

  digitalWrite(SLEEP_PIN, LOW);
  Serial.println("Stepper motor test complete.");
}

void handleMotor()
{
  const char *param1 = cmdParser.getCmdParam(1); // Direction (F or B)
  const char *param2 = cmdParser.getCmdParam(2); // Steps count

  if (param1 == nullptr || param2 == nullptr)
  {
    Serial.println("Invalid motor command. Use: M F 200 or M B 200");
    return;
  }

  char direction = param1[0];
  int steps = atoi(param2);

  testStepMotor(direction, steps);
}

void setup()
{
  opu_spi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, -1);    // -1 means no default chip select pin
  driver_spi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1); // -1 means no default chip select pin
  // Start serial communication at 115200 baud
  Serial.begin(115200);

  // Step Motors
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
}
void loop()
{
  if (myBuffer.readFromSerial(&Serial, 30000))
  { // Timeout set to 30 seconds
    // Parse the command from the buffer
    if (cmdParser.parseCmd(&myBuffer) != CMDPARSER_ERROR)
    {
      const char *cmd = cmdParser.getCommand();

      if (strcmp(cmd, "R") == 0)
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
      else if (strcmp(cmd, "T") == 0)
      {
        handleTriangleWave();
      }
      else if (strcmp(cmd, "M") == 0)
      {
        handleMotor();
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