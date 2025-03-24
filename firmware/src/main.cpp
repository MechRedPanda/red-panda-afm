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

#define REST 0

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

void handleTriangleWave(int numPeriods)
{
  const int frequency = 1;                    // Hz
  const int period = 1000000 / frequency;     // Period in microseconds
  const int amplitude = 30000;                // Example amplitude value
  const int center = 32767;                   // 16-bit signed midpoint
  const int steps = 1000;                     // Steps per phase
  const int stepDelay = period / (4 * steps); // Delay per step
  const int stepSize = amplitude / steps;     // Full swing step (600)

  AD5761 *axes[] = {&dac_x, &dac_y}; // X/Y DAC array
  int currentAxis = 0;               // Start with X

  for (int swapCount = 0; swapCount < 10; swapCount++)
  {
    for (int periodCount = 0; periodCount < numPeriods * 10; periodCount++)
    {
      AD5761 *dac = axes[currentAxis];

      dac->write(center); // Start at center
      delayMicroseconds(stepDelay);

      // Ramp up: Center -> +Amplitude (100 steps)
      for (int i = 0; i <= steps; i++)
      {
        dac->write(center + i * (amplitude / steps)); //
        delayMicroseconds(stepDelay);
      }

      // Ramp down: +Amplitude -> -Amplitude (200 steps)
      for (int i = 0; i <= steps * 2; i++)
      {
        dac->write(center + amplitude - i * stepSize); //
        delayMicroseconds(stepDelay);
      }

      // Corrected return phase: -Amplitude -> Center (100 steps)
      for (int i = 0; i <= steps; i++)
      {
        dac->write(center - amplitude + i * (amplitude / steps)); // +300/step
        delayMicroseconds(stepDelay);
      }

      dac->write(center); // Final center alignment
    }
    currentAxis = 1 - currentAxis; // Toggle X/Y
  }
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

void toneDAC(AD5761 &dac, int frequency, int duration = -1)
{
  const int center = 32767;                  // Midpoint of DAC output
  const int amplitude = 5000;                // Amplitude of the waveform
  const int halfPeriod = 500000 / frequency; // Half-period in microseconds (for square wave)

  unsigned long startTime = millis(); // Track start time

  while (duration < 0 || millis() - startTime < duration)
  {
    unsigned long t1 = micros(); // Record the start time before writing

    // High phase
    dac.write(center + amplitude);
    unsigned long t2 = micros(); // Measure time after write
    int elapsed = t2 - t1;       // Time taken by dac.write()
    if (elapsed < halfPeriod)
    {
      delayMicroseconds(halfPeriod - elapsed);
    }

    t1 = micros(); // Record time before second write

    // Low phase
    dac.write(center - amplitude);
    t2 = micros();
    elapsed = t2 - t1;
    if (elapsed < halfPeriod)
    {
      delayMicroseconds(halfPeriod - elapsed);
    }
  }

  dac.write(center); // Reset to center voltage when done
}

void toneDAC(int frequency, int duration = -1)
{
  toneDAC(dac_z, frequency, duration);
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

int tempo = 120;

int melody[][2] = {
    {440, -4},
    {440, -4},
    {440, 16},
    {440, 16},
    {440, 16},
    {440, 16},
    {349, 8},
    {REST, 8},
    {440, -4},
    {440, -4},
    {440, 16},
    {440, 16},
    {440, 16},
    {440, 16},
    {349, 8},
    {REST, 8},
    {440, 4},
    {440, 4},
    {440, 4},
    {349, -8},
    {523, 16},
    {440, 4},
    {349, -8},
    {523, 16},
    {440, 2},
    {659, 4},
    {659, 4},
    {659, 4},
    {698, -8},
    {523, 16},
    {440, 4},
    {349, -8},
    {523, 16},
    {440, 2},
    {880, 4},
    {440, -8},
    {440, 16},
    {880, 4},
    {831, -8},
    {784, 16},
    {622, 16},
    {587, 16},
    {622, 8},
    {REST, 8},
    {440, 8},
    {622, 4},
    {587, -8},
    {554, 16},
    {523, 16},
    {494, 16},
    {523, 16},
    {REST, 8},
    {349, 8},
    {415, 4},
    {349, -8},
    {440, -16},
    {523, 4},
    {440, -8},
    {523, 16},
    {659, 2},
    {880, 4},
    {440, -8},
    {440, 16},
    {880, 4},
    {831, -8},
    {784, 16},
    {622, 16},
    {587, 16},
    {622, 8},
    {REST, 8},
    {440, 8},
    {622, 4},
    {587, -8},
    {554, 16},
    {523, 16},
    {494, 16},
    {523, 16},
    {REST, 8},
    {349, 8},
    {415, 4},
    {349, -8},
    {440, -16},
    {440, 4},
    {349, -8},
    {523, 16},
    {440, 2},
};

int notes = sizeof(melody) / sizeof(melody[0]);
int wholenote = (60000 * 4) / tempo;

void imperialmarch()
{
  for (int i = 0; i < notes; i++)
  {
    int frequency = melody[i][0];
    int duration;
    int divider = melody[i][1];

    if (divider > 0)
    {
      duration = wholenote / divider;
    }
    else
    {
      duration = (wholenote / abs(divider)) * 1.5;
    }

    if (frequency != REST)
    {
      toneDAC(frequency, duration * 0.9);
    }
    delay(duration * 0.1);
  }
}

void playNokiaRingtone()
{
  int tempo = 180;
  int melody[] = {
      659, 8, 587, 8, 370, 4, 415, 4,
      554, 8, 494, 8, 294, 4, 330, 4,
      494, 8, 440, 8, 277, 4, 330, 4,
      440, 2};

  int notes = sizeof(melody) / sizeof(melody[0]) / 2;
  int wholenote = (60000 * 4) / tempo;
  int divider = 0, noteDuration = 0;

  for (int thisNote = 0; thisNote < notes * 2; thisNote += 2)
  {
    divider = melody[thisNote + 1];
    if (divider > 0)
    {
      noteDuration = wholenote / divider;
    }
    else
    {
      noteDuration = (wholenote / abs(divider)) * 1.5;
    }

    toneDAC(melody[thisNote], noteDuration * 0.9);
  }
}

void handleTone()
{
  const char *param1 = cmdParser.getCmdParam(1); // DAC identifier (e.g., "T", "F", "X", etc.)
  const char *param2 = cmdParser.getCmdParam(2); // Frequency (Hz)
  const char *param3 = cmdParser.getCmdParam(3); // Duration (ms), optional

  if (param1 == nullptr || param2 == nullptr)
  {
    Serial.println("Invalid tone command. Use: P <DAC> <frequency> [duration]");
    return;
  }

  int frequency = atoi(param2);
  int duration = (param3 != nullptr) ? atoi(param3) : -1; // If no duration is provided, play indefinitely

  AD5761 *selectedDAC = nullptr;

  if (strcmp(param1, "T") == 0)
    selectedDAC = &dac_t;
  else if (strcmp(param1, "F") == 0)
    selectedDAC = &dac_f;
  else if (strcmp(param1, "X") == 0)
    selectedDAC = &dac_x;
  else if (strcmp(param1, "Y") == 0)
    selectedDAC = &dac_y;
  else if (strcmp(param1, "Z") == 0)
    selectedDAC = &dac_z;
  else
  {
    Serial.println("Unknown DAC identifier.");
    return;
  }

  if (selectedDAC)
  {
    toneDAC(*selectedDAC, frequency, duration);
  }
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
        handleTriangleWave(1);
      }
      else if (strcmp(cmd, "M") == 0)
      {
        handleMotor();
      }
      else if (strcmp(cmd, "P") == 0)
      {
        // playNokiaRingtone();
        // playHappyBirthday();
        imperialmarch();
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