#include <Arduino.h>
#include "AD5761.hpp"
#include "ads868x.hpp"
#include <ArduinoJson.h>
#include <atomic>
#include "esp_task_wdt.h"

// Define pin connections and SPI interfaces
#define DIR_PIN 27
#define STEP_PIN3 32
#define STEP_PIN2 33
#define STEP_PIN1 25
#define SLEEP_PIN 26

#define VSPI_SCK 18
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define HSPI_SCK 14
#define HSPI_MISO 12
#define HSPI_MOSI 13

SPIClass opu_spi = SPIClass(VSPI);
SPIClass driver_spi = SPIClass(HSPI);

// Initialize DACs and ADCs
AD5761 dac_f = AD5761(&opu_spi, 16, 0b0000000101101);
AD5761 dac_t = AD5761(&opu_spi, 17, 0b0000000101101);
AD5761 dac_x = AD5761(&driver_spi, 15, 0b0000000101000);
AD5761 dac_y = AD5761(&driver_spi, 2, 0b0000000101000);
AD5761 dac_z = AD5761(&driver_spi, 4, 0b0000000101000);

ADC_ads868x adc_0 = ADC_ads868x(&opu_spi, 1, 21, 0x0004);
ADC_ads868x adc_1 = ADC_ads868x(&opu_spi, 1, 5, 0x0003);

enum State
{
  IDLE = 0,
  FOCUSING = 1,
  APPROACHING = 2,
  SCANNING = 3,
};

enum MotorStatus
{
  MOTOR_IDLE = 0,
  MOTOR_MOVING = 1,
  MOTOR_ERROR = 2
};

struct MotorState
{
  int32_t current_position = 0;
  int32_t target_position = 0;
  uint32_t last_update = 0;
  MotorStatus status = MOTOR_IDLE;
  uint8_t active = 0; // 0 = inactive, 1-3 = motor number
};

struct AFM_State
{
  // Existing DAC/ADC values
  uint16_t dac_f_val = 0;
  uint16_t dac_t_val = 0;
  uint16_t dac_x_val = 0;
  uint16_t dac_y_val = 0;
  uint16_t dac_z_val = 0;
  uint16_t adc_0_val = 0;
  uint16_t adc_1_val = 0;

  // Motor tracking for all three motors
  MotorState motors[3]; // Index 0-2 for motors 1-3

  // System state
  int32_t timestamp = 0;
  enum State current_state = IDLE;
};

AFM_State current_afm_state;

// Global motor control state
typedef struct
{
  uint8_t activeMotor;
  int remainingSteps;
  int direction;
  unsigned long nextStepTime;
  unsigned int stepDelay;
  bool isMoving;
} MotorControlState;

MotorControlState motorCtrlState = {0, 0, 0, 0, 500, false};

void handleReset()
{
  dac_f.reset();
  dac_f.write(1<<15); // Set to 0V
  dac_t.reset();
  dac_t.write(1<<15); // Set to 0V
  dac_x.reset();
  dac_x.write(1<<15); // Set to 0V
  dac_y.reset();
  dac_y.write(1<<15); // Set to 0V
  dac_z.reset();
  dac_z.write(1<<15); // Set to 0V
  adc_0.reset();
  adc_1.reset();
  current_afm_state.dac_f_val = 1<<15;
  current_afm_state.dac_t_val = 1<<15;
  current_afm_state.dac_x_val = 1<<15;
  current_afm_state.dac_y_val = 1<<15;
  current_afm_state.dac_z_val = 1<<15;
  current_afm_state.current_state = IDLE;
}

void handleRestore()
{
  dac_f.reset();
  dac_f.write(current_afm_state.dac_f_val); // Restore previous value
  dac_t.reset();
  dac_t.write(current_afm_state.dac_t_val); // Restore previous value
  dac_x.reset();
  dac_x.write(current_afm_state.dac_x_val); // Restore previous value
  dac_y.reset();
  dac_y.write(current_afm_state.dac_y_val); // Restore previous value
  dac_z.reset();
  dac_z.write(current_afm_state.dac_z_val); // Restore previous value
  adc_0.reset();
  adc_1.reset();
}

// Initialize motor control pins
void setupMotorPins()
{
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);

  // Start with drivers in sleep mode
  digitalWrite(SLEEP_PIN, LOW);
}

bool startMotorMovement(uint8_t motorNumber, int steps, int direction, unsigned int stepDelay = 500)
{
  if (motorNumber < 1 || motorNumber > 3)
    return false;

  // Validate
  if (steps <= 0 || motorCtrlState.isMoving ||
      current_afm_state.motors[motorNumber - 1].status == MOTOR_MOVING)
  {
    return false;
  }

  // Update AFM state
  current_afm_state.motors[motorNumber - 1].target_position =
      current_afm_state.motors[motorNumber - 1].current_position +
      (direction ? steps : -steps);
  current_afm_state.motors[motorNumber - 1].status = MOTOR_MOVING;
  current_afm_state.motors[motorNumber - 1].last_update = millis();
  current_afm_state.motors[motorNumber - 1].active = motorNumber;

  // Hardware control
  digitalWrite(SLEEP_PIN, HIGH);
  delay(1); // Wake-up time
  digitalWrite(DIR_PIN, direction);
  delayMicroseconds(50);
  // Update control state
  motorCtrlState.activeMotor = motorNumber;
  motorCtrlState.remainingSteps = steps;
  motorCtrlState.direction = direction;
  motorCtrlState.stepDelay = stepDelay;
  motorCtrlState.nextStepTime = micros() + stepDelay;
  motorCtrlState.isMoving = true;
  
  return true;
}

void updateMotorMovement()
{
  if (!motorCtrlState.isMoving)
    return;

  unsigned long currentTime = micros();
  if (currentTime >= motorCtrlState.nextStepTime)
  {
    // Calculate how many steps we need to take to catch up
    unsigned long timeSinceLastStep = currentTime - motorCtrlState.nextStepTime;
    int stepsToTake = 1 + (timeSinceLastStep / motorCtrlState.stepDelay);
    
    // Limit steps to remaining steps
    stepsToTake = min(stepsToTake, motorCtrlState.remainingSteps);
    
    // Pulse the appropriate step pin for each step
    uint8_t stepPin;
    switch (motorCtrlState.activeMotor)
    {
    case 1:
      stepPin = STEP_PIN1;
      break;
    case 2:
      stepPin = STEP_PIN2;
      break;
    case 3:
      stepPin = STEP_PIN3;
      break;
    default:
      return;
    }

    for (int i = 0; i < stepsToTake; i++)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(20);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(20);
    }

    // Update control state
    motorCtrlState.remainingSteps -= stepsToTake;
    motorCtrlState.nextStepTime = currentTime + motorCtrlState.stepDelay;

    // Update AFM state
    uint8_t motorIdx = motorCtrlState.activeMotor - 1;
    current_afm_state.motors[motorIdx].current_position +=
        (motorCtrlState.direction ? stepsToTake : -stepsToTake);
    current_afm_state.motors[motorIdx].last_update = millis();

    // Check completion
    if (motorCtrlState.remainingSteps <= 0)
    {
      motorCtrlState.isMoving = false;
      current_afm_state.motors[motorIdx].status = MOTOR_IDLE;
      digitalWrite(SLEEP_PIN, LOW);
    }
  }
}

/**
 * Checks if any motor is currently moving
 * @return True if any motor is active, false otherwise
 */
bool isMotorMoving()
{
  // Check both the control state and individual motor states
  if (motorCtrlState.isMoving)
  {
    return true;
  }

  // Double-check all motor states
  for (int i = 0; i < 3; i++)
  {
    if (current_afm_state.motors[i].status == MOTOR_MOVING)
    {
      return true;
    }
  }

  return false;
}

/**
 * Stops the currently moving motor immediately
 * @return True if a motor was stopped, false if no motor was moving
 */
bool stopMotor()
{
  if (!motorCtrlState.isMoving)
  {
    return false;
  }

  // Update AFM state for the active motor
  uint8_t motorIdx = motorCtrlState.activeMotor - 1;
  current_afm_state.motors[motorIdx].status = MOTOR_IDLE;
  current_afm_state.motors[motorIdx].last_update = millis();

  // Reset control state
  motorCtrlState.isMoving = false;
  motorCtrlState.activeMotor = 0;

  // Put driver to sleep
  digitalWrite(SLEEP_PIN, LOW);

  return true;
}

// Global atomic flag for ADC status
std::atomic<bool> adcBusy(false);

// Function to check if ADC is in use
bool adcInUse()
{
  return adcBusy.load();
}

// Function to safely claim ADC access
bool acquireADC()
{
  bool expected = false;
  return adcBusy.compare_exchange_strong(expected, true);
}

// Function to release ADC access
void releaseADC()
{
  adcBusy = false;
}

void updateAFMState()
{
  // Skip if ADC is busy
  if (adcInUse())
  {
    return;
  }

  // Try to acquire ADC
  if (acquireADC())
  {
    current_afm_state.adc_0_val = adc_0.readADC();
    current_afm_state.adc_1_val = adc_1.readADC();
    current_afm_state.timestamp = millis();
    releaseADC();
  }
}

JsonDocument getAFMStateAsJson()
{
  // Allocate memory for JSON document
  JsonDocument doc;

  // Core AFM measurements
  doc["data_type"] = "update";
  doc["timestamp"] = current_afm_state.timestamp;
  doc["adc_0"] = current_afm_state.adc_0_val;
  doc["adc_1"] = current_afm_state.adc_1_val;
  doc["dac_f"] = current_afm_state.dac_f_val;
  doc["dac_t"] = current_afm_state.dac_t_val;
  doc["dac_x"] = current_afm_state.dac_x_val;
  doc["dac_y"] = current_afm_state.dac_y_val;
  doc["dac_z"] = current_afm_state.dac_z_val;

  // System state
  doc["state"] = current_afm_state.current_state == IDLE ? "IDLE" : current_afm_state.current_state == FOCUSING  ? "FOCUSING"
                                                                : current_afm_state.current_state == APPROACHING ? "APPROACHING"
                                                                                                                 : "SCANNING";

  // Motor status (all three motors)
  JsonObject motor1 = doc["motor_1"].to<JsonObject>();
  motor1["position"] = current_afm_state.motors[0].current_position;
  motor1["target"] = current_afm_state.motors[0].target_position;
  motor1["is_running"] = current_afm_state.motors[0].status == MOTOR_MOVING;

  JsonObject motor2 = doc["motor_2"].to<JsonObject>();
  motor2["position"] = current_afm_state.motors[1].current_position;
  motor2["target"] = current_afm_state.motors[1].target_position;
  motor2["is_running"] = current_afm_state.motors[1].status == MOTOR_MOVING;

  JsonObject motor3 = doc["motor_3"].to<JsonObject>();
  motor3["position"] = current_afm_state.motors[2].current_position;
  motor3["target"] = current_afm_state.motors[2].target_position;
  motor3["is_running"] = current_afm_state.motors[2].status == MOTOR_MOVING;
  return doc;
}

// New blocking motor movement function
bool moveMotorBlocking(uint8_t motorNumber, int steps, int direction, unsigned int stepDelay = 500)
{
  if (!startMotorMovement(motorNumber, steps, direction, stepDelay))
  {
    return false;
  }

  // Wait until movement is complete
  while (motorCtrlState.isMoving)
  {
    updateMotorMovement();
  }

  return true;
}

String processCommand(const String &jsonCommand)
{
  // Allocate memory for JSON documents
  JsonDocument doc;
  JsonDocument response;

  // Parse incoming JSON
  DeserializationError error = deserializeJson(doc, jsonCommand);

  if (error)
  {
    response["status"] = "error";
    response["message"] = "Invalid JSON";
    String output;
    serializeJson(response, output);
    return output;
  }

  if (!doc["command"].is<String>())
  {
    response["status"] = "error";
    response["message"] = "Invalid DAC channel";
    String output;
    serializeJson(response, output);
    return output;
  }

  const char *command = doc["command"];

  if (strcmp(command, "reset") == 0)
  {
    handleReset();
    response["status"] = "success";
  }
  else if (strcmp(command, "restore") == 0)
  {
    handleRestore();
    response["status"] = "success";
  }
  else if (strcmp(command, "get_status") == 0)
  {
    response = getAFMStateAsJson();
  }
  else if (strcmp(command, "read_adc") == 0)
  {
    unsigned long start = millis();
    while (adcInUse() && (millis() - start < 100))
    {
      delay(1); // Small delay to prevent busy-wait
    }

    if (acquireADC())
    {
      int adc_0_value = adc_0.readADC();
      delayMicroseconds(10);
      int adc_1_value = adc_1.readADC();
      delayMicroseconds(10);
      releaseADC();

      response["adc_0"] = adc_0_value;
      response["adc_1"] = adc_1_value;
      response["status"] = "success";
    }
    else
    {
      response["status"] = "error";
      response["message"] = "ADC busy - timeout";
    }
  }
  else if (strcmp(command, "set_dac") == 0)
  {
    const char *channel = doc["channel"];
    int value = doc["value"];

    if (strcmp(channel, "T") == 0)
    {
      dac_t.write(value);
      current_afm_state.dac_t_val = value; // Update the state
      response["status"] = "success";
    }
    else if (strcmp(channel, "F") == 0)
    {
      dac_f.write(value);
      current_afm_state.dac_f_val = value; // Update the state
      response["status"] = "success";
    }
    else if (strcmp(channel, "X") == 0)
    {
      dac_x.write(value);
      current_afm_state.dac_x_val = value; // Update the state
      response["status"] = "success";
    }
    else if (strcmp(channel, "Y") == 0)
    {
      dac_y.write(value);
      current_afm_state.dac_y_val = value; // Update the state
      response["status"] = "success";
    }
    else if (strcmp(channel, "Z") == 0)
    {
      dac_z.write(value);
      current_afm_state.dac_z_val = value; // Update the state
      response["status"] = "success";
    }
    else
    {
      response["status"] = "error";
      response["message"] = "Invalid DAC channel";
    }
  }
  else if (strcmp(command, "start_motor") == 0)
  {
    // Get references to parameters first
    const auto &motorParam = doc["motor"];
    const auto &stepsParam = doc["steps"];
    const auto &directionParam = doc["direction"];

    // Validate required parameters
    if (!motorParam.is<int>() || !stepsParam.is<int>() || !directionParam.is<int>())
    {
      response["status"] = "error";
      response["message"] = "Missing required parameters (motor, steps, direction)";
    }
    // Validate motor number (1-3)
    else if (motorParam.as<int>() < 1 || motorParam.as<int>() > 3)
    {
      response["status"] = "error";
      response["message"] = "Invalid motor number (must be 1-3)";
    }
    // Validate steps (positive integer)
    else if (stepsParam.as<int>() <= 0)
    {
      response["status"] = "error";
      response["message"] = "Steps must be a positive integer";
    }
    // Validate direction (0 or 1)
    else if (directionParam.as<int>() != 0 && directionParam.as<int>() != 1)
    {
      response["status"] = "error";
      response["message"] = "Direction must be 0 (CCW) or 1 (CW)";
    }
    // Check if another motor is already moving
    else if (isMotorMoving())
    {
      response["status"] = "error";
      response["message"] = "Another motor is already moving";
    }
    else
    {
      // Get optional parameters with defaults
      unsigned int speed = doc["speed"] | 500; // Default 500μs delay
      speed = constrain(speed, 100, 5000);     // Clamp between 100-5000μs

      // Start the movement
      startMotorMovement(
          motorParam.as<int>(),
          stepsParam.as<int>(),
          directionParam.as<int>(),
          speed);

      // Prepare success response
      response["status"] = "started";
      response["motor"] = motorParam.as<int>();
      response["steps"] = stepsParam.as<int>();
      response["direction"] = directionParam.as<int>() ? "CW" : "CCW";
      response["speed"] = speed;
      response["target_position"] = current_afm_state.motors[motorParam.as<int>() - 1].target_position;
    }
  }
  else if (strcmp(command, "get_motor_status") == 0)
  {
    if (doc["motor"].is<int>())
    {
      int motorNum = doc["motor"];
      if (motorNum >= 1 && motorNum <= 3)
      {
        int idx = motorNum - 1;
        response["motor"] = motorNum;
        response["position"] = current_afm_state.motors[idx].current_position;
        response["target"] = current_afm_state.motors[idx].target_position;
        response["status"] = current_afm_state.motors[idx].status;
        response["last_update"] = current_afm_state.motors[idx].last_update;
      }
    }
    else
    {
      response["status"] = "error";
      response["message"] = "Invalid motor number";
    }
  }
  else if (strcmp(command, "stop_all_motors") == 0)
  {
    motorCtrlState.isMoving = false;
    for (int i = 0; i < 3; i++)
    {
      current_afm_state.motors[i].status = MOTOR_IDLE;
    }
    digitalWrite(SLEEP_PIN, LOW);
    response["status"] = "stopped";
  }
  // Command handler version
  else if (strcmp(command, "stop_motor") == 0)
  {
    if (stopMotor())
    {
      response["status"] = "stopped";
      response["motor"] = motorCtrlState.activeMotor + 1; // Returns which motor was stopped
    }
    else
    {
      response["status"] = "idle";
      response["message"] = "No motor was moving";
    }
  }
  // Command handler versions
  else if (strcmp(command, "is_motor_moving") == 0)
  {
    response["is_moving"] = isMotorMoving();
    response["status"] = "success";
  }
  else if (strcmp(command, "set_state") == 0)
  {
    const char *state = doc["state"];

    if (strcmp(state, "IDLE") == 0)
    {
      current_afm_state.current_state = IDLE;
    }
    else if (strcmp(state, "FOCUSING") == 0)
    {
      current_afm_state.current_state = FOCUSING;
    }
    else if (strcmp(state, "APPROACHING") == 0)
    {
      current_afm_state.current_state = APPROACHING;
    }
    else if (strcmp(state, "SCANNING") == 0)
    {
      current_afm_state.current_state = SCANNING;
    }
    else
    {
      response["status"] = "error";
      response["message"] = "Invalid state";
      String output;
      serializeJson(response, output);
      return output;
    }

    response["status"] = "success";
  }
  else if (strcmp(command, "move_motor_blocking") == 0)
  {
    // Get references to parameters first
    const auto &motorParam = doc["motor"];
    const auto &stepsParam = doc["steps"];
    const auto &directionParam = doc["direction"];

    // Validate required parameters
    if (!motorParam.is<int>() || !stepsParam.is<int>() || !directionParam.is<int>())
    {
      response["status"] = "error";
      response["message"] = "Missing required parameters (motor, steps, direction)";
    }
    // Validate motor number (1-3)
    else if (motorParam.as<int>() < 1 || motorParam.as<int>() > 3)
    {
      response["status"] = "error";
      response["message"] = "Invalid motor number (must be 1-3)";
    }
    // Validate steps (positive integer)
    else if (stepsParam.as<int>() <= 0)
    {
      response["status"] = "error";
      response["message"] = "Steps must be a positive integer";
    }
    // Validate direction (0 or 1)
    else if (directionParam.as<int>() != 0 && directionParam.as<int>() != 1)
    {
      response["status"] = "error";
      response["message"] = "Direction must be 0 (CCW) or 1 (CW)";
    }
    // Check if another motor is already moving
    else if (isMotorMoving())
    {
      response["status"] = "error";
      response["message"] = "Another motor is already moving";
    }
    else
    {
      // Get optional parameters with defaults
      unsigned int speed = doc["speed"] | 500; // Default 500μs delay
      speed = constrain(speed, 100, 5000);     // Clamp between 100-5000μs

      // Start the movement and wait for completion
      if (moveMotorBlocking(
          motorParam.as<int>(),
          stepsParam.as<int>(),
          directionParam.as<int>(),
          speed))
      {
        // Prepare success response
        response["status"] = "completed";
        response["motor"] = motorParam.as<int>();
        response["steps"] = stepsParam.as<int>();
        response["direction"] = directionParam.as<int>() ? "CW" : "CCW";
        response["speed"] = speed;
        response["final_position"] = current_afm_state.motors[motorParam.as<int>() - 1].current_position;
      }
      else
      {
        response["status"] = "error";
        response["message"] = "Failed to start motor movement";
      }
    }
  }
  else
  {
    response["status"] = "error";
    response["message"] = "Unknown action";
  }

  String output;
  serializeJson(response, output);
  return output;
}

TaskHandle_t PIDTaskHandle;

void runControl(void *parameter)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1); // 1 ms control loop
  esp_task_wdt_add(NULL);

  while (1)
  {
    updateMotorMovement();
    updateAFMState();
    esp_task_wdt_reset();
    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void setup()
{
  esp_task_wdt_init(5, true); // 5-second timeout, panic on trigger
  opu_spi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, -1);
  driver_spi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1);
  // Initialize motor control pins
  setupMotorPins();
  // Initialize serial communication
  Serial.setRxBufferSize(1024); // Set the RX buffer size to 1024 bytes
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect
  }

  xTaskCreatePinnedToCore(
      runControl,               // Function to run
      "PIDTask",                // Task name
      4096,                     // Stack size
      NULL,                     // Task parameter
      configMAX_PRIORITIES - 2, // 
      &PIDTaskHandle,           // Task handle
      0                         // Run on Core 0
  );
}

void loop()
{
  // Handle serial commands
  if (Serial.available())
  {
    String jsonCommand = Serial.readStringUntil('\n');
    jsonCommand.trim();

    if (jsonCommand.length() > 0)
    {
      String response = processCommand(jsonCommand);
      Serial.println(response);
    }
  }
}