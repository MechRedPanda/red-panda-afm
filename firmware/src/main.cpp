#include <Arduino.h>
#include "AD5761.hpp"
#include "ads868x.hpp"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <config.hpp>

// Define pin connections and SPI interfaces
#define DIR_PIN 27
#define STEP_PIN1 32
#define STEP_PIN2 33
#define STEP_PIN3 25
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

ADC_ads868x adc_0 = ADC_ads868x(&opu_spi, 1, 21);
ADC_ads868x adc_1 = ADC_ads868x(&opu_spi, 1, 5);

WiFiServer server(PORT); // Match the port in Python code

struct AFM_State
{
  uint16_t dac_f_val = 0;
  uint16_t dac_t_val = 0;
  uint16_t dac_x_val = 0;
  uint16_t dac_y_val = 0;
  uint16_t dac_z_val = 0;
  uint16_t adc_0_val = 0;
  uint16_t adc_1_val = 0;
  int32_t stepper_position_0 = 0;
  int32_t timestamp = 0;
};

void handleReset()
{
  dac_f.reset();
  dac_t.reset();
  dac_x.reset();
  dac_y.reset();
  dac_z.reset();
  adc_0.reset();
  adc_1.reset();
  Serial.println("Reset complete");
}

AFM_State current_afm_state;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 100; // Send state every 100ms

void updateAFMState()
{
  current_afm_state.adc_0_val = adc_0.readADC(); // Using FE channel as default
  current_afm_state.adc_1_val = adc_1.readADC(); // Using FE channel as default
  current_afm_state.timestamp = millis();
}

String getAFMStateAsJson()
{
  // Allocate memory for JSON document
  JsonDocument doc;

  // Fill the JSON document with AFM state data
  doc["dac_f"] = current_afm_state.dac_f_val;
  doc["dac_t"] = current_afm_state.dac_t_val;
  doc["dac_x"] = current_afm_state.dac_x_val;
  doc["dac_y"] = current_afm_state.dac_y_val;
  doc["dac_z"] = current_afm_state.dac_z_val;
  doc["adc_0"] = current_afm_state.adc_0_val;
  doc["adc_1"] = current_afm_state.adc_1_val;
  doc["timestamp"] = current_afm_state.timestamp;
  doc["stepper_position_0"] = current_afm_state.stepper_position_0;

  String output;
  serializeJson(doc, output);
  return output;
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
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    response["status"] = "error";
    response["message"] = "Invalid JSON";
    String output;
    serializeJson(response, output);
    return output;
  }

  if (!doc.containsKey("command"))
  {
    Serial.println("command found in JSON");
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
  else if (strcmp(command, "read_adc") == 0)
  {
    int adc_0_value = adc_0.readADC(); // Using FE channel as default
    response["adc_0"] = adc_0_value;
    int adc_1_value = adc_1.readADC(); // Using FE channel as default
    response["adc_1"] = adc_1_value;
    response["status"] = "success";
  }
  else if (strcmp(command, "set_dac") == 0)
  {
    const char *channel = doc["channel"];
    int value = doc["value"];

    if (strcmp(channel, "T") == 0)
    {
      dac_t.write(value);
      response["status"] = "success";
    }
    else if (strcmp(channel, "F") == 0)
    {
      dac_f.write(value);
      response["status"] = "success";
    }
    else
    {
      response["status"] = "error";
      response["message"] = "Invalid DAC channel";
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

// Your PID control function
void runPIDControl(void *parameter)
{
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(1); // 1 ms to allow FreeRTOS scheduling

  while (1)
  {
    // Run the PID loop (replace with your actual PID function)
    updateAFMState(); // Update the AFM state

    // Perform the control calculations
    // Example: double output = pidController.compute(input);

    // Adjust output hardware (DAC, PWM, etc.)
    // Example: analogWrite(PWM_PIN, output);

    // Allow some FreeRTOS scheduling (tweak delay if needed)
    vTaskDelay(xMaxBlockTime);
  }
}

void setup()
{
  opu_spi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, -1);
  driver_spi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1);

  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);

  // Connect to WiFi
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nConnected to Wi-Fi!");
  server.begin();
  Serial.println("Server started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  xTaskCreatePinnedToCore(
      runPIDControl,            // Function to run
      "PIDTask",                // Task name
      4096,                     // Stack size
      NULL,                     // Task parameter
      configMAX_PRIORITIES - 1, // Highest priority
      &PIDTaskHandle,           // Task handle
      0                         // Run on Core 0
  );
}

void loop()
{
  WiFiClient client = server.available();

  if (client)
  {
    Serial.println("New client connected");

    while (client.connected())
    {
      if (client.available())
      {
        String jsonCommand = client.readStringUntil('\n');
        jsonCommand.trim();

        Serial.print("Received: ");
        Serial.println(jsonCommand);

        String response = processCommand(jsonCommand);
        client.println(response);

        Serial.print("Sent: ");
        Serial.println(response);
      }
      else
      {
        // Send AFM state updates periodically when idle
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= updateInterval)
        {
          lastUpdateTime = currentTime;
          Serial.print("Sent AFM state: ");
          client.println(getAFMStateAsJson());
        }
      }
    }

    client.stop();
    Serial.println("Client disconnected");
  }
}