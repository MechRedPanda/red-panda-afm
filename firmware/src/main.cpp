#include <Arduino.h>
#include "AD5761.hpp"
#include "ads868x.hpp"
#include <ArduinoJson.h>
#include <atomic>
#include "esp_task_wdt.h"

// ============================================================================
// Hardware Configuration
// ============================================================================

// Pin Definitions
#define DIR_PIN    27
#define STEP_PIN1  25
#define STEP_PIN2  33
#define STEP_PIN3  32
#define SLEEP_PIN  26

// SPI Pin Definitions
#define VSPI_SCK   18
#define VSPI_MISO  19
#define VSPI_MOSI  23
#define HSPI_SCK   14
#define HSPI_MISO  12
#define HSPI_MOSI  13

// SPI Interface Instances
SPIClass opu_spi = SPIClass(VSPI);
SPIClass driver_spi = SPIClass(HSPI);

// ============================================================================
// Device Instances
// ============================================================================

// DAC Instances
AD5761 dac_f = AD5761(&opu_spi, 16, 0b0000000101101);
AD5761 dac_t = AD5761(&opu_spi, 17, 0b0000000101101);
AD5761 dac_x = AD5761(&driver_spi, 15, 0b0000000101000);
AD5761 dac_y = AD5761(&driver_spi, 2, 0b0000000101000);
AD5761 dac_z = AD5761(&driver_spi, 4, 0b0000000101000);

// ADC Instances
ADC_ads868x adc_0 = ADC_ads868x(&opu_spi, 1, 21, 0x0004);
ADC_ads868x adc_1 = ADC_ads868x(&opu_spi, 1, 5, 0x0003);

// ============================================================================
// State Definitions
// ============================================================================

// System States
enum class SystemState {
    IDLE = 0,
    FOCUSING = 1,
    APPROACHING = 2,
    SCANNING = 3
};

// Motor States
enum class MotorStatus {
    IDLE = 0,
    MOVING = 1,
    ERROR = 2
};

// Motor State Structure
struct MotorState {
    int32_t current_position = 0;
    int32_t target_position = 0;
    uint32_t last_update = 0;
    MotorStatus status = MotorStatus::IDLE;
    uint8_t active = 0; // 0 = inactive, 1-3 = motor number
};

// AFM System State Structure
struct AFMState {
    // DAC/ADC Values
    uint16_t dac_f_val = 0;
    uint16_t dac_t_val = 0;
    uint16_t dac_x_val = 0;
    uint16_t dac_y_val = 0;
    uint16_t dac_z_val = 0;
    uint16_t adc_0_val = 0;
    uint16_t adc_1_val = 0;

    // Motor States
    MotorState motors[3]; // Index 0-2 for motors 1-3

    // System State
    int32_t timestamp = 0;
    SystemState current_state = SystemState::IDLE;

    // PID Control Parameters
    bool pid_enabled = false;
    uint16_t pid_target = 0;
    float pid_kp = 1.0;
    float pid_ki = 0.0;
    float pid_kd = 0.0;
    bool pid_invert = false;
    float pid_integral = 0;
    uint16_t pid_last_error = 0;
    uint32_t pid_last_time = 0;
    float pid_slew_rate = 1000.0; // Maximum change per second
    float pid_last_output = 0;
    float pid_bias = 0; // Initial bias value
};

// ============================================================================
// Global Variables
// ============================================================================

AFMState current_afm_state;
std::atomic<bool> adc_busy(false);

// Motor Control State
struct MotorControlState {
    uint8_t active_motor;
    int remaining_steps;
    int direction;
    unsigned long next_step_time;
    unsigned int step_delay;
    bool is_moving;
};

MotorControlState motor_ctrl_state = {0, 0, 0, 0, 500, false};

// ============================================================================
// Hardware Initialization
// ============================================================================

void setupMotorPins() {
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(STEP_PIN2, OUTPUT);
    pinMode(STEP_PIN3, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    digitalWrite(SLEEP_PIN, LOW); // Start with drivers in sleep mode
}

void setupSPI() {
    opu_spi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, -1);
    driver_spi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1);
}

// ============================================================================
// Device Control Functions
// ============================================================================

void resetAllDevices() {
    // Reset and initialize DACs
    dac_f.reset();
    dac_f.write(1<<15); // Set to 0V
    dac_t.reset();
    dac_t.write(1<<15);
    dac_x.reset();
    dac_x.write(1<<15);
    dac_y.reset();
    dac_y.write(1<<15);
    dac_z.reset();
    dac_z.write(1<<15);

    // Reset ADCs
    adc_0.reset();
    adc_1.reset();

    // Update state
    current_afm_state.dac_f_val = 1<<15;
    current_afm_state.dac_t_val = 1<<15;
    current_afm_state.dac_x_val = 1<<15;
    current_afm_state.dac_y_val = 1<<15;
    current_afm_state.dac_z_val = 1<<15;
    current_afm_state.current_state = SystemState::IDLE;
}

void restoreDeviceStates() {
    dac_f.reset();
    dac_f.write(current_afm_state.dac_f_val);
    dac_t.reset();
    dac_t.write(current_afm_state.dac_t_val);
    dac_x.reset();
    dac_x.write(current_afm_state.dac_x_val);
    dac_y.reset();
    dac_y.write(current_afm_state.dac_y_val);
    dac_z.reset();
    dac_z.write(current_afm_state.dac_z_val);
    adc_0.reset();
    adc_1.reset();
}

// ============================================================================
// Motor Control Functions
// ============================================================================

bool startMotorMovement(uint8_t motor_number, int steps, int direction, unsigned int step_delay = 500) {
    if (motor_number < 1 || motor_number > 3) return false;
    if (steps <= 0 || motor_ctrl_state.is_moving || 
        current_afm_state.motors[motor_number - 1].status == MotorStatus::MOVING) {
        return false;
    }

    // Update AFM state
    current_afm_state.motors[motor_number - 1].target_position =
        current_afm_state.motors[motor_number - 1].current_position +
        (direction ? steps : -steps);
    current_afm_state.motors[motor_number - 1].status = MotorStatus::MOVING;
    current_afm_state.motors[motor_number - 1].last_update = millis();
    current_afm_state.motors[motor_number - 1].active = motor_number;

    // Hardware control
    digitalWrite(SLEEP_PIN, HIGH);
    delay(1); // Wake-up time
    digitalWrite(DIR_PIN, direction);
    delayMicroseconds(50);

    // Update control state
    motor_ctrl_state.active_motor = motor_number;
    motor_ctrl_state.remaining_steps = steps;
    motor_ctrl_state.direction = direction;
    motor_ctrl_state.step_delay = step_delay;
    motor_ctrl_state.next_step_time = micros() + step_delay;
    motor_ctrl_state.is_moving = true;
    
    return true;
}

void updateMotorMovement() {
    if (!motor_ctrl_state.is_moving) return;

    unsigned long current_time = micros();
    if (current_time >= motor_ctrl_state.next_step_time) {
        // Calculate steps to take
        unsigned long time_since_last_step = current_time - motor_ctrl_state.next_step_time;
        int steps_to_take = 1 + (time_since_last_step / motor_ctrl_state.step_delay);
        steps_to_take = min(steps_to_take, motor_ctrl_state.remaining_steps);
        
        // Get step pin
        uint8_t step_pin;
        switch (motor_ctrl_state.active_motor) {
            case 1: step_pin = STEP_PIN1; break;
            case 2: step_pin = STEP_PIN2; break;
            case 3: step_pin = STEP_PIN3; break;
            default: return;
        }

        // Pulse step pin
        for (int i = 0; i < steps_to_take; i++) {
            digitalWrite(step_pin, HIGH);
            delayMicroseconds(20);
            digitalWrite(step_pin, LOW);
            delayMicroseconds(20);
        }

        // Update control state
        motor_ctrl_state.remaining_steps -= steps_to_take;
        motor_ctrl_state.next_step_time = current_time + motor_ctrl_state.step_delay;

        // Update AFM state
        uint8_t motor_idx = motor_ctrl_state.active_motor - 1;
        current_afm_state.motors[motor_idx].current_position +=
            (motor_ctrl_state.direction ? steps_to_take : -steps_to_take);
        current_afm_state.motors[motor_idx].last_update = millis();

        // Check completion
        if (motor_ctrl_state.remaining_steps <= 0) {
            motor_ctrl_state.is_moving = false;
            current_afm_state.motors[motor_idx].status = MotorStatus::IDLE;
            digitalWrite(SLEEP_PIN, LOW);
        }
    }
}

bool isMotorMoving() {
    if (motor_ctrl_state.is_moving) return true;
    for (int i = 0; i < 3; i++) {
        if (current_afm_state.motors[i].status == MotorStatus::MOVING) return true;
    }
    return false;
}

bool stopMotor() {
    if (!motor_ctrl_state.is_moving) return false;

    uint8_t motor_idx = motor_ctrl_state.active_motor - 1;
    current_afm_state.motors[motor_idx].status = MotorStatus::IDLE;
    current_afm_state.motors[motor_idx].last_update = millis();

    motor_ctrl_state.is_moving = false;
    motor_ctrl_state.active_motor = 0;
    digitalWrite(SLEEP_PIN, LOW);

    return true;
}

bool moveMotorBlocking(uint8_t motor_number, int steps, int direction, unsigned int step_delay = 500) {
    if (!startMotorMovement(motor_number, steps, direction, step_delay)) return false;
    while (motor_ctrl_state.is_moving) updateMotorMovement();
    return true;
}

// ============================================================================
// ADC Control Functions
// ============================================================================

bool adcInUse() {
    return adc_busy.load();
}

bool acquireADC() {
    bool expected = false;
    return adc_busy.compare_exchange_strong(expected, true);
}

void releaseADC() {
    adc_busy = false;
}

void updateAFMState() {
    if (adcInUse()) return;
    if (acquireADC()) {
        current_afm_state.adc_0_val = adc_0.readADC();
        current_afm_state.adc_1_val = adc_1.readADC();
        current_afm_state.timestamp = millis();
        releaseADC();
    }
}

// ============================================================================
// PID Control Functions
// ============================================================================

void updatePIDControl() {
    if (!current_afm_state.pid_enabled) return;

    uint32_t current_time = millis();
    uint32_t dt = current_time - current_afm_state.pid_last_time;
    if (dt == 0) return;

    // Calculate error
    int32_t error = current_afm_state.pid_target - current_afm_state.adc_0_val;
    if (current_afm_state.pid_invert) error = -error;

    // Calculate PID terms
    float p_term = current_afm_state.pid_kp * error;
    current_afm_state.pid_integral += current_afm_state.pid_ki * error * dt / 1000.0;
    float d_term = current_afm_state.pid_kd * (error - current_afm_state.pid_last_error) / (dt / 1000.0);

    // Calculate output
    float pid_output = p_term + current_afm_state.pid_integral + d_term;
    float desired_output = pid_output + current_afm_state.pid_bias;

    // Apply slew rate limiting
    float max_change = current_afm_state.pid_slew_rate * dt / 1000.0;
    float delta = desired_output - current_afm_state.pid_last_output;
    if (abs(delta) > max_change) {
        delta = (delta > 0) ? max_change : -max_change;
    }
    float output = current_afm_state.pid_last_output + delta;

    // Clamp and update
    output = constrain(output, 0, 65535);
    dac_z.write(static_cast<uint16_t>(output));
    current_afm_state.dac_z_val = static_cast<uint16_t>(output);

    // Update state
    current_afm_state.pid_last_error = error;
    current_afm_state.pid_last_time = current_time;
    current_afm_state.pid_last_output = output;
}

// ============================================================================
// JSON Response Functions
// ============================================================================

JsonDocument getAFMStateAsJson() {
    JsonDocument doc;
    
    // Core measurements
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
    const char* state_str;
    switch (current_afm_state.current_state) {
        case SystemState::IDLE: state_str = "IDLE"; break;
        case SystemState::FOCUSING: state_str = "FOCUSING"; break;
        case SystemState::APPROACHING: state_str = "APPROACHING"; break;
        case SystemState::SCANNING: state_str = "SCANNING"; break;
        default: state_str = "UNKNOWN";
    }
    doc["state"] = state_str;

    // Motor states
    for (int i = 0; i < 3; i++) {
        JsonObject motor = doc[String("motor_") + String(i + 1)].to<JsonObject>();
        motor["position"] = current_afm_state.motors[i].current_position;
        motor["target"] = current_afm_state.motors[i].target_position;
        motor["is_running"] = current_afm_state.motors[i].status == MotorStatus::MOVING;
    }

    return doc;
}

// ============================================================================
// Command Processing
// ============================================================================

String processCommand(const String &json_command) {
    JsonDocument doc;
    JsonDocument response;
    DeserializationError error = deserializeJson(doc, json_command);

    if (error) {
        response["status"] = "error";
        response["message"] = "Invalid JSON";
        String output;
        serializeJson(response, output);
        return output;
    }

    if (!doc["command"].is<String>()) {
        response["status"] = "error";
        response["message"] = "Invalid command format";
        String output;
        serializeJson(response, output);
        return output;
    }

    const char* command = doc["command"];

    // Command processing
    if (strcmp(command, "reset") == 0) {
        resetAllDevices();
        response["status"] = "success";
    }
    else if (strcmp(command, "restore") == 0) {
        restoreDeviceStates();
        response["status"] = "success";
    }
    else if (strcmp(command, "get_status") == 0) {
        response = getAFMStateAsJson();
    }
    else if (strcmp(command, "read_adc") == 0) {
        unsigned long start = millis();
        while (adcInUse() && (millis() - start < 100)) delay(1);

        if (acquireADC()) {
            int adc_0_value = adc_0.readADC();
            delayMicroseconds(10);
            int adc_1_value = adc_1.readADC();
            delayMicroseconds(10);
            releaseADC();

            response["adc_0"] = adc_0_value;
            response["adc_1"] = adc_1_value;
            response["status"] = "success";
        }
        else {
            response["status"] = "error";
            response["message"] = "ADC busy - timeout";
        }
    }
    else if (strcmp(command, "set_dac") == 0) {
        const char* channel = doc["channel"];
        int value = doc["value"];
        bool success = false;

        if (strcmp(channel, "T") == 0) {
            dac_t.write(value);
            current_afm_state.dac_t_val = value;
            success = true;
        }
        else if (strcmp(channel, "F") == 0) {
            dac_f.write(value);
            current_afm_state.dac_f_val = value;
            success = true;
        }
        else if (strcmp(channel, "X") == 0) {
            dac_x.write(value);
            current_afm_state.dac_x_val = value;
            success = true;
        }
        else if (strcmp(channel, "Y") == 0) {
            dac_y.write(value);
            current_afm_state.dac_y_val = value;
            success = true;
        }
        else if (strcmp(channel, "Z") == 0) {
            dac_z.write(value);
            current_afm_state.dac_z_val = value;
            success = true;
        }

        response["status"] = success ? "success" : "error";
        if (!success) response["message"] = "Invalid DAC channel";
    }
    else if (strcmp(command, "start_motor") == 0) {
        const auto &motor_param = doc["motor"];
        const auto &steps_param = doc["steps"];
        const auto &direction_param = doc["direction"];

        if (!motor_param.is<int>() || !steps_param.is<int>() || !direction_param.is<int>()) {
            response["status"] = "error";
            response["message"] = "Missing required parameters (motor, steps, direction)";
        }
        else if (motor_param.as<int>() < 1 || motor_param.as<int>() > 3) {
            response["status"] = "error";
            response["message"] = "Invalid motor number (must be 1-3)";
        }
        else if (steps_param.as<int>() <= 0) {
            response["status"] = "error";
            response["message"] = "Steps must be a positive integer";
        }
        else if (direction_param.as<int>() != 0 && direction_param.as<int>() != 1) {
            response["status"] = "error";
            response["message"] = "Direction must be 0 (CCW) or 1 (CW)";
        }
        else if (isMotorMoving()) {
            response["status"] = "error";
            response["message"] = "Another motor is already moving";
        }
        else {
            unsigned int speed = doc["speed"] | 500;
            speed = constrain(speed, 100, 5000);

            if (startMotorMovement(motor_param.as<int>(), steps_param.as<int>(), 
                                 direction_param.as<int>(), speed)) {
                response["status"] = "started";
                response["motor"] = motor_param.as<int>();
                response["steps"] = steps_param.as<int>();
                response["direction"] = direction_param.as<int>() ? "CW" : "CCW";
                response["speed"] = speed;
                response["target_position"] = current_afm_state.motors[motor_param.as<int>() - 1].target_position;
            }
            else {
                response["status"] = "error";
                response["message"] = "Failed to start motor movement";
            }
        }
    }
    else if (strcmp(command, "stop_motor") == 0) {
        if (stopMotor()) {
            response["status"] = "stopped";
            response["motor"] = motor_ctrl_state.active_motor;
        }
        else {
            response["status"] = "idle";
            response["message"] = "No motor was moving";
        }
    }
    else if (strcmp(command, "pid_control") == 0) {
        if (doc["action"] == "enable") {
            current_afm_state.pid_enabled = true;
            current_afm_state.pid_target = doc["target"] | current_afm_state.adc_0_val;
            current_afm_state.pid_last_time = millis();
            current_afm_state.pid_integral = 0;
            current_afm_state.pid_last_error = 0;
            current_afm_state.pid_last_output = current_afm_state.dac_z_val;
            current_afm_state.pid_bias = current_afm_state.dac_z_val;
            response["status"] = "success";
            response["message"] = "PID control enabled";
        }
        else if (doc["action"] == "disable") {
            current_afm_state.pid_enabled = false;
            response["status"] = "success";
            response["message"] = "PID control disabled";
        }
        else if (doc["action"] == "set_params") {
            current_afm_state.pid_kp = doc["kp"] | current_afm_state.pid_kp;
            current_afm_state.pid_ki = doc["ki"] | current_afm_state.pid_ki;
            current_afm_state.pid_kd = doc["kd"] | current_afm_state.pid_kd;
            current_afm_state.pid_invert = doc["invert"] | current_afm_state.pid_invert;
            current_afm_state.pid_slew_rate = doc["slew_rate"] | current_afm_state.pid_slew_rate;
            current_afm_state.pid_bias = doc["bias"] | current_afm_state.pid_bias;
            response["status"] = "success";
            response["message"] = "PID parameters updated";
        }
        else if (doc["action"] == "get_status") {
            response["enabled"] = current_afm_state.pid_enabled;
            response["target"] = current_afm_state.pid_target;
            response["kp"] = current_afm_state.pid_kp;
            response["ki"] = current_afm_state.pid_ki;
            response["kd"] = current_afm_state.pid_kd;
            response["invert"] = current_afm_state.pid_invert;
            response["slew_rate"] = current_afm_state.pid_slew_rate;
            response["bias"] = current_afm_state.pid_bias;
            response["current_value"] = current_afm_state.adc_0_val;
            response["output"] = current_afm_state.dac_z_val;
            response["status"] = "success";
        }
        else {
            response["status"] = "error";
            response["message"] = "Invalid PID action";
        }
    }
    else {
        response["status"] = "error";
        response["message"] = "Unknown command";
    }

    String output;
    serializeJson(response, output);
    return output;
}

// ============================================================================
// Task Management
// ============================================================================

TaskHandle_t control_task_handle;

void controlTask(void *parameter) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1 ms control loop
    esp_task_wdt_add(NULL);

    while (1) {
        updateMotorMovement();
        updateAFMState();
        updatePIDControl();
        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake_time, period);
    }
}

// ============================================================================
// Main Setup and Loop
// ============================================================================

void setup() {
    // Initialize watchdog
    esp_task_wdt_init(5, true); // 5-second timeout, panic on trigger
    
    // Initialize hardware
    setupSPI();
    setupMotorPins();
    
    // Initialize serial communication
    Serial.setRxBufferSize(1024);
    Serial.begin(115200);
    while (!Serial) delay(1);

    // Create control task
    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        &control_task_handle,
        0
    );
}

void loop() {
    if (Serial.available()) {
        String json_command = Serial.readStringUntil('\n');
        json_command.trim();

        if (json_command.length() > 0) {
            String response = processCommand(json_command);
            Serial.println(response);
        }
    }
}