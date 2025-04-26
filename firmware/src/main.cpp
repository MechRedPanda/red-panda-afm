#include "AD5761.hpp"
#include "ads8681.hpp"
#include <ArduinoJson.h>
#include <atomic>
#include "esp_task_wdt.h"
#include <cstring> // Needed for memcpy

// For thread synchronization
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

// ============================================================================
// PID Controller Class
// ============================================================================

class PIDController
{
public:
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f,
                  float integralMin = -50000.0f, float integralMax = 50000.0f,
                  float slewRate = 1000000.0f)
        : kp_(kp), ki_(ki), kd_(kd), integral_min_(integralMin),
          integral_max_(integralMax), slew_rate_(slewRate)
    {
        reset();
    }

    void setTunings(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutputLimits(float min, float max)
    {
        output_min_ = min;
        output_max_ = max;
    }

    void setIntegralLimits(float min, float max)
    {
        integral_min_ = min;
        integral_max_ = max;
    }

    void setSlewRate(float rate)
    {
        slew_rate_ = rate;
    }

    void reset()
    {
        integral_ = 0;
        last_error_ = 0;
        last_feedback_ = 0;
        last_output_ = 0;
        last_time_ = 0;
        filtered_d_term_ = 0;
    }

    float compute(float setpoint, float feedback)
    {
        uint32_t now = millis();
        uint32_t dt = now - last_time_;
        if (dt == 0)
            return last_output_;

        float error = setpoint - feedback;
        if (invert_)
            error = -error;

        // Proportional term
        float p_term = kp_ * error;

        // Integral term with anti-windup
        float pre_integral = integral_;
        integral_ += ki_ * error * dt / 1000.0f;
        integral_ = constrain(integral_, integral_min_, integral_max_);

        // Back-calculation anti-windup
        if (integral_ != pre_integral)
        {
            float windup = integral_ - pre_integral;
            integral_ = pre_integral + windup * 0.5f; // Reduce windup effect
        }

        // Derivative term on process variable (not error) to reduce kicks
        float d_term_raw = -kd_ * (feedback - last_feedback_) / (dt / 1000.0f);

        // Low-pass filter on derivative term
        filtered_d_term_ = alpha_ * filtered_d_term_ + (1.0f - alpha_) * d_term_raw;
        float d_term = filtered_d_term_;

        // Calculate output with bias
        float pid_output = p_term + integral_ + d_term;
        float desired_output = pid_output + bias_;

        // Slew rate limiting
        float max_change = slew_rate_ * dt / 1000.0f;
        float delta = desired_output - last_output_;
        if (abs(delta) > max_change)
        {
            delta = (delta > 0) ? max_change : -max_change;
        }
        float output = last_output_ + delta;

        // Output limiting
        output = constrain(output, output_min_, output_max_);

        // Update state
        last_error_ = error;
        last_feedback_ = feedback;
        last_output_ = output;
        last_time_ = now;

        return output;
    }

    void setInvert(bool invert) { invert_ = invert; }
    void setBias(float bias) { bias_ = bias; }
    void setAlpha(float alpha) { alpha_ = constrain(alpha, 0.0f, 1.0f); }

    // Getter methods
    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }
    float getIntegral() const { return integral_; }
    float getBias() const { return bias_; }
    float getCurrentOutput() const { return last_output_; }
    bool getInvert() const { return invert_; }

private:
    // PID parameters
    float kp_, ki_, kd_;
    float integral_ = 0;
    float last_error_ = 0;
    float last_feedback_ = 0;
    float last_output_ = 0;
    uint32_t last_time_ = 0;

    // Output limits
    float output_min_ = 0;
    float output_max_ = 65535;

    // Integral limits
    float integral_min_;
    float integral_max_;

    // Slew rate limit
    float slew_rate_;

    // Configuration
    bool invert_ = false;
    float bias_ = 0;

    // Derivative filtering
    float alpha_ = 0.2f; // Filter coefficient (0 < alpha < 1)
    float filtered_d_term_ = 0;
};

// ============================================================================
// Configuration
// ============================================================================
#define DEBUG_ENABLED 0 // Set to 0 to disable all debug output

// Debug output macros - these will compile to nothing when debugging is disabled
#if DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(format, ...) Serial.printf(format, __VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(format, ...)
#endif

// ============================================================================
// Global Constants
// ============================================================================
static const int MAX_APPROACH_POINTS = 1000;
#define MAX_ADC_AVG_WINDOW_SIZE 16 // Maximum ADC average window size

#define DAC_Z_PROBE_TARGET 1000        // Fixed Z DAC value for probing (closer to 0)
#define DAC_Z_RETRACT_TARGET (1 << 15) // Fixed Z DAC value when retracted (mid-scale)
#define Z_CYCLE_STEPS 1000             // Fixed number of steps for Z probe/retract cycle
#define Z_STEP_DELAY_US 1              // Delay between Z DAC steps in approach cycle (us)

// ============================================================================
// Hardware Configuration
// ============================================================================

// Pin Definitions
#define DIR_PIN 27
#define STEP_PIN1 25
#define STEP_PIN2 33
#define STEP_PIN3 32
#define SLEEP_PIN 26

// SPI Pin Definitions
#define VSPI_SCK 18
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define HSPI_SCK 14
#define HSPI_MISO 12
#define HSPI_MOSI 13

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
ADC_ads8681 adc_0 = ADC_ads8681(&opu_spi, 21, 0x0004);
ADC_ads8681 adc_1 = ADC_ads8681(&opu_spi, 5, 0x0003);

// ============================================================================
// State Definitions
// ============================================================================

// System States
enum class SystemState
{
    IDLE = 0,
    FOCUSING = 1,
    APPROACHING = 2,
    SCANNING = 3
};

// Motor States
enum class MotorStatus
{
    IDLE = 0,
    MOVING = 1,
    ERROR = 2
};

// Approach Phase States (for Woodpecker)
enum class ApproachPhase
{
    IDLE = 0,
    READY_TO_STEP = 1,
    STEPPING_MOTOR = 2,
    READY_TO_PROBE = 3,
    Z_PROBING = 4,
    Z_RETRACTING = 5,
    INTERACTION_FOUND = 6,
    FINISHED_MAX_STEPS = 7
};

// Motor State Structure
struct MotorState
{
    int32_t current_position = 0;
    int32_t target_position = 0;
    uint32_t last_update = 0;
    MotorStatus status = MotorStatus::IDLE;
    uint8_t active = 0; // 0 = inactive, 1-3 = motor number
};

// AFM System State Structure
struct AFMState
{
    // DAC/ADC Values - Default DAC values set to mid-scale (0V)
    uint16_t dac_f_val = (1 << 15);
    uint16_t dac_t_val = (1 << 15);
    uint16_t dac_x_val = (1 << 15);
    uint16_t dac_y_val = (1 << 15);
    uint16_t dac_z_val = (1 << 15);
    uint16_t adc_0_val = 0;
    uint16_t adc_1_val = 0;

    // ADC Moving Average
    float adc_0_avg = 0.0f;
    float adc_1_avg = 0.0f;
    uint16_t adc_0_history[MAX_ADC_AVG_WINDOW_SIZE]; // Use MAX size
    uint16_t adc_1_history[MAX_ADC_AVG_WINDOW_SIZE]; // Use MAX size
    uint8_t adc_history_index = 0;
    uint8_t adc_history_count = 0;
    uint8_t adc_avg_window_size = 3; // Default window size, configurable

    // Motor States
    MotorState motors[3]; // Index 0-2 for motors 1-3

    // System State
    int32_t timestamp = 0;
    SystemState current_state = SystemState::IDLE;

    // PID Control
    bool pid_enabled = false;
    float pid_target = 0.0f;
    PIDController pid_controller; // Using default constructor values

    // Approach Data
    struct ApproachData
    {
        int32_t steps;
        float adc;
        int32_t position;
    };
    ApproachData approach_data[MAX_APPROACH_POINTS];
    int approach_data_count = 0;
    bool approach_running = false;
    uint8_t approach_motor = 0;
    int approach_step_size = 0;
    int approach_direction = 0;
    uint16_t approach_threshold = 0;
    float approach_initial_adc = 0.0f;
    int32_t approach_initial_position = 0;
    int32_t approach_max_steps = 0;
    unsigned long approach_polling_interval = 50; // ms - NOTE: No longer used for blocking delay
    unsigned long approach_last_update = 0;
    int32_t approach_last_stored_step = 0;  // Track last stored step
    unsigned int approach_step_delay = 500; // Default step delay in microseconds (renamed from speed)
    int32_t approach_total_steps_taken = 0; // Track total steps taken
    // Z-cycle specific state
    // uint16_t dac_z_probe_target = DAC_Z_PROBE_TARGET; // REMOVED - Target Z DAC value for probing (closer to 0)
    // uint16_t dac_z_retract_target = DAC_Z_RETRACT_TARGET; // REMOVED - Target Z DAC value when retracted (mid-scale)
    // int z_cycle_steps = 100; // REMOVED - Number of steps for Z probe/retract cycle

    // Scan Data & State (using Ring Buffer)
    struct ScanPoint
    {
        uint16_t x;
        uint16_t y;
        uint16_t adc0;
        uint16_t dacz;
        uint8_t flags; // Store flags with each point rather than computing at send time
    };
    static const int MAX_SCAN_POINTS = 256; // Size of the ring buffer
    ScanPoint scan_data[MAX_SCAN_POINTS];
    uint16_t scan_buffer_head = 0;  // Index where next point will be written
    uint16_t scan_buffer_tail = 0;  // Index from where next point will be sent
    uint16_t scan_buffer_count = 0; // Number of points currently in the buffer
    bool scan_active = false;
    uint16_t scan_x_start = 0;
    uint16_t scan_x_end = 0;
    uint16_t scan_y_start = 0;
    uint16_t scan_y_end = 0;
    uint16_t scan_resolution = 256;
    uint16_t scan_current_x_index = 0;
    uint16_t scan_current_y_index = 0;
    uint16_t scan_current_x = 0;
    uint16_t scan_current_y = 0;
    bool scan_y_forward = true;    // Direction for serpentine scan
    bool scan_return_line = false; // True when scanning the return (second) line for an X position
    unsigned long scan_last_point_time = 0;
    unsigned int scan_dwell_time = 1; // ms between points

    // Control Loop Timing - Switched to Period/Frequency Measurement
    uint32_t control_loop_last_period_us = 0;  // Period of the last full loop iteration
    uint64_t control_loop_total_period_us = 0; // Use 64-bit to prevent quick overflow
    uint32_t control_loop_count = 0;           // Count for averaging period

    // DAC XY Ramping State
    bool ramp_active = false;
    uint16_t ramp_target_x = 0;
    uint16_t ramp_target_y = 0;
    uint16_t ramp_target_z = 0; // Added Z ramp target
    uint16_t ramp_start_x = 0;
    uint16_t ramp_start_y = 0;
    uint16_t ramp_start_z = 0; // Added Z ramp start
    int32_t ramp_delta_x = 0;
    int32_t ramp_delta_y = 0;
    int32_t ramp_delta_z = 0;    // Added Z ramp delta
    int ramp_duration_ms = 1000; // Default duration
    int ramp_step_delay_ms = 1;  // Default step delay
    int ramp_total_steps = 0;
    int ramp_current_step = 0;
    uint64_t ramp_next_step_time_us = 0; // Use microsecond timer
};

// ============================================================================
// Global Variables
// ============================================================================

AFMState current_afm_state;
std::atomic<bool> adc_busy(false);
std::atomic<bool> scan_data_ready_to_send; // Flag for push mechanism (initialized in setup)
const int SCAN_SEND_CHUNK_SIZE = 32;       // Number of points to send per loop iteration

// Add mutex for scan buffer protection
SemaphoreHandle_t scan_buffer_mutex = NULL;

// Motor Control State
struct MotorControlState
{
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

void setupMotorPins()
{
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN1, OUTPUT);
    pinMode(STEP_PIN2, OUTPUT);
    pinMode(STEP_PIN3, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    digitalWrite(SLEEP_PIN, LOW); // Start with drivers in sleep mode
}

void setupSPI()
{
    opu_spi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, -1);
    driver_spi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1);
}

// ============================================================================
// Device Control Functions
// ============================================================================

void resetAllDevices()
{
    // Reset and initialize DACs
    dac_f.reset();
    dac_f.write(1 << 15); // Set to 0V
    dac_t.reset();
    dac_t.write(1 << 15);
    dac_x.reset();
    dac_x.write(1 << 15);
    dac_y.reset();
    dac_y.write(1 << 15);
    dac_z.reset();
    dac_z.write(1 << 15);

    // Reset ADCs
    adc_0.reset();
    adc_1.reset();

    // Update state
    current_afm_state.dac_f_val = 1 << 15;
    current_afm_state.dac_t_val = 1 << 15;
    current_afm_state.dac_x_val = 1 << 15;
    current_afm_state.dac_y_val = 1 << 15;
    current_afm_state.dac_z_val = 1 << 15;
    current_afm_state.current_state = SystemState::IDLE;
}

void restoreDeviceStates()
{
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

bool startMotorMovement(uint8_t motor_number, int steps, int direction, unsigned int step_delay = 500)
{
    if (motor_number < 1 || motor_number > 3)
        return false;
    if (steps <= 0 || motor_ctrl_state.is_moving ||
        current_afm_state.motors[motor_number - 1].status == MotorStatus::MOVING)
    {
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

void updateMotorMovement()
{
    if (!motor_ctrl_state.is_moving)
        return;

    unsigned long current_time = micros();
    if (current_time >= motor_ctrl_state.next_step_time)
    {
        // Calculate steps to take
        unsigned long time_since_last_step = current_time - motor_ctrl_state.next_step_time;
        int steps_to_take = 1 + (time_since_last_step / motor_ctrl_state.step_delay);
        steps_to_take = min(steps_to_take, motor_ctrl_state.remaining_steps);

        // Get step pin
        uint8_t step_pin;
        switch (motor_ctrl_state.active_motor)
        {
        case 1:
            step_pin = STEP_PIN1;
            break;
        case 2:
            step_pin = STEP_PIN2;
            break;
        case 3:
            step_pin = STEP_PIN3;
            break;
        default:
            return;
        }

        // Pulse step pin
        for (int i = 0; i < steps_to_take; i++)
        {
            digitalWrite(step_pin, HIGH);
            delayMicroseconds(10);
            digitalWrite(step_pin, LOW);
            delayMicroseconds(10);
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
        if (motor_ctrl_state.remaining_steps <= 0)
        {
            motor_ctrl_state.is_moving = false;
            current_afm_state.motors[motor_idx].status = MotorStatus::IDLE;
            digitalWrite(SLEEP_PIN, LOW);
        }
    }
}

bool isMotorMoving()
{
    if (motor_ctrl_state.is_moving)
        return true;
    for (int i = 0; i < 3; i++)
    {
        if (current_afm_state.motors[i].status == MotorStatus::MOVING)
            return true;
    }
    return false;
}

bool stopMotor()
{
    if (!motor_ctrl_state.is_moving)
        return false;

    uint8_t motor_idx = motor_ctrl_state.active_motor - 1;
    current_afm_state.motors[motor_idx].status = MotorStatus::IDLE;
    current_afm_state.motors[motor_idx].last_update = millis();

    motor_ctrl_state.is_moving = false;
    motor_ctrl_state.active_motor = 0;
    digitalWrite(SLEEP_PIN, LOW);

    return true;
}

bool moveMotorBlocking(uint8_t motor_number, int steps, int direction, unsigned int step_delay = 500)
{
    if (!startMotorMovement(motor_number, steps, direction, step_delay))
        return false;
    while (motor_ctrl_state.is_moving)
        updateMotorMovement();
    return true;
}

// ============================================================================
// ADC Control Functions
// ============================================================================

bool adcInUse()
{
    return adc_busy.load();
}

bool acquireADC()
{
    bool expected = false;
    return adc_busy.compare_exchange_strong(expected, true);
}

void releaseADC()
{
    adc_busy = false;
}

void updateAFMState()
{
    if (adcInUse())
        return;
    if (acquireADC())
    {
        delayMicroseconds(1);
        uint16_t new_adc0 = adc_0.readADC();
        delayMicroseconds(1);
        uint16_t new_adc1 = adc_1.readADC();
        delayMicroseconds(1);
        current_afm_state.timestamp = millis();
        releaseADC();

        // Update raw ADC values
        current_afm_state.adc_0_val = new_adc0;
        current_afm_state.adc_1_val = new_adc1;

        // Update moving average
        // Store new value in history buffer
        current_afm_state.adc_0_history[current_afm_state.adc_history_index] = new_adc0;
        current_afm_state.adc_1_history[current_afm_state.adc_history_index] = new_adc1;

        // Increment index and wrap around using the current window size
        current_afm_state.adc_history_index = (current_afm_state.adc_history_index + 1) % current_afm_state.adc_avg_window_size;

        // Increment count until buffer reaches the current window size
        if (current_afm_state.adc_history_count < current_afm_state.adc_avg_window_size)
        {
            current_afm_state.adc_history_count++;
        }

        // Calculate sum
        uint32_t sum0 = 0;
        uint32_t sum1 = 0;
        for (uint8_t i = 0; i < current_afm_state.adc_history_count; i++)
        {
            sum0 += current_afm_state.adc_0_history[i];
            sum1 += current_afm_state.adc_1_history[i];
        }

        // Calculate and store average (avoid division by zero)
        if (current_afm_state.adc_history_count > 0)
        {
            current_afm_state.adc_0_avg = (float)sum0 / current_afm_state.adc_history_count;
            current_afm_state.adc_1_avg = (float)sum1 / current_afm_state.adc_history_count;
        }
        else
        {
            current_afm_state.adc_0_avg = 0.0f;
            current_afm_state.adc_1_avg = 0.0f;
        }
    }
}

// ============================================================================
// PID Control Functions
// ============================================================================

void updatePIDControl()
{
    if (!current_afm_state.pid_enabled)
        return;

    float output = current_afm_state.pid_controller.compute(
        current_afm_state.pid_target,
        current_afm_state.adc_0_avg);

    dac_z.write(static_cast<uint16_t>(output));
    current_afm_state.dac_z_val = static_cast<uint16_t>(output);
}

// ============================================================================
// Approach Control Functions
// ============================================================================

// Non-blocking approach initialization function
bool startApproach(uint8_t motor, int step_size, uint16_t threshold, int32_t max_steps, unsigned int delay_val = 500)
{
    if (motor < 1 || motor > 3 || step_size == 0 || threshold <= 0 || max_steps == 0)
        return false; // Basic validation
    if (isMotorMoving() || current_afm_state.approach_running || current_afm_state.scan_active)
        return false; // Prevent starting if busy

    // --- Initialize Approach State ---
    current_afm_state.current_state = SystemState::APPROACHING;
    current_afm_state.approach_running = true; // Set flag to true to enable updateApproach logic
    current_afm_state.approach_motor = motor;
    current_afm_state.approach_step_size = abs(step_size);
    current_afm_state.approach_direction = max_steps > 0 ? 1 : 0;
    current_afm_state.approach_threshold = threshold;
    current_afm_state.approach_max_steps = abs(max_steps);
    current_afm_state.approach_step_delay = delay_val;
    current_afm_state.approach_total_steps_taken = 0;
    current_afm_state.approach_data_count = 0; // Clear previous data
    current_afm_state.ramp_active = false;     // Ensure Z ramp isn't active initially

    // Get initial ADC value (update state first)
    updateAFMState();
    current_afm_state.approach_initial_adc = current_afm_state.adc_0_avg;
    current_afm_state.approach_initial_position = current_afm_state.motors[motor - 1].current_position;

    // Set initial Z position to retracted state
    dac_z.write(DAC_Z_RETRACT_TARGET);
    current_afm_state.dac_z_val = DAC_Z_RETRACT_TARGET;

    // Log initial data point
    if (current_afm_state.approach_data_count < MAX_APPROACH_POINTS)
    {
        current_afm_state.approach_data[current_afm_state.approach_data_count++] = {
            current_afm_state.approach_total_steps_taken,
            current_afm_state.approach_initial_adc,
            current_afm_state.approach_initial_position};
    }

    // --- Removed blocking loop ---
    // bool threshold_met = false;
    // while (current_afm_state.approach_running) { ... loop content removed ... }
    // --- Removed Cleanup ---
    // current_afm_state.approach_running = false;
    // current_afm_state.current_state = SystemState::IDLE;
    // return threshold_met;

    return true; // Indicate successful initialization
}

void stopApproach()
{
    if (current_afm_state.approach_running)
    {
        stopMotor(); // Stop motor if it was moving
        current_afm_state.approach_running = false;

        // Only change system state if it was APPROACHING
        if (current_afm_state.current_state == SystemState::APPROACHING)
        {
            current_afm_state.current_state = SystemState::IDLE;
        }
    }
}

// updateApproach performs one blocking step cycle including Z-probe/retract
void updateApproach()
{
    if (!current_afm_state.approach_running)
        return; // Not active, do nothing

    // --- Preliminary Checks ---
    if (current_afm_state.approach_total_steps_taken >= current_afm_state.approach_max_steps)
    {
        stopApproach(); // Max steps reached
        return;
    }
    // Check initial threshold before moving (in case already engaged)
    updateAFMState(); // Get latest ADC
    if (abs(current_afm_state.adc_0_avg - current_afm_state.approach_initial_adc) >= current_afm_state.approach_threshold)
    {
        stopApproach(); // Already past threshold
        return;
    }

    // --- Perform Blocking Motor Step ---
    bool motor_success = moveMotorBlocking(
        current_afm_state.approach_motor,
        current_afm_state.approach_step_size,
        current_afm_state.approach_direction,
        current_afm_state.approach_step_delay);

    if (!motor_success)
    {
        stopApproach(); // Failed to move motor
        return;
    }

    // --- Blocking Settle Delay ---
    delay(current_afm_state.approach_polling_interval);

    // --- Blocking Z-Probe Cycle ---
    bool threshold_met_during_probe = false;
    uint16_t start_z = DAC_Z_RETRACT_TARGET;
    uint16_t target_z = DAC_Z_PROBE_TARGET;
    int32_t delta_z = (int32_t)target_z - (int32_t)start_z;
    int total_steps = Z_CYCLE_STEPS;
    if (total_steps <= 0)
        total_steps = 1;
    uint64_t next_step_time_us = esp_timer_get_time();

    for (int i = 1; i <= total_steps; ++i)
    {
        // Wait for step delay using microseconds
        delayMicroseconds(Z_STEP_DELAY_US);

        // Calculate next Z
        int64_t intermediate_z = (int64_t)start_z + ((int64_t)delta_z * i) / total_steps;
        if (intermediate_z < 0)
            intermediate_z = 0;
        if (intermediate_z > 65535)
            intermediate_z = 65535;
        uint16_t next_z = (uint16_t)intermediate_z;

        // Write Z DAC
        dac_z.write(next_z);
        current_afm_state.dac_z_val = next_z;

        // Update ADC State & Check Threshold
        updateAFMState();
        if (abs(current_afm_state.adc_0_avg - current_afm_state.approach_initial_adc) >= current_afm_state.approach_threshold)
        {
            threshold_met_during_probe = true;
            break; // Exit Z-probe loop
        }
    }
    // Ensure final target is reached if loop completed normally
    if (!threshold_met_during_probe && current_afm_state.dac_z_val != target_z)
    {
        dac_z.write(target_z);
        current_afm_state.dac_z_val = target_z;
        updateAFMState(); // Update state one last time at the probe target
        // Check threshold one last time at the very end of probe travel
        if (abs(current_afm_state.adc_0_avg - current_afm_state.approach_initial_adc) >= current_afm_state.approach_threshold)
        {
            threshold_met_during_probe = true;
        }
    }

    // --- Handle Probe Result ---
    if (threshold_met_during_probe)
    {
        // Log final data point where threshold was met
        if (current_afm_state.approach_data_count < MAX_APPROACH_POINTS)
        {
            current_afm_state.approach_data[current_afm_state.approach_data_count++] = {
                current_afm_state.approach_total_steps_taken + current_afm_state.approach_step_size, // Log steps *including* this one
                current_afm_state.adc_0_avg,
                current_afm_state.motors[current_afm_state.approach_motor - 1].current_position};
        }
        stopApproach(); // Threshold met
        return;
    }
    else
    {
        // --- Blocking Z-Retract Cycle ---
        start_z = DAC_Z_PROBE_TARGET; // Start from probe position
        target_z = DAC_Z_RETRACT_TARGET;
        delta_z = (int32_t)target_z - (int32_t)start_z;
        // Use same duration/delay for retract
        next_step_time_us = esp_timer_get_time();

        for (int i = 1; i <= total_steps; ++i) // Use the same total_steps
        {
            delayMicroseconds(Z_STEP_DELAY_US);

            int64_t intermediate_z = (int64_t)start_z + ((int64_t)delta_z * i) / total_steps;
            if (intermediate_z < 0)
                intermediate_z = 0;
            if (intermediate_z > 65535)
                intermediate_z = 65535;
            uint16_t next_z = (uint16_t)intermediate_z;

            dac_z.write(next_z);
            current_afm_state.dac_z_val = next_z;
        }
        // Ensure final retract target is reached
        if (current_afm_state.dac_z_val != target_z)
        {
            dac_z.write(target_z);
            current_afm_state.dac_z_val = target_z;
        }

        // --- Log Data & Increment Step Count (after successful Z cycle) ---
        current_afm_state.approach_total_steps_taken += current_afm_state.approach_step_size;
        if (current_afm_state.approach_data_count < MAX_APPROACH_POINTS)
        {
            // Log the ADC value measured at the *probe target* before retracting
            current_afm_state.approach_data[current_afm_state.approach_data_count++] = {
                current_afm_state.approach_total_steps_taken,
                current_afm_state.adc_0_avg, // This holds the value from the end of the probe
                current_afm_state.motors[current_afm_state.approach_motor - 1].current_position};
        }
        else
        {
            stopApproach(); // Buffer full
            return;
        }

        // --- Final Check after Step Cycle ---
        if (current_afm_state.approach_total_steps_taken >= current_afm_state.approach_max_steps)
        {
            stopApproach(); // Max steps reached
            return;
        }
    }
}

// ============================================================================
// Scan Control Functions
// ============================================================================

void stopScan()
{
    DEBUG_PRINTLN("# DEBUG: Inside stopScan()");
    if (current_afm_state.scan_active)
    {
        DEBUG_PRINTLN("# DEBUG: Setting scan_active to false");
        current_afm_state.scan_active = false;
        if (current_afm_state.current_state == SystemState::SCANNING)
        {
            DEBUG_PRINTLN("# DEBUG: Setting state to IDLE");
            current_afm_state.current_state = SystemState::IDLE;
        }
        // Signal to send any remaining data in the buffer
        if (current_afm_state.scan_buffer_count > 0)
        { // Check buffer count
            DEBUG_PRINTLN("# DEBUG: Setting scan_data_ready_to_send to true");
            scan_data_ready_to_send = true;
        }

        // Send explicit JSON command indicating scan completion
        // This will be recognized as a command by the PC, not just a comment
        JsonDocument scanCompleteDoc;
        scanCompleteDoc["command"] = "scan";
        scanCompleteDoc["status"] = "complete";
        scanCompleteDoc["message"] = "Scan has been completed";
        scanCompleteDoc["timestamp"] = millis();
        String output;
        serializeJson(scanCompleteDoc, output);
        Serial.println(output);
    }
    else
    {
        DEBUG_PRINTLN("# DEBUG: scan_active was already false!");
    }
}

// Helper function to calculate DAC value based on index and range
// Avoids floating point for potentially better performance/determinism
uint16_t calculateDacValue(uint16_t start, uint16_t end, uint16_t index, uint16_t resolution)
{
    if (resolution <= 1)
    {
        return start; // Or perhaps average of start/end?
    }
    // Use integer math for calculation: start + (range * index) / (resolution - 1)
    // Add half of the divisor for rounding: (range * index + (resolution - 1) / 2) / (resolution - 1)
    uint32_t range = (end > start) ? (end - start) : (start - end); // Handle reversed range if needed, though startScan validates start <= end
    uint32_t scaled_index = range * index;
    uint32_t divisor = resolution - 1;
    uint32_t rounded_scaled_index = scaled_index + divisor / 2;
    uint16_t offset = rounded_scaled_index / divisor;

    return start + offset;
}

// Forward declaration for the helper function
void sendScanDataBuffer();

// Function to update DAC ramping state machine
void updateDacRamp()
{
    if (!current_afm_state.ramp_active)
    {
        return;
    }

    // Check if this ramp is for Z (handled by updateZramp in approach)
    // Add a simple check: if delta_z is non-zero, it's likely a Z ramp.
    // A more robust method might involve a dedicated flag.
    if (current_afm_state.ramp_delta_z != 0)
    {
        // This assumes ramps are either XY or Z, not XYZ.
        // If Z is ramping, let updateZramp handle it during approach.
        if (current_afm_state.current_state == SystemState::APPROACHING)
        {
            return;
        }
        // If not approaching, but Z ramp is active (e.g., manual Z move command needed)
        // then call updateZramp or integrate Z logic here. For now, we assume
        // non-approach ramps are XY only.
        // If non-approach Z ramps are needed, integrate updateZramp logic here.
    }

    // Proceed with XY ramp logic (only if ramp_delta_z is zero or not approaching)
    uint64_t current_time_us = esp_timer_get_time();

    if (current_time_us >= current_afm_state.ramp_next_step_time_us)
    {
        // Increment step
        current_afm_state.ramp_current_step++;

        uint16_t next_x;
        uint16_t next_y;

        // Check if ramp is complete
        if (current_afm_state.ramp_current_step >= current_afm_state.ramp_total_steps)
        {
            next_x = current_afm_state.ramp_target_x;
            next_y = current_afm_state.ramp_target_y;
            current_afm_state.ramp_active = false; // Mark ramp as complete
        }
        else
        {
            // Calculate intermediate values using integer math
            next_x = current_afm_state.ramp_start_x + ((int64_t)current_afm_state.ramp_delta_x * current_afm_state.ramp_current_step) / current_afm_state.ramp_total_steps;
            next_y = current_afm_state.ramp_start_y + ((int64_t)current_afm_state.ramp_delta_y * current_afm_state.ramp_current_step) / current_afm_state.ramp_total_steps;
        }

        // Write values if changed
        bool xy_changed = false;
        if (next_x != current_afm_state.dac_x_val)
        {
            dac_x.write(next_x);
            current_afm_state.dac_x_val = next_x;
            xy_changed = true;
        }
        if (next_y != current_afm_state.dac_y_val)
        {
            dac_y.write(next_y);
            current_afm_state.dac_y_val = next_y;
            xy_changed = true;
        }

        // If ramp is still active, schedule next step
        if (current_afm_state.ramp_active)
        {
            current_afm_state.ramp_next_step_time_us = current_time_us + (uint64_t)current_afm_state.ramp_step_delay_ms * 1000;
        }
    }
}

bool startScan(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end,
               uint16_t resolution, unsigned int dwell_time)
{
    // Basic validation
    if (resolution < 2)
        return false;
    if (x_start > x_end || y_start > y_end)
        return false;
    // Prevent starting if already scanning, moving, approaching, OR RAMPING
    if (current_afm_state.scan_active || isMotorMoving() || current_afm_state.approach_running || current_afm_state.ramp_active)
        return false;

    // Store parameters
    current_afm_state.scan_x_start = x_start;
    current_afm_state.scan_x_end = x_end;
    current_afm_state.scan_y_start = y_start;
    current_afm_state.scan_y_end = y_end;
    current_afm_state.scan_resolution = resolution;
    current_afm_state.scan_dwell_time = dwell_time;

    // Initialize scan state variables (indices, direction, etc.)
    current_afm_state.scan_current_x_index = 0;
    current_afm_state.scan_current_y_index = 0;
    current_afm_state.scan_y_forward = true;
    current_afm_state.scan_return_line = false;
    current_afm_state.scan_buffer_head = 0;
    current_afm_state.scan_buffer_tail = 0;
    current_afm_state.scan_buffer_count = 0;
    current_afm_state.scan_last_point_time = millis(); // Use millis for dwell time check

    // Calculate target start position
    uint16_t target_x = calculateDacValue(x_start, x_end, 0, resolution);
    uint16_t target_y = calculateDacValue(y_start, y_end, 0, resolution);

    // Get current position
    uint16_t current_x = current_afm_state.dac_x_val;
    uint16_t current_y = current_afm_state.dac_y_val;

    // --- Initiate Ramp if needed ---
    if (current_x != target_x || current_y != target_y)
    {
        current_afm_state.ramp_target_x = target_x;
        current_afm_state.ramp_target_y = target_y;
        current_afm_state.ramp_start_x = current_x;
        current_afm_state.ramp_start_y = current_y;
        current_afm_state.ramp_delta_x = target_x - current_x;
        current_afm_state.ramp_delta_y = target_y - current_y;
        // Use default duration/step for now, could be made parameters later
        current_afm_state.ramp_duration_ms = 1000;
        current_afm_state.ramp_step_delay_ms = 1;
        if (current_afm_state.ramp_step_delay_ms < 1)
            current_afm_state.ramp_step_delay_ms = 1; // Sanity check
        current_afm_state.ramp_total_steps = current_afm_state.ramp_duration_ms / current_afm_state.ramp_step_delay_ms;

        if (current_afm_state.ramp_total_steps > 0)
        {
            current_afm_state.ramp_current_step = 0;
            current_afm_state.ramp_next_step_time_us = esp_timer_get_time() + (uint64_t)current_afm_state.ramp_step_delay_ms * 1000;
            current_afm_state.ramp_active = true;
        }
        else
        {
            // Duration or step too small, just jump
            dac_x.write(target_x);
            dac_y.write(target_y);
            current_afm_state.dac_x_val = target_x;
            current_afm_state.dac_y_val = target_y;
            current_afm_state.ramp_active = false;
        }
    }
    else
    {
        // Already at the target, no ramp needed
        current_afm_state.ramp_active = false;
        // Ensure state is correct even if no ramp occurred
        current_afm_state.dac_x_val = target_x;
        current_afm_state.dac_y_val = target_y;
    }

    // Update scan state variable *after* initiating ramp or jumping
    current_afm_state.scan_current_x = target_x; // Scan logic uses this as the target/current scan position
    current_afm_state.scan_current_y = target_y;

    // Mark scan as active (even if ramping)
    current_afm_state.scan_active = true;
    current_afm_state.current_state = SystemState::SCANNING;

    // NOTE: No delay here anymore, scan starts logically,
    // but actual movement might be delayed by ramp.
    // First point acquisition in updateScan will wait for dwell time anyway.

    return true;
}

void updateScan()
{
    if (!current_afm_state.scan_active)
        return;

    if (current_afm_state.ramp_active)
    {
        return;
    }

    unsigned long current_time = millis();
    if (current_time - current_afm_state.scan_last_point_time < current_afm_state.scan_dwell_time)
    {
        return; // Wait for dwell time
    }

    // Calculate flags for this point
    uint8_t point_flags = 0;
    if (current_afm_state.scan_y_forward)
        point_flags |= 0x01;
    if (current_afm_state.scan_return_line)
        point_flags |= 0x02;

    // Prepare the data point before acquiring the mutex
    AFMState::ScanPoint new_point = {
        current_afm_state.scan_current_x,
        current_afm_state.scan_current_y,
        current_afm_state.adc_0_val,
        current_afm_state.dac_z_val,
        point_flags};

    // --- Ring Buffer Write Logic with Mutex Protection ---
    // Take mutex with short timeout
    if (xSemaphoreTake(scan_buffer_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        // Critical section - protected by mutex

        // Store data at the current head position
        current_afm_state.scan_data[current_afm_state.scan_buffer_head] = new_point;

        // Calculate next head position
        uint16_t next_head = (current_afm_state.scan_buffer_head + 1) % current_afm_state.MAX_SCAN_POINTS;

        // Check if buffer is full (about to overwrite tail)
        if (current_afm_state.scan_buffer_count == current_afm_state.MAX_SCAN_POINTS)
        {
            // Overwrite oldest data: advance tail
            current_afm_state.scan_buffer_tail = (current_afm_state.scan_buffer_tail + 1) % current_afm_state.MAX_SCAN_POINTS;
            // Count remains MAX_SCAN_POINTS
        }
        else
        {
            // Buffer not full: increment count
            current_afm_state.scan_buffer_count++;
        }

        // Advance head
        current_afm_state.scan_buffer_head = next_head;

        // Signal that data is available (if buffer not empty)
        if (current_afm_state.scan_buffer_count > 0)
        {
            scan_data_ready_to_send = true;
        }

        // Release mutex
        xSemaphoreGive(scan_buffer_mutex);
    }
    else
    {
        // Could not obtain mutex - skip storing this point (better than deadlock)
    }
    // End of critical section
    // --- End Ring Buffer Write Logic ---

    // Calculate next position index
    uint16_t next_x_index = current_afm_state.scan_current_x_index;
    uint16_t next_y_index = current_afm_state.scan_current_y_index;
    bool line_finished = false;
    bool scan_finished = false; // Flag to check if the whole scan ended

    if (current_afm_state.scan_y_forward)
    {
        // Moving forward (increasing Y index)
        if (next_y_index + 1 >= current_afm_state.scan_resolution)
        {
            next_y_index = current_afm_state.scan_resolution - 1; // Clamp to last index
            line_finished = true;
            // Debug: Line ended at forward limit
            DEBUG_PRINT("# DEBUG: Line finished at Y forward limit, X=");
            DEBUG_PRINT(next_x_index);
            DEBUG_PRINT(", return_line=");
            DEBUG_PRINTLN(current_afm_state.scan_return_line ? "true" : "false");
        }
        else
        {
            next_y_index++;
        }
    }
    else
    {
        // Moving backward (decreasing Y index)
        if (next_y_index == 0)
        {
            // next_y_index remains 0
            line_finished = true;
            // Debug: Line ended at backward limit
            DEBUG_PRINT("# DEBUG: Line finished at Y backward limit, X=");
            DEBUG_PRINT(next_x_index);
            DEBUG_PRINT(", return_line=");
            DEBUG_PRINTLN(current_afm_state.scan_return_line ? "true" : "false");
        }
        else
        {
            next_y_index--;
        }
    }

    if (line_finished)
    {
        // Check if we need to do a return line at the same X position
        if (!current_afm_state.scan_return_line)
        {
            // We just finished the first scan direction, now do the return line
            current_afm_state.scan_return_line = true;
            current_afm_state.scan_y_forward = false; // Always go backward for retrace
            // next_y_index is already at the right end position for the return line
            DEBUG_PRINT("# DEBUG: Starting return line at X=");
            DEBUG_PRINTLN(next_x_index);
        }
        else
        {
            // We finished both scan directions for this X position, move to next X
            current_afm_state.scan_return_line = false; // Reset for the new X position
            current_afm_state.scan_y_forward = true;    // Always reset to forward for new X position

            // Check if we've finished the entire scan (all X positions)
            if (next_x_index + 1 >= current_afm_state.scan_resolution)
            {
                scan_finished = true;
                DEBUG_PRINTLN("# DEBUG: Scan complete! Setting scan_finished=true");
            }
            else
            {
                next_x_index++; // Move to next X position
                DEBUG_PRINT("# DEBUG: Moving to next X position: ");
                DEBUG_PRINTLN(next_x_index);
            }
        }
    }

    // Check if the entire scan is finished AFTER processing the last point
    if (scan_finished)
    {
        DEBUG_PRINTLN("# DEBUG: Calling stopScan()");
        // stopScan() will handle flagging remaining data if needed
        stopScan(); // Now stop the scan formally
        DEBUG_PRINT("# DEBUG: After stopScan(), scan_active=");
        DEBUG_PRINTLN(current_afm_state.scan_active ? "true" : "false");
        return; // Exit updateScan as scan is complete
    }

    // Update indices in state
    current_afm_state.scan_current_x_index = next_x_index;
    current_afm_state.scan_current_y_index = next_y_index;

    // Calculate new DAC target values based on updated indices
    uint16_t next_x_dac = calculateDacValue(current_afm_state.scan_x_start, current_afm_state.scan_x_end,
                                            current_afm_state.scan_current_x_index, current_afm_state.scan_resolution);
    uint16_t next_y_dac = calculateDacValue(current_afm_state.scan_y_start, current_afm_state.scan_y_end,
                                            current_afm_state.scan_current_y_index, current_afm_state.scan_resolution);

    // Check if position actually changed before writing DACs (optimization)
    bool position_changed = (next_x_dac != current_afm_state.scan_current_x) || (next_y_dac != current_afm_state.scan_current_y);

    // Update current position state
    current_afm_state.scan_current_x = next_x_dac;
    current_afm_state.scan_current_y = next_y_dac;

    // Update DACs if position changed
    if (position_changed)
    {
        dac_x.write(current_afm_state.scan_current_x);
        dac_y.write(current_afm_state.scan_current_y);
        // Update state dac values immediately after writing
        current_afm_state.dac_x_val = current_afm_state.scan_current_x;
        current_afm_state.dac_y_val = current_afm_state.scan_current_y;
    }

    current_afm_state.scan_last_point_time = current_time; // Update time for the *start* of the next dwell
}

// Modify sendScanDataBuffer to send CSV data
void sendScanDataBuffer()
{
    // Check if the flag is set (indicates potential data)
    if (!scan_data_ready_to_send.load())
    {
        return;
    }

    // Local variables for data to be sent outside the critical section
    int points_to_send = 0;
    // Create a temporary buffer to hold points to send
    // This avoids holding the mutex while formatting/sending strings
    AFMState::ScanPoint points_buffer[SCAN_SEND_CHUNK_SIZE];
    bool is_final_data = false;

    // --- Critical section: Copy data from ring buffer ---
    if (xSemaphoreTake(scan_buffer_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        // Check actual count in the buffer
        if (current_afm_state.scan_buffer_count == 0)
        {
            scan_data_ready_to_send = false;   // Reset flag
            xSemaphoreGive(scan_buffer_mutex); // Release mutex
            return;
        }

        // Calculate how many points to send in this chunk
        points_to_send = min((int)current_afm_state.scan_buffer_count, SCAN_SEND_CHUNK_SIZE);

        // Copy the points to the temporary buffer
        for (int i = 0; i < points_to_send; i++)
        {
            int idx = (current_afm_state.scan_buffer_tail + i) % current_afm_state.MAX_SCAN_POINTS;
            points_buffer[i] = current_afm_state.scan_data[idx];
        }

        // Update ring buffer state (tail and count)
        current_afm_state.scan_buffer_tail =
            (current_afm_state.scan_buffer_tail + points_to_send) % current_afm_state.MAX_SCAN_POINTS;
        current_afm_state.scan_buffer_count -= points_to_send;

        // Check if this is the final batch of data and scan is no longer active
        is_final_data = (current_afm_state.scan_buffer_count == 0 && !current_afm_state.scan_active);

        // Update the flag if buffer is now empty
        if (current_afm_state.scan_buffer_count == 0)
        {
            scan_data_ready_to_send = false;
        }

        // End critical section
        xSemaphoreGive(scan_buffer_mutex);
    }
    else
    {
        // Could not obtain mutex - try again later
        return;
    }
    // --- End Critical Section ---

    // --- Send data as CSV lines (outside mutex) ---
    for (int i = 0; i < points_to_send; i++)
    {
        const AFMState::ScanPoint &point = points_buffer[i];
        // Format: x,y,adc0,dacz,flags\n
        Serial.print(point.x);
        Serial.print(",");
        Serial.print(point.y);
        Serial.print(",");
        Serial.print(point.adc0);
        Serial.print(",");
        Serial.print(point.dacz);
        Serial.print(",");
        Serial.println(point.flags);
    }

    // Optional: Wait for transmit buffer to empty slightly
    // Serial.flush(); // Or delayMicroseconds(50);
}

// ============================================================================
// JSON Response Functions
// ============================================================================

JsonDocument getAFMStateAsJson()
{
    JsonDocument doc;

    // Core measurements
    doc["data_type"] = "update";
    doc["timestamp"] = current_afm_state.timestamp;
    doc["adc_0"] = current_afm_state.adc_0_val;
    doc["adc_1"] = current_afm_state.adc_1_val;
    doc["adc_0_avg"] = current_afm_state.adc_0_avg;
    doc["adc_1_avg"] = current_afm_state.adc_1_avg;
    doc["dac_f"] = current_afm_state.dac_f_val;
    doc["dac_t"] = current_afm_state.dac_t_val;
    doc["dac_x"] = current_afm_state.dac_x_val;
    doc["dac_y"] = current_afm_state.dac_y_val;
    doc["dac_z"] = current_afm_state.dac_z_val;

    // System state
    const char *state_str;
    switch (current_afm_state.current_state)
    {
    case SystemState::IDLE:
        state_str = "IDLE";
        break;
    case SystemState::FOCUSING:
        state_str = "FOCUSING";
        break;
    case SystemState::APPROACHING:
        state_str = "APPROACHING";
        break;
    case SystemState::SCANNING:
        state_str = "SCANNING";
        break;
    default:
        state_str = "UNKNOWN";
    }
    doc["state"] = state_str;

    // Motor states
    for (int i = 0; i < 3; i++)
    {
        JsonObject motor = doc[String("motor_") + String(i + 1)].to<JsonObject>();
        motor["position"] = current_afm_state.motors[i].current_position;
        motor["target"] = current_afm_state.motors[i].target_position;
        motor["is_running"] = current_afm_state.motors[i].status == MotorStatus::MOVING;
    }

    // Control Loop Timing (Frequency)
    doc["loop_last_period_us"] = current_afm_state.control_loop_last_period_us;
    float last_freq_hz = 0.0f;
    if (current_afm_state.control_loop_last_period_us > 0)
    {
        last_freq_hz = 1000000.0f / current_afm_state.control_loop_last_period_us;
    }
    doc["loop_last_freq_hz"] = last_freq_hz;

    float avg_freq_hz = 0.0f;
    if (current_afm_state.control_loop_count > 0)
    {
        float avg_period_us = (float)current_afm_state.control_loop_total_period_us / current_afm_state.control_loop_count;
        if (avg_period_us > 0)
        {
            avg_freq_hz = 1000000.0f / avg_period_us;
        }
    }
    doc["loop_avg_freq_hz"] = avg_freq_hz;

    return doc;
}

// ============================================================================
// Command Processing
// ============================================================================

String processCommand(const String &json_command)
{
    JsonDocument doc;
    JsonDocument response;
    DeserializationError error = deserializeJson(doc, json_command);

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
        response["message"] = "Invalid command format";
        String output;
        serializeJson(response, output);
        return output;
    }

    const char *command = doc["command"];

    // Command processing
    if (strcmp(command, "reset") == 0)
    {
        resetAllDevices();
        response["status"] = "success";
    }
    else if (strcmp(command, "restore") == 0)
    {
        restoreDeviceStates();
        response["status"] = "success";
    }
    else if (strcmp(command, "get_status") == 0)
    {
        response = getAFMStateAsJson();
        // Serialize and print as before
        String output;
        serializeJson(response, output);
        Serial.println(output);
        // Add flush to wait for transmission to complete
        Serial.flush();
    }
    else if (strcmp(command, "read_adc") == 0)
    {
        unsigned long start = millis();
        while (adcInUse() && (millis() - start < 100))
            delay(1);

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
        bool success = false;

        if (strcmp(channel, "T") == 0)
        {
            dac_t.write(value);
            current_afm_state.dac_t_val = value;
            success = true;
        }
        else if (strcmp(channel, "F") == 0)
        {
            dac_f.write(value);
            current_afm_state.dac_f_val = value;
            success = true;
        }
        else if (strcmp(channel, "X") == 0)
        {
            dac_x.write(value);
            current_afm_state.dac_x_val = value;
            success = true;
        }
        else if (strcmp(channel, "Y") == 0)
        {
            dac_y.write(value);
            current_afm_state.dac_y_val = value;
            success = true;
        }
        else if (strcmp(channel, "Z") == 0)
        {
            dac_z.write(value);
            current_afm_state.dac_z_val = value;
            success = true;
        }

        response["status"] = success ? "success" : "error";
        if (!success)
            response["message"] = "Invalid DAC channel";
    }
    else if (strcmp(command, "start_motor") == 0)
    {
        const auto &motor_param = doc["motor"];
        const auto &steps_param = doc["steps"];
        const auto &direction_param = doc["direction"];

        if (!motor_param.is<int>() || !steps_param.is<int>() || !direction_param.is<int>())
        {
            response["status"] = "error";
            response["message"] = "Missing required parameters (motor, steps, direction)";
        }
        else if (motor_param.as<int>() < 1 || motor_param.as<int>() > 3)
        {
            response["status"] = "error";
            response["message"] = "Invalid motor number (must be 1-3)";
        }
        else if (steps_param.as<int>() <= 0)
        {
            response["status"] = "error";
            response["message"] = "Steps must be a positive integer";
        }
        else if (direction_param.as<int>() != 0 && direction_param.as<int>() != 1)
        {
            response["status"] = "error";
            response["message"] = "Direction must be 0 (CCW) or 1 (CW)";
        }
        else if (isMotorMoving())
        {
            response["status"] = "error";
            response["message"] = "Another motor is already moving";
        }
        else
        {
            unsigned int delay_val = doc["delay"] | 500; // Use "delay" instead of "speed"
            delay_val = constrain(delay_val, 100, 5000); // Apply constraints

            if (startMotorMovement(motor_param.as<int>(), steps_param.as<int>(),
                                   direction_param.as<int>(), delay_val)) // Pass delay_val
            {
                response["status"] = "started";
                response["motor"] = motor_param.as<int>();
                response["steps"] = steps_param.as<int>();
                response["direction"] = direction_param.as<int>() ? "CW" : "CCW";
                response["delay"] = delay_val; // Report "delay"
                response["target_position"] = current_afm_state.motors[motor_param.as<int>() - 1].target_position;
            }
            else
            {
                response["status"] = "error";
                response["message"] = "Failed to start motor movement";
            }
        }
    }
    else if (strcmp(command, "stop_motor") == 0)
    {
        if (stopMotor())
        {
            response["status"] = "stopped";
            response["motor"] = motor_ctrl_state.active_motor;
        }
        else
        {
            response["status"] = "idle";
            response["message"] = "No motor was moving";
        }
    }
    else if (strcmp(command, "pid_control") == 0)
    {
        if (doc["action"] == "enable")
        {
            current_afm_state.pid_enabled = true;
            response["status"] = "success";
            response["message"] = "PID control enabled";
        }
        else if (doc["action"] == "disable")
        {
            current_afm_state.pid_enabled = false;
            response["status"] = "success";
            response["message"] = "PID control disabled";
        }
        else if (doc["action"] == "set_params")
        {
            bool target_updated = false;
            if (doc["target"].is<int>())
            {
                current_afm_state.pid_target = doc["target"].as<float>();
                // Reset PID controller state and set initial bias to current Z value
                current_afm_state.pid_controller.reset();
                current_afm_state.pid_controller.setBias(current_afm_state.dac_z_val);
                target_updated = true;
            }

            // Handle other parameters (Kp, Ki, Kd, etc.)
            if (doc["kp"].is<float>() || doc["kp"].is<int>() ||
                doc["ki"].is<float>() || doc["ki"].is<int>() ||
                doc["kd"].is<float>() || doc["kd"].is<int>())
            {
                float kp = doc["kp"] | current_afm_state.pid_controller.getKp();
                float ki = doc["ki"] | current_afm_state.pid_controller.getKi();
                float kd = doc["kd"] | current_afm_state.pid_controller.getKd();
                current_afm_state.pid_controller.setTunings(kp, ki, kd);
            }

            // Handle slew rate
            if (doc["slew_rate"].is<float>() || doc["slew_rate"].is<int>())
            {
                current_afm_state.pid_controller.setSlewRate(doc["slew_rate"].as<float>());
            }

            // Handle integral limits
            if ((doc["integral_min"].is<float>() || doc["integral_min"].is<int>()) &&
                (doc["integral_max"].is<float>() || doc["integral_max"].is<int>()))
            {
                float min_val = doc["integral_min"].as<float>();
                float max_val = doc["integral_max"].as<float>();
                current_afm_state.pid_controller.setIntegralLimits(min_val, max_val);
            }

            // Handle invert flag
            if (doc["invert"].is<bool>())
            {
                current_afm_state.pid_controller.setInvert(doc["invert"].as<bool>());
            }

            // Only update bias if target was NOT updated
            if (!target_updated && (doc["bias"].is<float>() || doc["bias"].is<int>()))
            {
                current_afm_state.pid_controller.setBias(doc["bias"].as<float>());
            }

            response["status"] = "success";
            response["message"] = "PID parameters updated";
        }
        else if (doc["action"] == "get_status")
        {
            response["enabled"] = current_afm_state.pid_enabled;
            response["target"] = current_afm_state.pid_target;
            response["kp"] = current_afm_state.pid_controller.getKp();
            response["ki"] = current_afm_state.pid_controller.getKi();
            response["kd"] = current_afm_state.pid_controller.getKd();
            response["bias"] = current_afm_state.pid_controller.getBias();
            response["invert"] = current_afm_state.pid_controller.getInvert();
            response["current_integral"] = current_afm_state.pid_controller.getIntegral();
            response["current_output"] = current_afm_state.pid_controller.getCurrentOutput();
            response["current_value"] = current_afm_state.adc_0_avg;
            response["output"] = current_afm_state.dac_z_val;
            response["status"] = "success";
        }
        else
        {
            response["status"] = "error";
            response["message"] = "Invalid PID action";
        }
    }
    else if (strcmp(command, "approach") == 0)
    {
        if (doc["action"] == "start")
        {
            const auto &motor_param = doc["motor"];
            const auto &step_size_param = doc["step_size"];
            const auto &threshold_param = doc["threshold"];
            const auto &max_steps_param = doc["max_steps"];
            const auto &delay_param = doc["delay"]; // Use "delay" instead of "speed"

            // Basic Parameter Validation
            if (!motor_param.is<int>() || !step_size_param.is<int>() || !threshold_param.is<int>() || !max_steps_param.is<int>()) // Max steps is mandatory
            {
                response["status"] = "error";
                response["message"] = "Missing required parameters (motor, step_size, threshold, max_steps)";
            }
            else if (motor_param.as<int>() < 1 || motor_param.as<int>() > 3)
            {
                response["status"] = "error";
                response["message"] = "Invalid motor number (must be 1-3)";
            }
            else if (step_size_param.as<int>() == 0)
            {
                response["status"] = "error";
                response["message"] = "Step size cannot be zero";
            }
            else if (threshold_param.as<int>() <= 0)
            {
                response["status"] = "error";
                response["message"] = "Threshold must be positive";
            }
            else if (max_steps_param.as<int>() == 0)
            {
                response["status"] = "error";
                response["message"] = "max_steps cannot be zero";
            }
            else // Basic params valid, process optional and start
            {
                // Process parameters with defaults
                unsigned int delay_val = delay_param.is<int>() ? constrain(delay_param.as<int>(), 100, 5000) : current_afm_state.approach_step_delay;

                // Call the non-blocking startApproach function
                if (startApproach(motor_param.as<int>(), step_size_param.as<int>(),
                                  threshold_param.as<int>(), max_steps_param.as<int>(), delay_val))
                {
                    // Respond immediately that approach sequence has been initiated
                    response["status"] = "started";
                    response["message"] = "Approach initiated.";
                    // Include parameters in response
                    response["motor"] = motor_param.as<int>();
                    response["step_size"] = step_size_param.as<int>();
                    response["threshold"] = threshold_param.as<int>();
                    response["delay"] = delay_val;
                    response["max_steps"] = max_steps_param.as<int>();
                }
                else
                {
                    // Failed initialization
                    response["status"] = "error";
                    response["message"] = "Failed to start approach (check state or parameters)";
                }
            }
        }
        else if (doc["action"] == "stop")
        {
            // Signal the background process (updateApproach) to stop
            stopApproach();                  // Calls the flag-setting function
            response["status"] = "stopping"; // Indicate stop was requested
            // Optional: Add check if it was already stopped
            // if (!current_afm_state.approach_running && current_afm_state.current_state != SystemState::APPROACHING) {
            //     response["message"] = "Approach not running.";
            // }
        }
        else if (doc["action"] == "get_data")
        {
            response["status"] = "success";
            JsonArray data = response["data"].to<JsonArray>();
            for (int i = 0; i < current_afm_state.approach_data_count; i++)
            {
                JsonObject obj = data.add<JsonObject>();
                obj["steps"] = current_afm_state.approach_data[i].steps;
                obj["adc"] = current_afm_state.approach_data[i].adc;
                obj["position"] = current_afm_state.approach_data[i].position;
            }
            // Clear the data after sending
            current_afm_state.approach_data_count = 0;
        }
        else
        {
            response["status"] = "error";
            response["message"] = "Invalid approach action";
        }
    }
    else if (strcmp(command, "scan") == 0)
    {
        // Example Start Scan Command:
        // {"command":"scan","action":"start","x_start":10000,"x_end":55000,"y_start":10000,"y_end":55000,"resolution":16,"dwell_time":2}

        if (doc["action"] == "start")
        {
            const auto &x_start_param = doc["x_start"];
            const auto &x_end_param = doc["x_end"];
            const auto &y_start_param = doc["y_start"];
            const auto &y_end_param = doc["y_end"];
            const auto &resolution_param = doc["resolution"];
            const auto &dwell_time_param = doc["dwell_time"];

            if (!x_start_param.is<int>() || !x_end_param.is<int>() || !y_start_param.is<int>() ||
                !y_end_param.is<int>() || !resolution_param.is<int>() || !dwell_time_param.is<int>())
            {
                response["status"] = "error";
                response["message"] = "Missing or invalid required parameters (x_start, x_end, y_start, y_end, resolution, dwell_time must be integers)";
            }
            else if (x_start_param.as<int>() < 0 || x_start_param.as<int>() > 65535 ||
                     x_end_param.as<int>() < 0 || x_end_param.as<int>() > 65535 ||
                     y_start_param.as<int>() < 0 || y_start_param.as<int>() > 65535 ||
                     y_end_param.as<int>() < 0 || y_end_param.as<int>() > 65535)
            {
                response["status"] = "error";
                response["message"] = "DAC coordinates out of range (0-65535)";
            }
            else if (x_start_param.as<int>() > x_end_param.as<int>() || y_start_param.as<int>() > y_end_param.as<int>())
            {
                response["status"] = "error";
                response["message"] = "Invalid range (start > end)";
            }
            else if (resolution_param.as<int>() < 2)
            {
                response["status"] = "error";
                response["message"] = "Resolution must be at least 2";
            }
            else if (dwell_time_param.as<int>() < 0)
            {
                response["status"] = "error";
                response["message"] = "Dwell time must be non-negative";
            }
            else
            {
                if (startScan(x_start_param.as<int>(), x_end_param.as<int>(), y_start_param.as<int>(), y_end_param.as<int>(), resolution_param.as<int>(), dwell_time_param.as<int>()))
                {
                    response["status"] = "started";
                    response["message"] = "Scan initiated";
                    response["x_start"] = x_start_param.as<int>();
                    response["x_end"] = x_end_param.as<int>();
                    response["y_start"] = y_start_param.as<int>();
                    response["y_end"] = y_end_param.as<int>();
                    response["resolution"] = resolution_param.as<int>();
                    response["dwell_time"] = dwell_time_param.as<int>();
                }
                else
                {
                    response["status"] = "error";
                    response["message"] = "Failed to start scan (check state or parameters)";
                }
            }
        }
        else if (doc["action"] == "stop")
        {
            stopScan();
            response["status"] = "stopped";
            response["message"] = "Scan stopped";
        }
        else if (doc["action"] == "get_data")
        {
            String format = doc["format"] | "json"; // Default to json -> this setting is now ignored for data itself

            // Data is pushed automatically. This command only returns status.
            // Check if scan is finished and buffer is empty
            if (current_afm_state.scan_buffer_count == 0 && !current_afm_state.scan_active)
            {
                response["status"] = "scan_complete";
                response["message"] = "Scan finished and all data sent.";
                // No data points to send, just return the status.
            }
            else
            {
                response["status"] = "success";
                response["message"] = "Scan status provided. Data is pushed via CSV stream.";
                response["scan_active"] = current_afm_state.scan_active;
                response["points_in_buffer"] = current_afm_state.scan_buffer_count; // Use buffer count
                                                                                    // response["data_waiting_send"] = scan_data_ready_to_send.load(); // This flag is less informative now
            }
            // No data is attached here anymore
            // Fall through to serialize and return the JSON response
        }
        else
        {
            response["status"] = "error";
            response["message"] = "Invalid scan action";
        }
    }
    else if (strcmp(command, "set_adc_avg_window") == 0)
    {
        const auto &window_size_param = doc["window_size"];
        if (!window_size_param.is<int>())
        {
            response["status"] = "error";
            response["message"] = "Missing or invalid 'window_size' parameter (must be integer)";
        }
        else
        {
            int new_size = window_size_param.as<int>();
            if (new_size >= 1 && new_size <= MAX_ADC_AVG_WINDOW_SIZE)
            {
                current_afm_state.adc_avg_window_size = new_size;
                // Reset history when changing window size
                current_afm_state.adc_history_index = 0;
                current_afm_state.adc_history_count = 0;
                // Optional: Clear history buffer, but not strictly necessary
                // memset(current_afm_state.adc_0_history, 0, sizeof(current_afm_state.adc_0_history));
                // memset(current_afm_state.adc_1_history, 0, sizeof(current_afm_state.adc_1_history));
                response["status"] = "success";
                response["message"] = String("ADC average window size set to ") + String(new_size);
            }
            else
            {
                response["status"] = "error";
                response["message"] = String("Invalid 'window_size' (must be 1-") + String(MAX_ADC_AVG_WINDOW_SIZE) + ")";
            }
        }
    }
    else
    {
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
TaskHandle_t pid_task_handle;

void pidControlTask(void *parameter)
{
    // Subscribe this task to the Task Watchdog Timer
    esp_task_wdt_add(NULL);

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms period for 1kHz control loop
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Wait for the next cycle using vTaskDelayUntil for precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Only update ADC and PID if PID is enabled
        if (current_afm_state.pid_enabled)
        {
            updateAFMState(); // Get latest ADC reading
            updatePIDControl();
        }

        // Reset watchdog
        esp_task_wdt_reset();
    }
}

void controlTask(void *parameter)
{
    // Subscribe this task to the Task Watchdog Timer
    esp_task_wdt_add(NULL);

    // Variable to store the start time of the previous iteration
    static uint64_t lastStartTime = 0;
    while (1)
    {
        // Wait for the next cycle.

        // --- Measure Period and Frequency ---
        uint64_t currentStartTime = esp_timer_get_time();
        uint32_t period_us = 0;

        if (lastStartTime != 0)
        { // Calculate period after the first iteration
            period_us = (uint32_t)(currentStartTime - lastStartTime);

            // Update state for averaging
            current_afm_state.control_loop_last_period_us = period_us;
            current_afm_state.control_loop_total_period_us += period_us;
            current_afm_state.control_loop_count++;

            // Reset accumulator periodically to prevent overflow and keep average relevant
            // Reset after ~1 million samples (adjust as needed)
            if (current_afm_state.control_loop_count >= 1000000)
            {
                // Start new average with last period
                current_afm_state.control_loop_total_period_us = current_afm_state.control_loop_last_period_us;
                current_afm_state.control_loop_count = 1;
            }
        }
        lastStartTime = currentStartTime; // Store current start time for the next iteration's calculation

        // Perform control actions (Execution time of these does not affect frequency measurement)
        updateMotorMovement();
        updateDacRamp();  // Handles XY ramps (and potentially non-approach Z)
        updateApproach(); // Handles approach state machine, including calling updateZramp
        updateScan();

        // Only update AFM state if PID is not enabled (PID task handles it otherwise)
        if (!current_afm_state.pid_enabled)
        {
            updateAFMState();
        }

        // Reset the WDT after completing the work for this cycle
        esp_task_wdt_reset();

        // Small delay/yield to prevent WDT issues on very fast loops and allow lower priority tasks
        if (current_afm_state.control_loop_last_period_us < 100)
        {                                 // e.g., faster than 10kHz
            vTaskDelay(pdMS_TO_TICKS(1)); // Yield for 1ms
        }
        else
        {
            taskYIELD(); // Yield briefly if loop is slower
        }
    }
}

// ============================================================================
// Main Setup and Loop
// ============================================================================

void setup()
{
    disableCore0WDT(); // Only if your task is pinned to core 0

    // Create the mutex for scan buffer protection
    scan_buffer_mutex = xSemaphoreCreateMutex();
    if (scan_buffer_mutex == NULL)
    {
        Serial.println("Error: Failed to create scan buffer mutex!");
    }

    scan_data_ready_to_send.store(false); // Initialize atomic flag

    // Initialize hardware
    setupSPI();
    setupMotorPins();

    // Initialize serial communication
    Serial.setRxBufferSize(2048);
    Serial.begin(921600);
    while (!Serial)
        delay(1);

    // Create PID control task with higher priority than main control task
    xTaskCreatePinnedToCore(
        pidControlTask,
        "PIDTask",
        4096,
        NULL,
        configMAX_PRIORITIES - 1, // Highest priority
        &pid_task_handle,
        1 // Run on core 1
    );

    // Create main control task
    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        4096,
        NULL,
        configMAX_PRIORITIES - 2, // Lower priority than PID
        &control_task_handle,
        0 // Run on core 0
    );
}

void loop()
{
    // Check if scan data needs to be sent (push mechanism)
    if (scan_data_ready_to_send.load())
    {
        sendScanDataBuffer(); // Send buffered data and reset flag
    }

    // Check for incoming commands
    if (Serial.available())
    {
        String json_command = Serial.readStringUntil('\n');
        json_command.trim();

        if (json_command.length() > 0)
        {
            String response_string = processCommand(json_command);
            // processCommand now never returns "" for scan data,
            // as CSV data is sent via sendScanDataBuffer.
            // So, always print the JSON response from processCommand.
            Serial.println(response_string);
        }
    }

    // We could add a small delay here if needed, but the controlTask
    // runs independently. Ensure this loop runs fast enough to
    // call sendScanDataBuffer promptly when needed.
    // delay(1); // Example: small delay if CPU usage is too high
}