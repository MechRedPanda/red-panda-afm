from enum import Enum
import json
import threading
from collections import deque
from datetime import datetime
import time
import serial
from serial.tools import list_ports
from typing import Dict, Optional, Union, List, Tuple, Any
from dataclasses import dataclass, field

# Default serial port settings
DEFAULT_PORT = None  # Will auto-detect if None
BAUDRATE = 115200
TIMEOUT = 1
QUEUE_MAXLEN = 10000


class AFMState(Enum):
    """Enum representing the possible states of the AFM system."""
    IDLE = "IDLE"
    FOCUSING = "FOCUSING"
    APPROACHING = "APPROACHING"
    SCANNING = "SCANNING"


@dataclass
class MotorStatus:
    """Data class representing the status of a motor."""
    position: int = 0       # Current step position
    target: int = 0         # Target position
    is_running: bool = False  # True if motor is currently moving


@dataclass
class AFMStatus:
    """Data class representing the complete status of the AFM system."""
    # Core AFM measurements
    timestamp: int          # Timestamp in milliseconds
    adc_0: int              # ADC channel 0 reading
    adc_1: int              # ADC channel 1 reading
    dac_f: int              # Feedback DAC value
    dac_t: int              # Tuning DAC value
    dac_x: int              # X-axis DAC value
    dac_y: int              # Y-axis DAC value
    dac_z: int              # Z-axis DAC value

    # Motor status (position, target, running state)
    motor_1: MotorStatus = field(default_factory=MotorStatus)  # Motor 1 status
    motor_2: MotorStatus = field(default_factory=MotorStatus)  # Motor 2 status
    motor_3: MotorStatus = field(default_factory=MotorStatus)  # Motor 3 status

    # System state
    state: AFMState = AFMState.IDLE


class AFM:
    """Main class for controlling the Atomic Force Microscope system."""

    def __init__(self):
        """Initialize the AFM system with default settings."""
        self.serial_port = DEFAULT_PORT
        self.baudrate = BAUDRATE
        self.timeout = TIMEOUT
        self.serial_ports = []
        self.serial_conn = None
        self.status_queue = deque(maxlen=QUEUE_MAXLEN)
        self.busy = False
        self.busy_lock = threading.Lock()  # Add lock for busy flag
        self.focus_interrupted = False
        self.current_state = AFMState.IDLE
        self.focus_results = []
        self.focus_running = False
        self.motor_delay = 500  # us, default motor delay
        self._approach_params = None
        self._approach_thread = None
        self.approach_data = []  # Store approach data points
        self.optimal_focus_value = None  # Store the optimal focus value
        self.serial_lock = threading.Lock()
        self._focus_event = threading.Event()
        self._approach_event = threading.Event()

    # Serial Communication Methods
    def get_serial_ports(self) -> List[Any]:
        """Get a list of available serial ports."""
        self.serial_ports = list_ports.comports()
        return self.serial_ports

    def find_afm_device(self) -> Optional[str]:
        """Try to automatically find the AFM device by checking serial ports."""
        ports = list_ports.comports()
        for port in ports:
            if "USB" in port.description or "Serial" in port.description:
                return port.device
        return None

    def connect(self, port: Optional[str] = None, baudrate: int = BAUDRATE,
                timeout: float = TIMEOUT) -> bool:
        """Connect to the AFM device via serial."""
        try:
            self.baudrate = baudrate
            self.timeout = timeout

            if port is None:
                print("Could not auto-detect AFM device. Please specify port manually.")
                return False

            self.serial_port = port
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )

            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            print(f"Connected to {self.serial_port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            self.serial_conn = None
            return False

    def is_connected(self) -> bool:
        """Check if the AFM is connected."""
        return self.serial_conn is not None and self.serial_conn.is_open

    def close(self) -> None:
        """Close the serial connection."""
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
            print("Connection closed.")

    # Command Communication Methods
    def send_command(self, command: Dict) -> None:
        """Send a command to the AFM device."""
        if not self.serial_conn:
            print("Not connected")
            return
        try:
            command_json = json.dumps(command) + "\n"
            self.serial_conn.write(command_json.encode('utf-8'))
        except Exception as e:
            print(f"Error sending command: {e}")

    def send_and_receive(self, command: Dict, timeout: Optional[float] = None) -> Optional[Dict]:
        """Send a command and wait for a response from the AFM device."""
        if not self.serial_conn:
            print("Not connected")
            return None

        try:
            original_timeout = self.serial_conn.timeout
            if timeout is not None:
                self.serial_conn.timeout = timeout

            self.serial_conn.reset_input_buffer()
            command_json = json.dumps(command) + "\n"
            self.serial_conn.write(command_json.encode('utf-8'))

            # Read response with retry logic
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    # Read until we get a complete line
                    response = ""
                    while True:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if not line:
                            if attempt < max_retries - 1:
                                time.sleep(0.1)  # Wait a bit before retrying
                                break
                            return None

                        response += line
                        # Try to parse the response
                        try:
                            # Check if we have a complete JSON object
                            if response.count('{') == response.count('}'):
                                return json.loads(response)
                            # If not complete, continue reading
                        except json.JSONDecodeError:
                            # If we have a complete line but it's not valid JSON,
                            # it might be a partial response, so continue reading
                            continue

                except Exception as e:
                    print(f"Error reading response: {e}")
                    print(f"cmd: {command}")
                    if attempt < max_retries - 1:
                        time.sleep(0.1)
                        continue
                    return None

        except Exception as e:
            print(f"Communication error: {e}")
            return None

        return None

    # Status and State Management
    def set_busy(self) -> bool:
        """
        Set the AFM as busy, pausing updates.
        Returns True if successfully set to busy, False if already busy.
        """
        with self.busy_lock:
            if self.busy:
                return False
            self.busy = True
            print("AFM is now busy. Updates paused.")
            return True

    def set_idle(self) -> None:
        """Set the AFM as idle, resuming updates."""
        with self.busy_lock:
            self.busy = False
            print("AFM is now idle. Updates resumed.")
            self.set_state(AFMState.IDLE)

    def is_busy(self) -> bool:
        """Check if AFM is busy."""
        with self.busy_lock:
            return self.busy

    def set_state(self, state: AFMState) -> Optional[Dict]:
        """Set the AFM state."""
        if not isinstance(state, AFMState):
            raise ValueError("Invalid state. Must be an instance of AFMState.")
        return self.send_and_receive({"command": "set_state", "state": state.name})

    def get_status(self) -> Optional[AFMStatus]:
        """Get current status from AFM device."""
        response = self.send_and_receive(
            {"command": "get_status"}, timeout=2.0)
        if response:
            try:
                def get_motor_data(num: int) -> Dict:
                    return response.get(f"motor_{num}", {})

                status = AFMStatus(
                    timestamp=int(response.get("timestamp", 0)),
                    adc_0=int(response.get("adc_0", 0)),
                    adc_1=int(response.get("adc_1", 0)),
                    dac_f=int(response.get("dac_f", 0)),
                    dac_t=int(response.get("dac_t", 0)),
                    dac_x=int(response.get("dac_x", 0)),
                    dac_y=int(response.get("dac_y", 0)),
                    dac_z=int(response.get("dac_z", 0)),
                    motor_1=MotorStatus(
                        position=int(get_motor_data(1).get("position", 0)),
                        target=int(get_motor_data(1).get("target", 0)),
                        is_running=bool(get_motor_data(
                            1).get("is_running", False))
                    ),
                    motor_2=MotorStatus(
                        position=int(get_motor_data(2).get("position", 0)),
                        target=int(get_motor_data(2).get("target", 0)),
                        is_running=bool(get_motor_data(
                            2).get("is_running", False))
                    ),
                    motor_3=MotorStatus(
                        position=int(get_motor_data(3).get("position", 0)),
                        target=int(get_motor_data(3).get("target", 0)),
                        is_running=bool(get_motor_data(
                            3).get("is_running", False))
                    ),
                    state=AFMState(response.get("state", "IDLE"))
                )

                self.status_queue.append(status)
                self.current_state = status.state
                return status

            except Exception as e:
                print(f"Error parsing AFM status: {e}")
                print(f"Raw response: {response}")

        return None

    # Motor Control Methods
    def move_motor(self, motor: int, steps: int) -> Optional[Dict]:
        """Start non-blocking motor movement."""
        if motor not in {1, 2, 3}:
            raise ValueError("Motor must be 1, 2, or 3")
        direction = 0 if steps < 0 else 1 if steps > 0 else 1
        steps = abs(steps)
        speed = self.motor_delay

        response = self.send_and_receive({
            'command': 'start_motor',
            'motor': motor,
            'steps': steps,
            'direction': direction,
            'speed': speed
        })
        print(response)
        return response

    def move_motor_blocking(self, motor: int, steps: int,
                            speed: Optional[int] = None) -> Optional[Dict]:
        """Move motor and wait until movement is complete."""
        if motor not in {1, 2, 3}:
            raise ValueError("Motor must be 1, 2, or 3")
        direction = 0 if steps < 0 else 1 if steps > 0 else 1
        steps = abs(steps)
        speed = speed if speed is not None else self.motor_delay

        response = self.send_and_receive({
            'command': 'move_motor_blocking',
            'motor': motor,
            'steps': steps,
            'direction': direction,
            'speed': speed
        })

        if response is None:
            print("Failed to receive response from motor movement")
            return None

        return response

    def stop_motor(self, motor: Optional[int] = None) -> Optional[Dict]:
        """Stop motor movement."""
        cmd = {'command': 'stop_motor'}
        if motor in {1, 2, 3}:
            cmd['motor'] = motor
        return self.send_and_receive(cmd)

    def is_moving(self, motor: Optional[int] = None) -> bool:
        """Check if motor(s) are moving."""
        status = self.get_motor_status(motor)
        if motor is None:
            return status.get('is_moving', False)
        return status.get('motors', [{}])[0].get('status', 'IDLE') == 'MOVING'

    # DAC Control Methods
    def set_dac(self, channel: str, value: int) -> bool:
        """Set DAC value for specified channel (F, T, X, Y, Z)"""
        if not self.is_connected():
            return False
        try:
            response = self.send_and_receive({
                "command": "set_dac",
                "channel": channel,
                "value": value
            })
            return response is not None
        except Exception as e:
            print(f"Error setting DAC: {e}")
            return False

    def ramp_dac(self, channel: str, target_value: int, step_size: int = 100, delay_ms: int = 10) -> bool:
        """
        Gradually ramp DAC value from current to target value.

        Args:
            channel (str): DAC channel (F, T, X, Y, Z)
            target_value (int): Target DAC value to reach
            step_size (int): Number of steps to change per iteration
            delay_ms (int): Delay between steps in milliseconds

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected():
            return False

        try:
            # Get current DAC value
            status = self.get_status()
            if not status:
                return False

            current_value = getattr(status, f'dac_{channel.lower()}')

            # Calculate number of steps needed
            steps = abs(target_value - current_value)
            if steps == 0:
                return True  # Already at target

            # Determine direction
            direction = 1 if target_value > current_value else -1

            # Calculate number of iterations
            iterations = steps // step_size
            if steps % step_size != 0:
                iterations += 1

            # Perform the ramp
            for i in range(iterations):
                # Calculate next value
                next_value = current_value + \
                    (direction * min(step_size, steps - (i * step_size)))

                # Set the new value using send_and_receive
                response = self.send_and_receive({
                    "command": "set_dac",
                    "channel": channel,
                    "value": next_value
                })

                if response is None:
                    return False

                # Wait for the specified delay
                time.sleep(delay_ms / 1000.0)

            return True

        except Exception as e:
            print(f"Error ramping DAC: {e}")
            return False

    def reset(self) -> Optional[Dict]:
        """Reset the AFM device."""
        return self.send_and_receive({"command": "reset"})

    def restore(self) -> bool:
        """
        Restore AFM to default settings by sending a restore command to the ESP32.
        This will reset all DAC values to default (32768) and reset motor positions.
        """
        if not self.is_connected():
            return False

        try:
            response = self.send_and_receive({
                "command": "restore"
            })
            return response is not None

        except Exception as e:
            print(f"Error during restore: {e}")
            return False

    # Focus Methods
    def focus(self, start: int, end: int, step_size: int) -> bool:
        """Start a DAC sweep on dac_f. Results are available in self.focus_results.
        Automatically finds and stores the optimal focus point.

        Args:
            start (int): Starting DAC value
            end (int): Ending DAC value
            step_size (int): Step size for the sweep
        """
        if self.busy:
            print("AFM is busy. Cannot start focus.")
            return False

        if not all(isinstance(v, int) for v in (start, end, step_size)):
            raise ValueError("Start, end, and step_size must all be integers.")
        if step_size <= 0:
            raise ValueError("Step size must be > 0.")
        if start == end:
            raise ValueError("Start and end values must differ.")
        self.focus_interrupted = False

        def run_focus():
            self.set_busy()
            self.focus_running = True
            self.focus_results = []
            direction = 1 if end > start else -1
            steps = range(start, end + direction, direction * step_size)

            try:
                self.set_state(AFMState.FOCUSING)

                # Ramp to initial position and wait
                if not self.ramp_dac("F", start, step_size=100, delay_ms=10):
                    print("Failed to ramp to initial position")
                    return
                time.sleep(1.0)  # Wait for 1 second before starting scan

                for dac_f_value in steps:
                    if self.focus_interrupted:
                        print("Focus sweep interrupted.")
                        break

                    response = self.send_and_receive(
                        {"command": "set_dac", "channel": "F", "value": dac_f_value},
                        timeout=0.5
                    )

                    if response is None:
                        continue
                    time.sleep(0.01)  # Allow time for DAC to settle

                    response = self.send_and_receive(
                        {"command": "read_adc"},
                        timeout=0.5
                    )
                    if response is None:
                        continue

                    adc_0 = int(response.get("adc_0", 0))
                    adc_1 = int(response.get("adc_1", 0))
                    self.focus_results.append((dac_f_value, adc_0, adc_1))

            finally:
                # Always calculate optimal focus point, even if interrupted
                if len(self.focus_results) > 0:
                    self.optimal_focus_value = self.find_optimal_focus()
                    if self.optimal_focus_value is not None:
                        print(
                            f"Optimal focus point found: {self.optimal_focus_value}")

                self.focus_running = False
                self.set_idle()

        threading.Thread(target=run_focus, daemon=True).start()
        return True

    def get_optimal_focus(self) -> Optional[int]:
        """Get the stored optimal focus value."""
        return self.optimal_focus_value

    def find_optimal_focus(self) -> Optional[int]:
        """
        Find the optimal focus point from the focus results.
        The optimal point is the midpoint between the highest and lowest ADC values.
        """
        if not self.focus_results:
            return None

        # Extract ADC values
        adc_values = [point[1] for point in self.focus_results]  # adc_0 values
        dac_values = [point[0] for point in self.focus_results]

        # Find min and max ADC values
        min_adc = min(adc_values)
        max_adc = max(adc_values)

        # Find corresponding DAC values
        min_dac = dac_values[adc_values.index(min_adc)]
        max_dac = dac_values[adc_values.index(max_adc)]

        # Calculate midpoint
        optimal_dac = (min_dac + max_dac) // 2

        print(f"Focus analysis:")
        print(f"Min ADC: {min_adc} at DAC: {min_dac}")
        print(f"Max ADC: {max_adc} at DAC: {max_dac}")
        print(f"Optimal DAC: {optimal_dac}")

        return optimal_dac

    def stop_focus(self) -> None:
        """Request the currently running focus sweep to stop."""
        self.focus_interrupted = True

    def is_focus_running(self) -> bool:
        """Check if focus sweep is currently active."""
        return self.focus_running

    # Approach Methods
    def approach(self, motor: int, step_size: int, adc_channel: int = 0, threshold: int = 10, max_steps: int = None, polling_interval: float = 0.05):
        """Approach procedure that moves a motor until ADC value changes by threshold.

        Args:
            motor (int): Motor number (1, 2, or 3)
            step_size (int): Number of steps to move in each iteration. Positive for forward, negative for backward.
            adc_channel (int): ADC channel to monitor (0 or 1)
            threshold (int): Threshold value for ADC change
            max_steps (int): Maximum total steps to move (None for no limit)
            polling_interval (float): Time between movements in seconds
        """
        if not self.is_connected():
            return False

        if self.busy:
            return False

        # Validate parameters
        if motor not in [1, 2, 3]:
            return False
        if adc_channel not in [0, 1]:
            return False
        if threshold <= 0:
            return False
        if max_steps is not None and max_steps <= 0:
            return False
        if step_size == 0:
            return False  # Don't allow zero step size

        # Clear previous approach data
        self.approach_data = []

        # Get initial status
        status = self.get_status()
        if not status:
            return False

        # Store initial values
        initial_adc = status.adc_0 if adc_channel == 0 else status.adc_1
        initial_position = getattr(status, f"motor_{motor}").position

        def approach_thread():
            try:
                self.busy = True
                self._approach_event.clear()
                self.set_state(AFMState.APPROACHING)

                total_steps_moved = 0
                while True:
                    if self._approach_event.is_set():
                        print("Approach interrupted by stop signal")
                        break

                    # Move motor by step_size (direction is determined by step_size sign)
                    if not self.move_motor_blocking(motor, step_size):
                        print("Failed to move motor")
                        break

                    # Get current status
                    status = self.get_status()
                    if not status:
                        print("Failed to get status")
                        break

                    # Get current ADC value and position
                    current_adc = status.adc_0 if adc_channel == 0 else status.adc_1
                    current_position = getattr(
                        status, f"motor_{motor}").position
                    # Track absolute steps moved
                    total_steps_moved += abs(step_size)

                    # Store data point
                    self.approach_data.append({
                        'steps': current_position - initial_position,
                        'adc': current_adc,
                        'position': current_position
                    })
                    print(self.approach_data[-1])
                    # Check if threshold is reached
                    if abs(current_adc - initial_adc) >= threshold:
                        print(
                            f"Threshold reached after {total_steps_moved} steps")
                        break

                    # Check max steps
                    if max_steps is not None and total_steps_moved >= max_steps:
                        print(f"Reached maximum steps: {max_steps}")
                        break

                    time.sleep(polling_interval)

            except Exception as e:
                print(f"Error in approach thread: {e}")
            finally:
                self.busy = False
                self.set_state(AFMState.IDLE)
                # Don't clear the event here, let stop_approach handle it

        # Start approach in a separate thread
        self._approach_thread = threading.Thread(target=approach_thread)
        self._approach_thread.daemon = True
        self._approach_thread.start()

        return True

    def stop_approach(self) -> None:
        """Request the currently running approach to stop."""
        if self.busy:  # Check busy flag first
            print("Stopping approach...")
            self._approach_event.set()
            # Wait for thread to finish
            # Increased timeout to 2 seconds
            self._approach_thread.join(timeout=2.0)
            if self._approach_thread.is_alive():
                print("Warning: Approach thread did not stop gracefully")
            self._approach_thread = None
            self._approach_event.clear()  # Clear the event after thread is done
            self.busy = False
            self.set_state(AFMState.IDLE)
            print("Approach stopped")
        else:
            print("Approach not running")

    def is_approach_running(self) -> bool:
        """Check if approach is currently active."""
        return self.busy  # Simplified check - if we're busy, we're running

    def enable_pid(self, target: Optional[int] = None) -> str:
        """Enable PID control.

        Args:
            target (Optional[int]): Target ADC value. If None, current ADC value is used.

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected():
            return "Not connected"

        try:
            # Prepare command
            command = {
                "command": "pid_control",
                "action": "enable"
            }

            # Add target if specified
            if target is not None:
                command["target"] = target

            # Send command
            return self.send_and_receive(command)

        except Exception as e:
            return(f"Error enabling PID: {e}")

    def disable_pid(self) -> bool:
        """Disable PID control.

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected():
            return False

        try:
            # Send disable command
            response = self.send_and_receive({
                "command": "pid_control",
                "action": "disable"
            })
            return response.get("status") == "success"

        except Exception as e:
            print(f"Error disabling PID: {e}")
            return False

    def set_pid_parameters(self, kp: Optional[float] = None, ki: Optional[float] = None,
                           kd: Optional[float] = None, invert: Optional[bool] = None) -> bool:
        """Set PID control parameters.

        Args:
            kp (Optional[float]): Proportional gain
            ki (Optional[float]): Integral gain
            kd (Optional[float]): Derivative gain
            invert (Optional[bool]): Whether to invert the PID output

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected():
            return False

        try:
            # Prepare command
            command = {
                "command": "pid_control",
                "action": "set_params"
            }

            # Add parameters if specified
            if kp is not None:
                command["kp"] = kp
            if ki is not None:
                command["ki"] = ki
            if kd is not None:
                command["kd"] = kd
            if invert is not None:
                command["invert"] = invert

            # Send command
            response = self.send_and_receive(command)
            return response.get("status") == "success"

        except Exception as e:
            print(f"Error setting PID parameters: {e}")
            return False

    def get_pid_status(self) -> Optional[Dict]:
        """Get current PID control status.

        Returns:
            Optional[Dict]: Dictionary containing PID status information, or None if failed
        """
        if not self.is_connected():
            return None

        try:
            # Send get status command
            response = self.send_and_receive({
                "command": "pid_control",
                "action": "get_status"
            })

            if response.get("status") == "success":
                return {
                    "enabled": response.get("enabled", False),
                    "target": response.get("target", 0),
                    "kp": response.get("kp", 0),
                    "ki": response.get("ki", 0),
                    "kd": response.get("kd", 0),
                    "invert": response.get("invert", False),
                    "current_value": response.get("current_value", 0),
                    "output": response.get("output", 0)
                }
            return None

        except Exception as e:
            print(f"Error getting PID status: {e}")
            return None


if __name__ == "__main__":
    afm = AFM()
    if afm.connect():
        try:
            while True:
                response = afm.get_status()
                print(response)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            afm.close()
