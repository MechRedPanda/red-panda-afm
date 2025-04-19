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
import numpy as np
import queue # Added for response queue

# Default serial port settings
DEFAULT_PORT = None  # Will auto-detect if None
BAUDRATE = 921600
TIMEOUT = 1
QUEUE_MAXLEN = 10000
SERIAL_READ_TIMEOUT = 0.1 # Timeout for serial read attempts in background thread
RESPONSE_TIMEOUT = 2.0 # Default timeout for waiting for a command response


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
        self.response_queue = queue.Queue()
        self._read_thread = None
        self._stop_read_thread = threading.Event()
        self.scan_data = deque() # Use deque for thread-safe appends
        self.scan_running = False
        self._scan_event = threading.Event() # Added for potential future use
        self.scan_resolution = 0 # Store resolution of the current/last scan

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

            # Start the background reader thread
            self._stop_read_thread.clear()
            self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
            self._read_thread.start()

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
        """Close the serial connection and stop the reader thread."""
        if self._read_thread and self._read_thread.is_alive():
            print("Stopping reader thread...")
            self._stop_read_thread.set()
            self._read_thread.join(timeout=2.0) # Wait for thread to finish
            if self._read_thread.is_alive():
                print("Warning: Reader thread did not stop gracefully.")
            self._read_thread = None

        if self.serial_conn and self.serial_conn.is_open:
            print("Closing serial connection...")
            try:
                self.serial_conn.close()
            except Exception as e:
                print(f"Error closing serial port: {e}")
            self.serial_conn = None
            print("Connection closed.")
        else:
            self.serial_conn = None # Ensure it's None even if it wasn't open

    # Command Communication Methods
    def send_command(self, command: Dict) -> bool:
        """Send a command to the AFM device. Returns True on success."""
        if not self.is_connected():
            print("Error: Not connected")
            return False
        try:
            command_json = json.dumps(command) + "\n"
            with self.serial_lock:
                self.serial_conn.write(command_json.encode("utf-8"))
            return True
        except serial.SerialException as e:
            print(f"Error sending command (SerialException): {e}")
            self.close() # Close connection on write error
            return False
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def send_and_receive(self, command: Dict, timeout: Optional[float] = None) -> Optional[Dict]:
        """Send a command and wait for its JSON response from the queue."""
        if not self.is_connected():
            print("Error: Not connected")
            return None

        # Clear any stale responses before sending
        while not self.response_queue.empty():
            try: self.response_queue.get_nowait()
            except queue.Empty: break

        if not self.send_command(command):
            return None # Sending failed

        effective_timeout = timeout if timeout is not None else RESPONSE_TIMEOUT

        try:
            # Wait for the response from the background reader thread
            response = self.response_queue.get(timeout=effective_timeout)
            return response
        except queue.Empty:
            print(f"Timeout waiting for response to command: {command}")
            return None
        except Exception as e:
            print(f"Error receiving response from queue: {e}")
            return None

    # --- Background Serial Reader Thread --- 
    def _serial_read_loop(self):
        """Background thread to continuously read serial data."""
        print("Serial reader thread started.")
        buffer = ""
        reading_scan_data = False
        points_to_read = 0
        scan_active_flag = False

        while not self._stop_read_thread.is_set():
            line = None
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    # Use a short timeout for readline to keep the loop responsive
                    # and allow checking the stop event regularly.
                    # Reading is blocking, so lock is needed.
                    with self.serial_lock:
                         # Check if bytes are available before blocking read
                        if self.serial_conn.in_waiting > 0:
                             line_bytes = self.serial_conn.readline()
                             if line_bytes:
                                 line = line_bytes.decode("utf-8", errors="ignore").strip()
                        else:
                             # No data immediately available, yield to prevent busy-wait
                             time.sleep(0.01)
                else:
                    # Connection closed externally
                    print("Reader thread: Connection lost.")
                    break # Exit thread if connection is closed

            except serial.SerialException as e:
                print(f"Reader thread: Serial error: {e}")
                self.close() # Attempt to close gracefully
                break # Exit thread on serial error
            except Exception as e:
                print(f"Reader thread: Unexpected error reading line: {e}")
                time.sleep(0.1) # Avoid tight loop on unexpected error
                continue

            if line is None:
                continue # No line read, loop again
            # --- Process Line --- 
            try:
                if reading_scan_data:
                    # Currently reading CSV data points after header
                    parts = line.split(',')
                    if len(parts) == 4:
                        try:
                            point_data = [int(p) for p in parts]
                            self.scan_data.append(point_data)
                            points_to_read -= 1
                        except ValueError:
                            print(f"Reader thread: Warning - Malformed CSV data skipped: {line}")
                    else:
                        print(f"Reader thread: Warning - Unexpected line format during scan data read: {line}")
                        # Potentially reset state if format is wrong?
                    
                    if points_to_read <= 0:
                        reading_scan_data = False
                        # Check the active flag received in the header
                        if not scan_active_flag and self.scan_running:
                            print("Reader thread: Scan reported inactive by ESP32.")
                            self.scan_running = False
                            # Transition state if needed (do this carefully outside lock)
                            if self.current_state == AFMState.SCANNING:
                                 self.set_idle() # Reset state and busy flag

                    print(len(self.scan_data))

                elif line.startswith("# SCAN_DATA_CSV"):
                    # Detected scan data header
                    parts = line.split(',')
                    try:
                        points_str = parts[1].split(':')[1].strip()
                        active_str = parts[2].split(':')[1].strip()
                        points_to_read = int(points_str)
                        scan_active_flag = (int(active_str) == 1)
                        if points_to_read > 0:
                            reading_scan_data = True
                            print(f"Reader thread: Receiving {points_to_read} scan points (active: {scan_active_flag})...") # Debug
                        else:
                             # Header indicates 0 points, check active flag immediately
                            if not scan_active_flag and self.scan_running:
                                print("Reader thread: Scan reported inactive by ESP32 (0 points).")
                                self.scan_running = False
                                if self.current_state == AFMState.SCANNING:
                                     self.set_idle()

                    except (IndexError, ValueError) as e:
                        print(f"Reader thread: Error parsing scan header ", line, ":", e)
                        reading_scan_data = False # Reset state on parse error
                        points_to_read = 0

                elif line.startswith("{") and line.endswith("}"):
                    # Looks like a JSON response
                    try:
                        json_response = json.loads(line)
                        self.response_queue.put(json_response)
                        # print(f"Reader thread: Queued response: {json_response}") # Debug
                    except json.JSONDecodeError:
                        print(f"Reader thread: Warning - JSON decode error for line: {line}")
                
                # Optional: Handle other types of messages if needed
                # elif line.startswith("# LOG:"): print(f"ESP32 LOG: {line[6:]}")

            except Exception as e:
                print(f"Reader thread: Error processing line ", line, ":", e)
                # Reset scan reading state on error to be safe
                reading_scan_data = False
                points_to_read = 0

        print("Serial reader thread finished.")

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
        """Set the AFM as idle, releasing the busy lock."""
        # Note: This method NO LONGER actively stops running processes.
        # Stopping should be handled by specific stop_xxx commands or
        # automatically when the ESP32 signals completion.
        with self.busy_lock:
            if not self.busy and self.current_state == AFMState.IDLE:
                return # Already idle
            
            print(f"Setting state to IDLE (was {self.current_state.name}). Busy flag: {self.busy}")
            self.busy = False
            # Update internal state if it wasn't already IDLE
            # This might be redundant if status updates already changed it, but ensures consistency
            if self.current_state != AFMState.IDLE:
                self.current_state = AFMState.IDLE
            # Ensure flags for specific operations are reset
            self.scan_running = False # If becoming idle, scan must be stopped
            self.focus_running = False
            # We assume approach state is managed by its own start/stop/get_data logic

    def is_busy(self) -> bool:
        """Check if AFM is busy."""
        with self.busy_lock:
            return self.busy

    def get_status(self) -> Optional[AFMStatus]:
        """Get current status from AFM device."""
        response = self.send_and_receive(
            {"command": "get_status"}, timeout=2.0)
        if response:
            try:
                # Check for all required fields
                required_fields = [
                    "timestamp", "adc_0", "adc_1", "dac_f", "dac_t", "dac_x", "dac_y", "dac_z",
                    "motor_1", "motor_2", "motor_3", "state"
                ]
                for field in required_fields:
                    if field not in response:
                        print(f"Missing required field in status response: {field}")
                        return None

                # Check motor data structure
                for motor_num in [1, 2, 3]:
                    motor_data = response.get(f"motor_{motor_num}", {})
                    if not all(key in motor_data for key in ["position", "target", "is_running"]):
                        print(f"Missing required motor data for motor {motor_num}")
                        return None

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
                        is_running=bool(get_motor_data(1).get("is_running", False))
                    ),
                    motor_2=MotorStatus(
                        position=int(get_motor_data(2).get("position", 0)),
                        target=int(get_motor_data(2).get("target", 0)),
                        is_running=bool(get_motor_data(2).get("is_running", False))
                    ),
                    motor_3=MotorStatus(
                        position=int(get_motor_data(3).get("position", 0)),
                        target=int(get_motor_data(3).get("target", 0)),
                        is_running=bool(get_motor_data(3).get("is_running", False))
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
                    if "adc_0" not in response or "adc_1" not in response:
                        print(f"Missing required ADC data in focus response: {response}")
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
    def approach(self, motor: int, step_size: int, threshold: int = 10, max_steps: int = None, speed: int = 500) -> bool:
        """Start the approach procedure on the ESP32.

        Args:
            motor (int): Motor number (1, 2, or 3)
            step_size (int): Number of steps to move in each iteration. Must be positive.
            threshold (int): Threshold value for ADC change
            max_steps (int): Maximum total steps to move. Positive for forward, negative for backward.
                            None for no limit.
            speed (int): Motor step delay in microseconds (default: 500).

        Returns:
            bool: True if approach started successfully, False otherwise
        """
        if not self.is_connected():
            return False

        if self.busy:
            return False

        # Validate parameters
        if motor not in [1, 2, 3]:
            return False
        if threshold <= 0:
            return False
        if step_size <= 0:
            return False
        if speed <= 0: # Also validate speed here
            print("Speed (motor delay) must be positive.")
            return False

        try:
            # Prepare command, including motor delay as speed
            command = {
                "command": "approach",
                "action": "start",
                "motor": motor,
                "step_size": step_size,
                "threshold": threshold,
                "max_steps": max_steps,
                "speed": speed # Use the provided speed parameter
            }

            # Send command
            response = self.send_and_receive(command)
            print(f"Approach response: {response}")
            if response and response.get("status") == "started":
                self.busy = True # Set busy only on successful start
                self.current_state = AFMState.APPROACHING # Update state
                return True
            else:
                # If start failed, ensure we are not marked as busy/approaching
                self.busy = False
                if self.current_state == AFMState.APPROACHING:
                    self.current_state = AFMState.IDLE # Reset internal state
                return False

        except Exception as e:
            print(f"Error starting approach: {e}")
            # Ensure state is reset on error
            self.busy = False
            if self.current_state == AFMState.APPROACHING:
                 self.current_state = AFMState.IDLE # Reset internal state
            return False

    def stop_approach(self) -> bool:
        """Stop the currently running approach procedure.

        Returns:
            bool: True if approach stopped successfully, False otherwise
        """
        # Store state before potentially changing it
        was_approaching = self.current_state == AFMState.APPROACHING

        if not self.is_connected():
             # Allow internal state reset even if not connected
             self.busy = False
             if was_approaching:
                  self.current_state = AFMState.IDLE # Reset internal state
             return True # Indicate internal stop

        # Check internal state first - more reliable than querying hardware constantly
        if not was_approaching and not self.busy:
             print("Approach not running (based on internal state)." )
             return True

        try:
            response = self.send_and_receive({
                "command": "approach",
                "action": "stop"
            })
            print(f"Stop approach response: {response}")

            # Always update internal state after sending stop
            self.busy = False
            if was_approaching:
                self.current_state = AFMState.IDLE # Reset internal state

            if response and response.get("status") == "stopped":
                return True
            else:
                print("Stop approach command sent, but hardware confirmation failed or was negative.")
                # Even if hardware failed, we stopped internally
                return False # Indicate potential hardware issue

        except Exception as e:
            print(f"Error stopping approach: {e}")
            # Ensure internal state is reset on error
            self.busy = False
            if was_approaching:
                self.current_state = AFMState.IDLE # Reset internal state
            return False

    def get_approach_data(self) -> Optional[List[Dict]]:
        """Get the approach data from the ESP32.

        Returns:
            Optional[List[Dict]]: List of approach data points, or None if failed
        """
        if not self.is_connected():
            return None
        # Only get data if we think we are approaching
        if self.current_state != AFMState.APPROACHING:
             # print("Not in APPROACHING state, skipping get_approach_data")
             return None

        try:
            response = self.send_and_receive({
                "command": "approach",
                "action": "get_data"
            })
            # print(f"Approach data response: {response}") # Optional debug
            if response and response.get("status") == "success":
                # Check if hardware indicates approach finished
                if not response.get("approach_running", True): # Assume running if key missing
                     print("Hardware indicated approach finished during get_data.")
                     # Update internal state if hardware says it stopped
                     if self.current_state == AFMState.APPROACHING:
                          self.busy = False
                          self.current_state = AFMState.IDLE # Reset internal state
                return response.get("data", [])
            else:
                print(f"Failed to get approach data or bad status: {response}")
                return None

        except Exception as e:
            print(f"Error getting approach data: {e}")
            return None

    def is_approach_running(self) -> bool:
        """Check if approach is currently active based on internal state."""
        # Relying on internal state updated by start/stop/get_data
        # This avoids potentially slow hardware queries in the GUI update loop
        is_running = self.current_state == AFMState.APPROACHING
        
        # Consistency check: if state is APPROACHING, busy should be true
        # if is_running and not self.busy:
        #      print("Warning: State is APPROACHING but busy flag is False.")
        #      # Potentially force self.busy = True here, or investigate why mismatch occurred
        
        # Consistency check: if state is not APPROACHING, busy might still be true
        # due to another operation. This function only cares about APPROACHING state.

        return is_running

    def enable_pid(self, target: Optional[int] = None) -> Optional[Dict]:
        """Enable PID control.

        Args:
            target (Optional[int]): Target ADC value. If None, current ADC value is used.

        Returns:
            Optional[Dict]: True if successful, False otherwise
        """
        if not self.is_connected():
            return None

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
            return None

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

    # Scan Methods
    def start_scan(self, x_start: int, x_end: int, y_start: int, y_end: int,
                     resolution: int, dwell_time: int) -> bool:
        """Start a scan procedure on the ESP32 (push model).

        Args:
            x_start (int): Starting DAC value for X-axis.
            x_end (int): Ending DAC value for X-axis.
            y_start (int): Starting DAC value for Y-axis.
            y_end (int): Ending DAC value for Y-axis.
            resolution (int): Number of points per axis (e.g., 256 for 256x256).
            dwell_time (int): Time in milliseconds to wait at each point.

        Returns:
            bool: True if scan started successfully, False otherwise.
        """
        if not self.is_connected():
            print("Not connected")
            return False
        if self.is_busy(): # Check generic busy flag
            print("AFM is busy. Cannot start scan.")
            return False

        # Basic validation (same as before)
        if not all(isinstance(v, int) for v in (x_start, x_end, y_start, y_end, resolution, dwell_time)):
            print("Scan range, resolution, and dwell time must be integers.")
            return False
        if x_start < 0 or x_start > 65535 or x_end < 0 or x_end > 65535 or \
           y_start < 0 or y_start > 65535 or y_end < 0 or y_end > 65535:
             print("DAC coordinates out of range (0-65535)")
             return False
        if x_start > x_end or y_start > y_end:
            print("Invalid range (start > end)")
            return False
        if resolution < 2:
            print("Resolution must be at least 2.")
            return False
        if dwell_time < 0:
            print("Dwell time must be non-negative.")
            return False

        # Acquire busy lock
        if not self.set_busy():
             print("Could not acquire busy lock for scan.")
             return False

        try:
            # Clear previous scan data and set parameters
            self.scan_data.clear()
            self.scan_resolution = resolution
            # Optimistically set state, confirmed by ESP32 later
            self.current_state = AFMState.SCANNING
            self.scan_running = True

            command = {
                "command": "scan",
                "action": "start",
                "x_start": x_start,
                "x_end": x_end,
                "y_start": y_start,
                "y_end": y_end,
                "resolution": resolution,
                "dwell_time": dwell_time,
            }

            # Send command and wait for confirmation
            response = self.send_and_receive(command, timeout=5.0) # Allow more time for scan setup
            
            if response and response.get("status") == "started":
                print("Scan started successfully.")
                return True
            else:
                print(f"Failed to start scan. Response: {response}")
                self.scan_resolution = 0 # Reset resolution on failure
                self.scan_running = False
                self.set_idle() # Release busy lock and reset internal state
                return False

        except Exception as e:
            print(f"Error starting scan: {e}")
            self.scan_resolution = 0 # Reset resolution on error
            self.scan_running = False
            if self.busy:
                self.set_idle() # Ensure cleanup on error
            return False

    def stop_scan(self) -> bool:
        """Stop the currently running scan procedure (push model)."""
        if not self.is_connected():
            print("Not connected, cannot send stop command.")
            # Mark as not running locally if needed
            if self.scan_running:
                print("Marking scan as stopped locally.")
                self.scan_running = False
                if self.current_state == AFMState.SCANNING:
                     self.set_idle() # Reset state if it was scanning
            return False # Indicate command wasn't sent

        # Only send stop command if we think it might be running
        if not self.scan_running and self.current_state != AFMState.SCANNING:
             print("Scan not running locally.")
             # Ensure idle state if busy flag somehow got stuck
             if self.busy:
                  self.set_idle()
             return True # Consider it stopped

        print("Sending stop scan command...")
        try:
            response = self.send_and_receive({
                "command": "scan",
                "action": "stop"
            })
            print(f"Stop scan response: {response}")

            # Regardless of response, mark as not running locally immediately
            self.scan_running = False
            
            # If response confirms stopped or is missing/error, transition to idle
            # The background reader might also do this if it sees an inactive flag
            if self.current_state == AFMState.SCANNING or self.busy:
                 self.set_idle() 

            if response and response.get("status") == "stopped":
                print("Scan stop confirmed by hardware.")
                return True
            else:
                print("Stop scan command sent, but hardware confirmation failed or was negative.")
                return False # Indicate potential hardware issue

        except Exception as e:
            print(f"Error sending stop scan command: {e}")
            # Ensure state is cleaned up even on exception
            self.scan_running = False
            if self.current_state == AFMState.SCANNING or self.busy:
                 self.set_idle()
            return False

    def is_scan_running(self) -> bool:
        """Check if scan procedure is currently believed to be active based on local flag."""
        # This now relies on the self.scan_running flag, 
        # which is updated by start/stop calls and the background reader/status updates.
        return self.scan_running

    def get_scan_image(self, channel_index=2) -> Optional[np.ndarray]: # Default to ADC0 (index 2 in CSV) 
        """Reconstructs the 2D scan image from the collected scan_data.

        Args:
            channel_index (int): The index within each data point list/tuple
                                 to use for the image pixel value (e.g., 0 for ADC0,
                                 1 for DACZ, etc., based on ESP32 data format).
                                 Defaults to 2 (index of adc0 in pushed CSV data x,y,adc0,dacz).

        Returns:
            Optional[np.ndarray]: A 2D numpy array representing the scan image,
                                  or None if no data/resolution is available.
        """
        if not self.scan_data or self.scan_resolution <= 0:
            return None
        
        # Access deque safely (though appends/pops are thread-safe, iteration isn't guaranteed)
        # Create a temporary list copy for stable iteration if needed, 
        # but for visualization, accessing the deque directly might be acceptable 
        # if updates are infrequent or minor artifacts are tolerable.
        # For simplicity here, we iterate directly. Consider locking or copying if issues arise.
        # with self.serial_lock: # Or a dedicated scan_data_lock
        #    local_scan_data = list(self.scan_data)
        local_scan_data = list(self.scan_data) # Create copy for safe iteration

        if not local_scan_data:
             return None

        # Initialize image array (use float for visualization flexibility)
        image = np.full((self.scan_resolution, self.scan_resolution), np.nan, dtype=np.float32)

        # Reconstruct image from flattened data
        flat_image_data = []
        for point_data in local_scan_data:
            # Check if point_data is a list/tuple and has enough elements
            if isinstance(point_data, (list, tuple)) and len(point_data) > channel_index:
                try:
                    flat_image_data.append(float(point_data[channel_index]))
                except (ValueError, TypeError):
                    flat_image_data.append(np.nan) # Use NaN on conversion error
            else:
                flat_image_data.append(np.nan) # Use NaN if data format issue or missing index

        # Ensure we don't exceed array bounds and fill the numpy array
        num_elements = min(len(flat_image_data), image.size)
        image.flat[:num_elements] = flat_image_data[:num_elements]

        # Return the reconstructed image (GUI will handle transposition)
        return image

    def get_scan_data_count(self) -> int:
        """Returns the number of scan data points currently stored."""
        return len(self.scan_data)


if __name__ == "__main__":
    afm = AFM()
    # --- Example Usage (Push Model) --- 
    if afm.connect():
        try:
            # Example: Start a scan
            print("Starting scan...")
            if afm.start_scan(x_start=10000, x_end=55000, y_start=10000, y_end=55000, resolution=16, dwell_time=5):
                print("Scan initiated.")
                # Monitor scan progress
                while afm.is_scan_running():
                    # Optionally check status periodically (less critical with push)
                    # afm.get_scan_data() # Removed call
                    print(f"Scan data points received: {afm.get_scan_data_count()} / {afm.scan_resolution**2}")
                    time.sleep(1) # Wait before checking again
                print("Scan finished or stopped.")
                print(f"Total points collected: {afm.get_scan_data_count()}")
                # Image reconstruction would happen here or in GUI
            else:
                print("Failed to start scan.")

            # Keep running to allow reader thread to potentially catch late data? 
            # Or just exit after scan.
            # print("Waiting for a bit...")
            # time.sleep(5)

        except KeyboardInterrupt:
            print("Stopping...")
            if afm.is_scan_running():
                afm.stop_scan()
        finally:
            print("Closing AFM connection.")
            afm.close()
    else:
        print("Failed to connect to AFM device.")
