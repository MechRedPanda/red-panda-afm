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
import csv
import struct # Needed for unpacking binary data

# Default serial port settings
DEFAULT_PORT = None  # Will auto-detect if None
BAUDRATE = 921600
TIMEOUT = 1
QUEUE_MAXLEN = 10000
SERIAL_READ_TIMEOUT = 0.1 # Timeout for serial read attempts in background thread
RESPONSE_TIMEOUT = 2.0 # Default timeout for waiting for a command response
SCAN_SEND_CHUNK_SIZE = 32 # Matches firmware setting for max points per binary chunk


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

    # Control loop timing (from firmware)
    loop_last_period_us: int = 0
    loop_last_freq_hz: float = 0.0
    loop_avg_freq_hz: float = 0.0


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
        
        # Modified for separate trace/retrace storage
        self.scan_data_trace = deque()  # Store trace (first pass) scan data
        self.scan_data_retrace = deque()  # Store retrace (second pass) scan data
        self.scan_data = deque()  # Maintain original for backward compatibility
        
        self.scan_running = False
        self._scan_event = threading.Event()  # Added for potential future use
        self.scan_resolution = 0  # Store resolution of the current/last scan
        
        # Initialize scan boundary attributes
        self.scan_x_start = 0
        self.scan_x_end = 0
        self.scan_y_start = 0
        self.scan_y_end = 0

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

    # --- Background Serial Reader Thread (Modified for Binary Data) ---
    def _serial_read_loop(self):
        """Background thread to continuously read serial data (JSON, CSV, Text)."""
        print("Serial reader thread started (CSV Scan Data Mode).")
        # Removed binary format definitions
        read_buffer = b''

        while not self._stop_read_thread.is_set():
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    with self.serial_lock:
                        data_to_read = self.serial_conn.in_waiting
                        if data_to_read > 0:
                             read_buffer += self.serial_conn.read(data_to_read)
                        else:
                             time.sleep(0.005)
                             continue
                else:
                    print("Reader thread: Connection lost.")
                    break
            except serial.SerialException as e:
                print(f"Reader thread: Serial error: {e}")
                self.close()
                break
            except Exception as e:
                print(f"Reader thread: Unexpected error reading serial: {e}")
                time.sleep(0.1)
                continue

            processed_bytes = 0
            buffer_len = len(read_buffer)

            while processed_bytes < buffer_len:
                # --- Check for newline-terminated lines ---
                newline_pos = read_buffer.find(b'\n', processed_bytes)
                if newline_pos != -1:
                    line_bytes = read_buffer[processed_bytes : newline_pos]
                    processed_bytes = newline_pos + 1

                    try:
                        line_str = line_bytes.decode('utf-8').strip()
                        if not line_str:
                            continue

                        # --- Check line type ---
                        
                        # 1. Try parsing as CSV scan data (heuristic: starts with digit, has 4 commas)
                        if line_str and line_str[0].isdigit() and line_str.count(',') == 4:
                            try:
                                parts = line_str.split(',')
                                if len(parts) == 5:
                                    # Convert to integers
                                    point_data = [int(p) for p in parts]
                                    x, y, adc0, dacz, flags = point_data
                                    
                                    # Store data
                                    self.scan_data.append(point_data)
                                    is_retrace = (flags == 2) # Check retrace flag
                                    if is_retrace:
                                        self.scan_data_retrace.append(point_data)
                                    else:
                                        self.scan_data_trace.append(point_data)
                                    continue # Handled as CSV scan data
                                else:
                                     # Malformed CSV, fall through to other checks
                                     pass 
                            except ValueError:
                                # Conversion to int failed, not valid scan data, fall through
                                pass
                            except Exception as csv_e:
                                print(f"Reader thread: Error processing potential CSV: {csv_e} - Line: {line_str}")
                                continue # Skip malformed potential CSV

                        # 2. Try parsing as JSON
                        try:
                            json_data = json.loads(line_str)

                            # Special case: Scan completion message
                            if json_data.get("command") == "scan" and json_data.get("status") == "complete":
                                print("Reader thread: Received scan completion message!")
                                self.scan_running = False
                                self.current_state = AFMState.IDLE 
                                self.set_idle() 
                                continue # Handled scan complete message

                            # Other valid JSON -> response queue
                            else:
                                self.response_queue.put(json_data)
                                continue # Queued JSON response

                        except json.JSONDecodeError:
                            # Not JSON: Treat as debug/unrecognized text
                            if line_str.startswith("# DEBUG:"):
                                print(f"Reader thread: Debug msg: {line_str[8:]}")
                            else:
                                print(f"Reader thread: Unrecognized text line (not CSV or JSON): {line_str}")
                            continue # Handled as text

                    except UnicodeDecodeError:
                        print(f"Reader thread: Could not decode bytes as UTF-8: {line_bytes}")
                        continue
                    except Exception as e:
                         print(f"Reader thread: Error processing line: {e} - Line: {line_bytes}")
                         continue

                # --- No newline found in buffer --- 
                else:
                    # Need more data to complete a line or potential binary block (which we now ignore)
                    break # Break inner loop to read more data

            # Update buffer by removing processed bytes
            if processed_bytes > 0:
                read_buffer = read_buffer[processed_bytes:]

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
            
            # We intentionally don't reset scan boundaries (scan_x_start, scan_x_end, etc.)
            # here to allow image retrieval after a scan has completed

    def is_busy(self) -> bool:
        """Check if AFM is busy."""
        with self.busy_lock:
            return self.busy

    def get_status(self) -> Optional[AFMStatus]:
        """Get current status from AFM device by sending command and parsing response."""
        response = self.send_and_receive(
            {"command": "get_status"}, timeout=2.0)
        if response and isinstance(response, dict):
            try:
                # --- Parsing logic moved here from _serial_read_loop ---
                required_fields = [
                    "timestamp", "adc_0", "adc_1", "dac_f", "dac_t", "dac_x", "dac_y", "dac_z",
                    "motor_1", "motor_2", "motor_3", "state"
                ]
                if not all(field in response for field in required_fields):
                    print(f"Missing required field in status response: {response}")
                    return None

                # Check motor data structure
                for motor_num in [1, 2, 3]:
                    motor_data = response.get(f"motor_{motor_num}", {})
                    if not all(key in motor_data for key in ["position", "target", "is_running"]):
                        print(f"Missing required motor data in status for motor {motor_num}: {response}")
                        return None

                def get_motor_data(num: int) -> Dict:
                    # Helper to safely get motor data, defaulting to empty dict
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
                    state=AFMState(response.get("state", "IDLE")),
                    # Parse new timing fields (period and frequencies)
                    loop_last_period_us=int(response.get("loop_last_period_us", 0)),
                    loop_last_freq_hz=float(response.get("loop_last_freq_hz", 0.0)),
                    loop_avg_freq_hz=float(response.get("loop_avg_freq_hz", 0.0))
                )
                # --- End of moved parsing logic ---

                self.status_queue.append(status) # Keep history in status_queue
                self.current_state = status.state # Update AFM's internal state

                print(f"Status: {status.loop_avg_freq_hz:.2f} Hz") # Updated print
                return status

            except Exception as e:
                print(f"Error parsing AFM status JSON in get_status: {e}")
                print(f"Raw response dictionary: {response}")
        elif response:
            # Handle cases where send_and_receive might return non-dict data (shouldn't happen with current loop)
            print(f"Received non-dictionary response for get_status: {response}")

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
                # Directly update the state attribute
                self.current_state = AFMState.FOCUSING 

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
        
        time.sleep(2) # Wait for the AFM to be ready

        try:
            # Clear previous scan data and set parameters
            self.scan_data.clear()
            self.scan_data_trace.clear()  # Clear trace data
            self.scan_data_retrace.clear()  # Clear retrace data
            self.scan_resolution = resolution
            # Store scan boundaries for image reconstruction
            self.scan_x_start = x_start
            self.scan_x_end = x_end
            self.scan_y_start = y_start
            self.scan_y_end = y_end
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
                # Reset scan boundaries on failure
                self.scan_x_start = 0
                self.scan_x_end = 0
                self.scan_y_start = 0
                self.scan_y_end = 0
                self.set_idle() # Release busy lock and reset internal state
                return False

        except Exception as e:
            print(f"Error starting scan: {e}")
            self.scan_resolution = 0 # Reset resolution on error
            self.scan_running = False
            # Reset scan boundaries on error
            self.scan_x_start = 0
            self.scan_x_end = 0
            self.scan_y_start = 0
            self.scan_y_end = 0
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
                # We don't reset scan boundaries here to allow getting scan images after stopping
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

    def get_scan_image(self, channel_index=0, scan_type="trace", save_raw=False):
        """Get scan image data for a specific channel and scan type.
        
        Args:
            channel_index (int): Index of the channel to get image for (0-3)
            scan_type (str): "trace" or "retrace"
            save_raw (bool): If True, save raw data to CSV files
            
        Returns:
            numpy.ndarray: 2D array of image data
        """
        if not self.scan_data:
            return None
            
        # Get data for the specified scan type
        if scan_type == "trace":
            data = list(self.scan_data_trace)
        elif scan_type == "retrace":
            data = list(self.scan_data_retrace)
        else:
            print(f"Invalid scan type: {scan_type}")
            return None
            
        if not data:
            return None
            
        # Convert to numpy array for easier processing
        data = np.array(data)
        
        # Extract coordinates and channel data
        x_coords = data[:, 0]
        y_coords = data[:, 1]
        channel_data = data[:, channel_index + 2]  # +2 to skip x,y coordinates
        
        # Get scan parameters
        x_start = self.scan_x_start
        x_end = self.scan_x_end
        y_start = self.scan_y_start
        y_end = self.scan_y_end
        resolution = self.scan_resolution
        
        # Calculate pixel coordinates using rounding
        x_pixels = np.round((x_coords - x_start) / (x_end - x_start) * (resolution - 1)).astype(int)
        y_pixels = np.round((y_coords - y_start) / (y_end - y_start) * (resolution - 1)).astype(int)
        
        # Create image array
        image = np.full((resolution, resolution), np.nan)
        
        # Track duplicate points
        duplicate_points = {}
        valid_points = 0
        
        # Fill image with data
        for i in range(len(data)):
            x, y = x_pixels[i], y_pixels[i]
            if 0 <= x < resolution and 0 <= y < resolution:
                key = (x, y)
                if key in duplicate_points:
                    duplicate_points[key].append((x_coords[i], y_coords[i], channel_data[i]))
                else:
                    duplicate_points[key] = [(x_coords[i], y_coords[i], channel_data[i])]
                image[y, x] = channel_data[i]
                valid_points += 1
        
        # Print detailed debugging information
        print(f"\nProcessing {len(data)} data points for {scan_type} image")
        print(f"X range in data: {x_coords.min()} to {x_coords.max()}, Y range: {y_coords.min()} to {y_coords.max()}")
        print(f"Image pixels - Valid: {valid_points}/{len(data)} ({valid_points/len(data)*100:.1f}%)")
        print(f"Pixel mapping - Unique pixels: {len(duplicate_points)}, Duplicate mappings: {sum(len(v) > 1 for v in duplicate_points.values())}")
        print(f"Max points per pixel: {max(len(v) for v in duplicate_points.values())}")
        
        # Print distribution of points per pixel
        point_dist = {}
        for points in duplicate_points.values():
            count = len(points)
            point_dist[count] = point_dist.get(count, 0) + 1
        print(f"Points per pixel distribution: {point_dist}")
        
        # Print details of duplicate points
        print("\nDuplicate points details:")
        for pixel, points in duplicate_points.items():
            if len(points) > 1:
                print(f"\nPixel ({pixel[0]}, {pixel[1]}) has {len(points)} points:")
                for i, (x, y, value) in enumerate(points):
                    print(f"  Point {i+1}: X={x}, Y={y}, Value={value}")
        
        # Find missing pixels
        missing_pixels = []
        for y in range(resolution):
            for x in range(resolution):
                if np.isnan(image[y, x]):
                    missing_pixels.append((x, y))
        
        
        # Save raw data if requested
        if save_raw:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"last_scan_{scan_type}_{timestamp}.csv"
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['X', 'Y', 'Channel0', 'Channel1', 'Channel2', 'Channel3'])
                for row in data:
                    writer.writerow(row)
            print(f"Saved raw {scan_type} data to {filename}")
        
        return image

    def get_scan_data_count(self, scan_type="combined") -> int:
        """Returns the number of scan data points currently stored.
        
        Args:
            scan_type (str): Type of scan data to count - "trace", "retrace", or "combined"
                            Defaults to "combined" for backward compatibility.
        
        Returns:
            int: Number of data points in the specified collection
        """
        if scan_type == "trace":
            count = len(self.scan_data_trace)
            return count
        elif scan_type == "retrace":
            count = len(self.scan_data_retrace)
            return count
        else:  # "combined" or any other value
            count = len(self.scan_data)
            return count
            
    def get_scan_data_summary(self) -> Dict:
        """Returns a summary of scan data counts and status.
        
        Returns:
            Dict: Dictionary with counts of different scan data types and scan status
        """
        trace_count = len(self.scan_data_trace)
        retrace_count = len(self.scan_data_retrace)
        combined_count = len(self.scan_data)
        
        total_expected = self.scan_resolution**2 * 2 if self.scan_resolution > 0 else 0
        
        return {
            "trace_count": trace_count,
            "retrace_count": retrace_count,
            "combined_count": combined_count,
            "resolution": self.scan_resolution,
            "total_expected": total_expected,
            "is_running": self.scan_running,
            "progress_percent": (combined_count / total_expected * 100) if total_expected > 0 else 0,
            "scan_state": self.current_state.name
        }


if __name__ == "__main__":
    afm = AFM()
    # --- Example Usage (Push Model) --- 
    port = afm.get_serial_ports()[0]
    if afm.connect(port.device):
        try:
            # Example: Start a scan
            print("Starting scan...")
            if afm.start_scan(x_start=10000, x_end=55000, y_start=10000, y_end=55000, resolution=128, dwell_time=2):
                print("Scan initiated.")
                # Monitor scan progress
                while afm.is_scan_running():
                    # Optionally check status periodically (less critical with push)
                    # afm.get_scan_data() # Removed call
                    print(f"Scan data points received: {afm.get_scan_data_count()} / {2*afm.scan_resolution**2}")
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

            # Get the scan image
            print(len(afm.scan_data))
            scan_image = afm.get_scan_image(channel_index=2, scan_type="trace")
            print(f"Scan image shape: {scan_image.shape}")

        except KeyboardInterrupt:
            print("Stopping...")
            if afm.is_scan_running():
                afm.stop_scan()
        finally:
            print("Closing AFM connection.")
            afm.close()
    else:
        print("Failed to connect to AFM device.")
