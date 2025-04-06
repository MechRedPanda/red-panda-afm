from enum import Enum
import json
import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import time
import serial
from serial.tools import list_ports
from typing import Dict, Optional, Union

# Default serial port settings
DEFAULT_PORT = None  # Will auto-detect if None
BAUDRATE = 115200
TIMEOUT = 1
QUEUE_MAXLEN = 10000


class AFMState(Enum):
    IDLE = "IDLE"
    FOCUSING = "FOCUSING"
    APPROACHING = "APPROACHING"
    SCANNING = "SCANNING"


@dataclass
class AFMStatus:
    timestamp: int
    adc_0: int
    adc_1: int
    dac_f: int
    dac_t: int
    dac_x: int
    dac_y: int
    dac_z: int
    stepper_position_0: int
    state: AFMState = AFMState.IDLE  # Default state


class AFM:
    def __init__(self):
        self.serial_port = DEFAULT_PORT
        self.baudrate = BAUDRATE
        self.timeout = TIMEOUT
        self.serial_ports = []
        self.serial_conn = None
        # Queue for storing AFM states
        self.status_queue = deque(maxlen=QUEUE_MAXLEN)
        self.busy = False  # AFM busy status
        self.focus_interrupted = False
        self.current_state = AFMState.IDLE  # Current state of the AFM
        self.focus_results = []  # Will hold (dac_f, adc_0, adc_1)
        self.focus_running = False

    def get_serial_ports(self):
        """Get a list of available serial ports."""
        self.serial_ports = list_ports.comports()
        return [port for port in self.serial_ports]

    def find_afm_device(self):
        """Try to automatically find the AFM device by checking serial ports."""
        ports = list_ports.comports()
        for port in ports:
            # You might need to adjust these checks based on your device
            if "USB" in port.description or "Serial" in port.description:
                return port.device
        return None

    def connect(self, port=None, baudrate=BAUDRATE, timeout=TIMEOUT):
        """Connect to the AFM device via serial."""
        try:
            self.baudrate = baudrate
            self.timeout = timeout

            # If no port specified, try to auto-detect
            if port is None:
                print(
                    "Could not auto-detect AFM device. Please specify port manually.")
                return False

            self.serial_port = port
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )

            # Clear any existing data in buffers
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            print(f"Connected to {self.serial_port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            self.serial_conn = None
            return False

    def is_connected(self):
        """Check if the AFM is connected."""
        return self.serial_conn is not None and self.serial_conn.is_open

    def set_busy(self):
        """Set the AFM as busy, pausing updates."""
        print("AFM is now busy. Updates paused.")
        self.busy = True

    def set_idle(self):
        """Set the AFM as idle, resuming updates."""
        print("AFM is now idle. Updates resumed.")
        self.set_state(AFMState.IDLE)
        self.busy = False

    def send_command(self, command):
        """Send a command to the AFM device."""
        if not self.serial_conn:
            print("Not connected")
            return None
        try:
            command_json = json.dumps(command) + "\n"
            self.serial_conn.write(command_json.encode('utf-8'))
        except Exception as e:
            print(f"Error sending command: {e}")

    def send_and_receive(self, command, timeout=None):
        """
        Send a command and wait for a response from the AFM device.

        Args:
            command: The command dictionary to send.
            timeout: Maximum time to wait for a response (seconds).

        Returns:
            The received JSON response (dict) or None if timeout.
        """
        if not self.serial_conn:
            print("Not connected")
            return None

        try:
            # Set timeout if specified
            original_timeout = self.serial_conn.timeout
            if timeout is not None:
                self.serial_conn.timeout = timeout

            # Clear input buffer before sending
            self.serial_conn.reset_input_buffer()

            # Send the command
            command_json = json.dumps(command) + "\n"
            self.serial_conn.write(command_json.encode('utf-8'))

            # Read response
            response = self.serial_conn.readline().decode('utf-8').strip()

            # Restore original timeout
            if timeout is not None:
                self.serial_conn.timeout = original_timeout

            if response:
                try:
                    return json.loads(response)
                except json.JSONDecodeError as e:
                    print(f"Error parsing response: {e}, received: {response}")
                    return None
            return None

        except Exception as e:
            print(f"Communication error: {e}")
            return None

    def get_status(self):
        """Get current status from AFM device."""
        response = self.send_and_receive(
            {"command": "get_status"}, timeout=2.0)
        if response:
            try:
                status = AFMStatus(
                    timestamp=int(response.get("timestamp", 0)),
                    adc_0=int(response.get("adc_0", 0)),
                    adc_1=int(response.get("adc_1", 0)),
                    dac_f=int(response.get("dac_f", 0)),
                    dac_t=int(response.get("dac_t", 0)),
                    dac_x=int(response.get("dac_x", 0)),
                    dac_y=int(response.get("dac_y", 0)),
                    dac_z=int(response.get("dac_z", 0)),
                    stepper_position_0=int(
                        response.get("stepper_position_0", 0)),
                    state=AFMState(response.get("state", "IDLE"))
                )
                self.status_queue.append(status)
                self.current_state = status.state
            except Exception as e:
                print(f"Error creating AFMStatus: {e}")
        return response

    def reset(self):
        """Reset the AFM device."""
        return self.send_and_receive({"command": "reset"})

    def set_dac(self, channel: str, value: float):
        """Set the DAC value for a specific channel."""
        if channel not in ["F", "T", "X", "Y", "Z"]:
            raise ValueError("Invalid channel. Must be one of: F, T, X, Y, Z.")
        return self.send_and_receive(
            {"command": "set_dac", "channel": channel, "value": value})

    def move_motor(self,
                   motor: int,
                   steps: int,
                   speed: int = 500) -> Dict:
        """
        Start non-blocking motor movement

        Args:
            motor: Motor number (1-3)
            steps: integer steps
            speed: Step delay in microseconds (100-5000, default 500)

        Returns:
            Dictionary with status and motor information
        """
        # Input validation
        if motor not in {1, 2, 3}:
            raise ValueError("Motor must be 1, 2, or 3")
        if not isinstance(steps, int) or steps <= 0:
            raise ValueError("Steps must be positive integer")
        direction = 0 if steps < 0 else 1 if steps > 0 else 1
        steps = abs(steps)  # Use absolute value for steps

        speed = max(100, min(5000, speed))  # Constrain speed

        reponse = self.send_and_receive({
            'command': 'start_motor',
            'motor': motor,
            'steps': steps,
            'direction': direction,
            'speed': speed
        })
        print(reponse)
        return reponse

    def stop_motor(self, motor: Optional[int] = None) -> Dict:
        """
        Stop motor movement

        Args:
            motor: Specific motor to stop (1-3) or None to stop all

        Returns:
            Dictionary with status
        """
        cmd = {'command': 'stop_motor'}
        if motor in {1, 2, 3}:
            cmd['motor'] = motor
        return self.send_and_receive(cmd)

    def get_motor_status(self, motor: Optional[int] = None) -> Dict:
        """
        Get motor status

        Args:
            motor: Specific motor (1-3) or None for all motors

        Returns:
            Dictionary with status information
        """
        cmd = {'command': 'get_motor_status'}
        if motor in {1, 2, 3}:
            cmd['motor'] = motor
        return self.send_and_receive(cmd)

    def is_moving(self, motor: Optional[int] = None) -> bool:
        """
        Check if motor(s) are moving

        Args:
            motor: Specific motor (1-3) or None to check any motor

        Returns:
            bool: True if specified motor (or any motor) is moving
        """
        status = self.get_motor_status(motor)
        if motor is None:
            return status.get('is_moving', False)
        return status.get('motors', [{}])[0].get('status', 'IDLE') == 'MOVING'

    def set_state(self, state: AFMState):
        """Set the AFM state."""
        if not isinstance(state, AFMState):
            raise ValueError("Invalid state. Must be an instance of AFMState.")
        return self.send_and_receive({"command": "set_state", "state": state.name})

    def close(self):
        """Close the connection."""
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
            print("Connection closed.")

    def focus(self, start: int, end: int, step_size: int):
        """
        Start a DAC sweep on dac_f. Results are available in self.focus_results.
        The sweep is interruptible and runs in a background thread.
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
                for dac_f_value in steps:
                    if self.focus_interrupted:
                        print("Focus sweep interrupted.")
                        break

                    # Set DAC value
                    response = self.send_and_receive(
                        {"command": "set_dac", "channel": "F", "value": dac_f_value},
                        timeout=0.5
                    )

                    if response is None:
                        continue

                    # Read ADC values
                    response = self.send_and_receive(
                        {"command": "read_adc"},
                        timeout=0.5
                    )
                    if response is None:
                        continue

                    adc_0 = int(response.get("adc_0", 0))
                    adc_1 = int(response.get("adc_1", 0))

                    # Store results
                    self.focus_results.append((dac_f_value, adc_0, adc_1))

            finally:
                self.focus_running = False
                self.set_idle()

        threading.Thread(target=run_focus, daemon=True).start()
        return True

    def stop_focus(self):
        """Request the currently running focus sweep to stop."""
        self.focus_interrupted = True

    def is_focus_running(self):
        """Check if focus sweep is currently active."""
        return self.focus_running


if __name__ == "__main__":
    afm = AFM()
    if afm.connect():
        try:
            while True:
                # Example usage: read ADC values continuously
                # response = afm.send_and_receive({"command": "read_adc"})
                response = afm.get_status()
                print(response)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            afm.close()
