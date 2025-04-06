from enum import Enum
import socket
import json
import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import time

ESP32_IP = "192.168.86.25"
ESP32_PORT = 12345
QUEUE_MAXLEN = 10000


class AFMState(Enum):
    IDLE = "IDLE"
    FOCUSING = "FOCUSING"
    APPROACHING = "APPROACHING"
    SCANNING = "SCANNING"


@dataclass
class AFMStatus:
    timestamp: datetime
    adc_0: float
    adc_1: float
    dac_f: float
    dac_t: float
    dac_x: float
    dac_y: float
    dac_z: float
    stepper_position_0: int
    state: AFMState = AFMState.IDLE  # Default state


class AFM:
    def __init__(self):
        self.host = ESP32_IP
        self.port = ESP32_PORT
        self.socket = None
        # Queue for storing AFM states
        self.status_queue = deque(maxlen=QUEUE_MAXLEN)
        self.busy = False  # AFM busy status
        self.focus_interrupted = False
        self.current_state = AFMState.IDLE  # Current state of the AFM
        self.focus_results = []  # Will hold (dac_f, adc_0, adc_1)

    def connect(self, host=ESP32_IP, port=ESP32_PORT):
        """Connect to the AFM device."""
        try:
            self.host = host
            self.port = port
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")

            # Start listening for updates in a separate thread
            self.running = True

            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            self.socket = None
            return False

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
        """Send a command to the ESP32."""
        if not self.socket:
            print("Not connected")
            return None
        try:
            command_json = json.dumps(command) + "\n"
            self.socket.sendall(command_json.encode('utf-8'))
        except Exception as e:
            print(f"Error sending command: {e}")

    def send_and_receive(self, command, timeout=1.0):
        """
        Send a command and wait for a response from the ESP32.

        Args:
            command: The command dictionary to send.
            timeout: Maximum time to wait for a response (seconds).

        Returns:
            The received raw response (bytes) or None if timeout.
        """
        if not self.socket:
            print("Not connected")
            return None

        try:
            # Clear receive buffer before sending the command
            self.socket.setblocking(False)
            while True:
                try:
                    self.socket.recv(1024)  # Read any leftover data
                except BlockingIOError:
                    break

            # Send the command
            command_json = json.dumps(command) + "\n"
            self.socket.sendall(command_json.encode('utf-8'))

            # Wait for response, checking if a full response is received
            self.socket.settimeout(timeout)
            buffer = ""
            while True:
                try:
                    data = self.socket.recv(1024)  # Receive data from socket
                    if not data:
                        break

                    buffer += data.decode('utf-8')

                    # If a complete message is received, parse and return it
                    while "\n" in buffer:
                        # Extract first complete message
                        message, buffer = buffer.split("\n", 1)

                        try:
                            json_data = json.loads(
                                message.strip())  # Parse JSON data
                            return json_data if json_data else None
                        except json.JSONDecodeError as e:
                            print(
                                f"Error parsing response: {e}, received: {message}")
                            continue

                except socket.timeout:
                    print("Response timeout")
                    break
                except Exception as e:
                    print(f"Error during response handling: {e}")
                    break

        except Exception as e:
            print(f"Communication error: {e}")
            return None
        finally:
            self.socket.setblocking(True)

    def get_status(self):
        response = self.send_and_receive(
            {"command": "get_status"}, timeout=2.0)
        print(f"Status response: {response}")

    def reset(self):
        """Reset the AFM device."""
        self.send_and_receive({"command": "reset"})

    def set_dac(self, channel: str, value: float):
        """Set the DAC value for a specific channel."""
        if channel not in ["F", "T", "X", "Y", "Z"]:
            raise ValueError("Invalid channel. Must be one of: F, T, X, Y, Z.")
        self.send_and_receive(
            {"command": "set_dac", "channel": channel, "value": value})

    def set_state(self, state: AFMState):
        """Set the AFM state."""
        if not isinstance(state, AFMState):
            raise ValueError("Invalid state. Must be an instance of AFMState.")
        self.send_and_receive({"command": "set_state", "state": state.name})

    def close(self):
        """Close the connection."""
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
            print("Connection closed.")

    def focus(self, start: int, end: int, step_size: int, delay: float = 0.01):
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
        if delay < 0:
            raise ValueError("Delay must be non-negative.")
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

                    # Send command and get raw response
                    response = self.send_and_receive(
                        {"command": "set_dac", "channel": "F", "value": dac_f_value},
                        timeout=0.5
                    )

                    if response is None:
                        continue
                    response = self.send_and_receive(
                        {"command": "read_adc"},
                        timeout=0.5
                    )
                    if response is None:
                        continue
                    adc_0 = int(response.get("adc_0", 0))
                    adc_1 = int(response.get("adc_1", 0))

                    # Store results with raw response
                    self.focus_results.append((dac_f_value, adc_0, adc_1))

            finally:
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
        while True:
            # afm.get_status()
            print(afm.send_and_receive({"command": "read_adc"}))
            time.sleep(1)
