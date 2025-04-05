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


class AFM:
    def __init__(self):
        self.host = ESP32_IP
        self.port = ESP32_PORT
        self.socket = None
        # Queue for storing AFM states
        self.status_queue = deque(maxlen=QUEUE_MAXLEN)
        self.busy = False  # AFM busy status
        self.focus_interrupted = False
        self.focus_running = False
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
            threading.Thread(target=self._receive_updates, daemon=True).start()

            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            self.socket = None
            return False

    def _receive_updates(self):
        """Continuously receive AFM state updates from ESP32 and store them in a queue."""
        buffer = ""  # Buffer to store partial JSON messages

        while self.running:
            if self.busy:  # Skip receiving updates if AFM is busy
                continue

            try:
                data = self.socket.recv(1024)  # Receive data from socket
                if not data:
                    break

                # Append received data to buffer
                buffer += data.decode('utf-8')

                # Process all complete JSON objects in buffer
                while "\n" in buffer:
                    message, buffer = buffer.split(
                        "\n", 1)  # Extract first JSON object

                    try:
                        json_data = json.loads(message.strip())  # Parse JSON
                        if json_data.get("data_type") != "update":
                            # Skip if not a status message
                            continue

                        # Parse JSON data into AFMStatus dataclass
                        status_entry = AFMStatus(
                            timestamp=datetime.now(),
                            adc_0=float(json_data.get("adc_0", 0.0)),
                            adc_1=float(json_data.get("adc_1", 0.0)),
                            dac_f=float(json_data.get("dac_f", 0.0)),
                            dac_t=float(json_data.get("dac_t", 0.0)),
                            dac_x=float(json_data.get("dac_x", 0.0)),
                            dac_y=float(json_data.get("dac_y", 0.0)),
                            dac_z=float(json_data.get("dac_z", 0.0)),
                            stepper_position_0=int(
                                json_data.get("stepper_position_0", 0)),
                        )

                        self.status_queue.append(status_entry)
                        print(
                            f"Received AFM State at {status_entry.timestamp}: {status_entry}")

                    except (json.JSONDecodeError, ValueError, TypeError) as e:
                        print(
                            f"Error parsing JSON data: {e}, Received: {message}")

            except socket.timeout:
                pass
            except Exception as e:
                print(f"Receive error: {e}")
                break

    def set_busy(self):
        """Set the AFM as busy, pausing updates."""
        print("AFM is now busy. Updates paused.")
        self.busy = True

    def set_idle(self):
        """Set the AFM as idle, resuming updates."""
        print("AFM is now idle. Updates resumed.")
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

    def reset(self):
        """Reset the AFM device."""
        self.send_command({"command": "reset"})

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
                for dac_f_value in steps:
                    if self.focus_interrupted:
                        print("Focus sweep interrupted.")
                        break

                    self.send_command(
                        {"command": "set_dac", "channel": "F", "value": dac_f_value})
                    time.sleep(delay)

                    # Get ADC values from last known status
                    if self.status_queue:
                        last_status = self.status_queue[-1]
                        adc_0 = getattr(last_status, "adc_0", None)
                        adc_1 = getattr(last_status, "adc_1", None)
                    else:
                        adc_0 = adc_1 = None

                    self.focus_results.append((dac_f_value, adc_0, adc_1))

                print("Focus sweep complete.")

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
        while True:
            user_input = input(
                "Enter command (or 'exit' to quit, 'busy' to pause updates, 'idle' to resume): ")
            if user_input.lower() == "exit":
                break
            elif user_input.lower() == "busy":
                afm.set_busy()
            elif user_input.lower() == "idle":
                afm.set_idle()
            else:
                afm.send_command(user_input)

        afm.close()
