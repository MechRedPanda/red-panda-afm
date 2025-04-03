import socket
import json
import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime

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
            command_json = json.dumps({"command": command}) + "\n"
            self.socket.sendall(command_json.encode('utf-8'))
        except Exception as e:
            print(f"Error sending command: {e}")

    def reset(self):
        """Reset the AFM device."""
        self.send_command("reset")

    def close(self):
        """Close the connection."""
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
            print("Connection closed.")


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
