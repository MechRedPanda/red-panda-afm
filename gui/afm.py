import serial
import threading
import time
import random
from dataclasses import dataclass
from datetime import datetime
import serial.tools.list_ports
import queue


@dataclass
class AFMStatus:
    adc_value: int
    dac_values: dict
    timestamp: datetime
    scan_data: list


class AFM:
    def __init__(self):
        self.port = None
        self.baud_rate = None
        self.serial_connection = None
        self.afm_status = AFMStatus(adc_value=0, dac_values={"T": 0, "F": 0},
                                    timestamp=datetime.now(), scan_data=[])
        self.scan_in_progress = False  # Flag to track scan progress
        self.scan_data_queue = queue.Queue()  # Thread-safe queue for scan data

    def connect(self, port, baud_rate=115200):
        """Connect to the AFM device using a serial connection."""
        self.port = port
        self.baud_rate = baud_rate
        try:
            self.serial_connection = serial.Serial(port, baud_rate, timeout=1)
            print(f"Successfully connected to {port} at {baud_rate} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.simulating = True

    def send_command(self, command):
        """Send a command to the AFM device."""
        if not self.port:
            print(f"[SIMULATION] Sending command: {command}")
        elif self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(f"{command}\n".encode("utf-8"))

    def read_response(self):
        """Read a response from the AFM device."""
        if not self.port:
            return ''
        elif self.serial_connection and self.serial_connection.is_open:
            try:
                return self.serial_connection.readline().decode("utf-8").strip()
            except Exception as e:
                print(f"Error reading from AFM: {e}")
                return None
        return None

    def control_dac(self, channel, value):
        """Control DAC channels."""
        if channel not in self.afm_status.dac_values:
            print(f"Invalid DAC channel: {channel}")
            return
        self.afm_status.dac_values[channel] = value
        self.send_command(f"DAC {channel} {value}")

    def read_adc(self):
        """Read the current ADC value."""
        if not self.port:
            return random.randint(2**15, 2**15 + 10000)
        self.send_command("A")
        response = self.read_response()
        try:
            self.afm_status.adc_value = int(response)
            return self.afm_status.adc_value
        except (ValueError, TypeError):
            print(f"Invalid ADC response: {response}")
            return None

    def set_dac(self, channel, value):
        """Set the DAC value for a specific channel with range checking."""
        if channel not in self.afm_status.dac_values:
            print(f"Invalid DAC channel: {channel}")
            return
        # Enforce 16-bit DAC range: 0 to 65535
        value = max(0, min(65535, value))
        self.afm_status.dac_values[channel] = value
        self.send_command(f"D {channel} {value}")

    def reset(self):
        """Reset the AFM device to its initial state."""
        if not self.port:
            print("[SIMULATION] Resetting AFM device.")
            return
        self.send_command("RESET")

    def start_scan(self, start, end, num_points):
        """Start a scan from start to end with a specified number of points."""
        self.scan_in_progress = True
        self.scan_thread = threading.Thread(
            target=self._scan, args=(start, end, num_points), daemon=True)
        self.scan_thread.start()

    def _scan(self, start, end, num_points):
        """Perform the scan in a separate thread."""
        step_size = (end - start) / (num_points - 1)

        if not self.port:
            for i in range(num_points):
                if not self.scan_in_progress:
                    break  # Exit early if scan is stopped
                step_value = int(start + i * step_size)
                adc_value = random.randint(2**15, 2**15 + 10000)
                self.scan_data_queue.put((step_value, adc_value))
                time.sleep(0.2)  # Simulate AFM stabilization time
            self.scan_in_progress = False
            return

        for i in range(num_points):
            if not self.scan_in_progress:
                break
            step_value = int(start + i * step_size)
            self.set_dac("F", step_value)
            time.sleep(0.1)  # Wait for the AFM to stabilize
            adc_value = self.read_adc()
            self.scan_data_queue.put((step_value, adc_value))
        self.scan_in_progress = False

    def stop_scan(self):
        """Stop the scan if it is running."""
        print("Stopping scan...")
        if hasattr(self, 'scan_thread') and self.scan_thread.is_alive():
            self.scan_in_progress = False
            self.scan_thread.join()
            print("Scan stopped.")
