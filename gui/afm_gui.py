import serial
import threading
import time
import random
from tkinter import ttk, Tk
import tkinter as tk  # For StringVar and other tkinter objects
from dataclasses import dataclass
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial.tools.list_ports
from collections import deque
import queue

from afm import AFM

# ---------------------------
# AFM Classes and Plotting
# ---------------------------


class RealtimePlot:
    def __init__(self, afm, data_type="adc"):
        self.afm = afm
        self.data = deque(maxlen=100)
        self.paused = False
        self.data_type = data_type
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        self.ax.set_title("Real-Time ADC Data")
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("ADC Value")
        self.set_plot_limits()

    def set_plot_limits(self):
        self.ax.set_xlim(0, 100)

    def start_plot(self):
        ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        plt.show()

    def toggle_pause(self):
        self.paused = not self.paused

    def update_plot(self, frame):
        if self.paused:
            return self.line,
        self.data.append(self.afm.read_adc())
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.set_xlim(0, max(100, len(self.data)))
        self.ax.set_ylim(min(self.data) - 10, max(self.data) + 10)
        return self.line,


class ScanPlot:
    def __init__(self, afm):
        self.afm = afm
        self.data = []
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        self.ax.set_title("AFM Scan Data")
        self.ax.set_xlabel("Step")
        self.ax.set_ylabel("ADC Value")
        self.set_plot_limits()

    def set_plot_limits(self):
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 1024)

    def update_plot(self, frame):
        while not self.afm.scan_data_queue.empty():
            step_value, adc_value = self.afm.scan_data_queue.get()
            self.data.append((step_value, adc_value))
        if self.data:
            dac_values, adc_values = zip(*self.data)
            self.line.set_data(dac_values, adc_values)
            self.ax.set_xlim(min(dac_values) - 10, max(dac_values) + 10)
            self.ax.set_ylim(min(adc_values) - 10, max(adc_values) + 10)
        return self.line,

    def start_real_time_plot(self):
        ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        plt.show()


# ---------------------------
# Refactored GUI Class
# ---------------------------

class AFM_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AFM Controller GUI")
        self.afm = AFM()
        self.create_widgets()

    def create_widgets(self):
        # We build separate panels for each semantic group:
        self.create_serial_panel()
        self.create_afm_control_panel()
        self.create_plot_control_panel()
        self.create_scan_control_panel()
        self.create_dac_control_panel()

    def create_serial_panel(self):
        # Panel for serial port settings
        serial_frame = ttk.LabelFrame(self.root, text="Serial Port Settings")
        serial_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(serial_frame, text="Port:").grid(
            row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_combobox = ttk.Combobox(
            serial_frame, state="readonly", width=20)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.refresh_ports()

        self.refresh_button = ttk.Button(
            serial_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=5, pady=5, sticky="w")

        ttk.Label(serial_frame, text="Baud Rate:").grid(
            row=1, column=0, padx=5, pady=5, sticky="w")
        self.baud_entry = ttk.Entry(serial_frame, width=10)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")

    def create_afm_control_panel(self):
        # Panel for general AFM controls (e.g., Connect and Reset)
        afm_ctrl_frame = ttk.LabelFrame(self.root, text="AFM Controls")
        afm_ctrl_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        self.connect_button = ttk.Button(
            afm_ctrl_frame, text="Connect", command=self.connect_afm)
        self.connect_button.grid(row=0, column=0, padx=5, pady=5)

        self.reset_button = ttk.Button(
            afm_ctrl_frame, text="Reset AFM", command=self.afm.reset)
        self.reset_button.grid(row=0, column=1, padx=5, pady=5)

    def create_plot_control_panel(self):
        # Panel for real-time plotting controls
        plot_ctrl_frame = ttk.LabelFrame(self.root, text="Plot Controls")
        plot_ctrl_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

        self.plot_button = ttk.Button(
            plot_ctrl_frame, text="Plot Realtime", command=self.start_realtime_plot)
        self.plot_button.grid(row=0, column=0, padx=5, pady=5)

        self.pause_button = ttk.Button(
            plot_ctrl_frame, text="Pause Realtime", command=self.pause_plot)
        self.pause_button.grid(row=0, column=1, padx=5, pady=5)

    def create_scan_control_panel(self):
        # Panel for scan controls
        scan_frame = ttk.LabelFrame(self.root, text="Scan Control")
        scan_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(scan_frame, text="Start:").grid(
            row=0, column=0, padx=5, pady=5, sticky="w")
        self.start_entry = ttk.Entry(scan_frame, width=10)
        self.start_entry.insert(0, "10000")
        self.start_entry.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(scan_frame, text="End:").grid(
            row=1, column=0, padx=5, pady=5, sticky="w")
        self.end_entry = ttk.Entry(scan_frame, width=10)
        self.end_entry.insert(0, "50000")
        self.end_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(scan_frame, text="Steps:").grid(
            row=2, column=0, padx=5, pady=5, sticky="w")
        self.steps_entry = ttk.Entry(scan_frame, width=10)
        self.steps_entry.insert(0, "100")
        self.steps_entry.grid(row=2, column=1, padx=5, pady=5, sticky="w")

        self.start_scan_button = ttk.Button(
            scan_frame, text="Start Scan", command=self.start_scan)
        self.start_scan_button.grid(row=3, column=0, padx=5, pady=5)

        self.stop_scan_button = ttk.Button(
            scan_frame, text="Stop Scan", command=self.stop_scan)
        self.stop_scan_button.grid(row=3, column=1, padx=5, pady=5)

    def create_dac_control_panel(self):
        # Panel for DAC control (showing current values and arrow buttons for adjustments)
        dac_frame = ttk.LabelFrame(self.root, text="DAC Control")
        dac_frame.grid(row=4, column=0, padx=10, pady=5, sticky="ew")

        # Current DAC values for channels F and T
        ttk.Label(dac_frame, text="F Channel:").grid(
            row=0, column=0, padx=5, pady=5, sticky="w")
        self.f_dac_value = tk.StringVar(
            value=str(self.afm.afm_status.dac_values["F"]))
        ttk.Label(dac_frame, textvariable=self.f_dac_value, width=10).grid(
            row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(dac_frame, text="T Channel:").grid(
            row=1, column=0, padx=5, pady=5, sticky="w")
        self.t_dac_value = tk.StringVar(
            value=str(self.afm.afm_status.dac_values["T"]))
        ttk.Label(dac_frame, textvariable=self.t_dac_value, width=10).grid(
            row=1, column=1, padx=5, pady=5, sticky="w")

        # Step Size entry for DAC adjustments
        ttk.Label(dac_frame, text="Step Size:").grid(
            row=2, column=0, padx=5, pady=5, sticky="w")
        self.step_entry = ttk.Entry(dac_frame, width=10)
        self.step_entry.insert(0, "100")
        self.step_entry.grid(row=2, column=1, padx=5, pady=5, sticky="w")

        # Arrow button pad for DAC adjustment
        arrow_frame = ttk.Frame(dac_frame)
        arrow_frame.grid(row=3, column=0, columnspan=2, pady=10)

        # Up arrow (increases F channel)
        self.up_button = ttk.Button(arrow_frame, text="↑",
                                    command=lambda: self.adjust_dac("F", +self.get_step()))
        self.up_button.grid(row=0, column=1, padx=5, pady=5)

        # Left arrow (decreases T channel)
        self.left_button = ttk.Button(arrow_frame, text="←",
                                      command=lambda: self.adjust_dac("T", -self.get_step()))
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        # Right arrow (increases T channel)
        self.right_button = ttk.Button(arrow_frame, text="→",
                                       command=lambda: self.adjust_dac("T", +self.get_step()))
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        # Down arrow (decreases F channel)
        self.down_button = ttk.Button(arrow_frame, text="↓",
                                      command=lambda: self.adjust_dac("F", -self.get_step()))
        self.down_button.grid(row=2, column=1, padx=5, pady=5)

    def get_step(self):
        try:
            return int(self.step_entry.get())
        except ValueError:
            return 100

    def adjust_dac(self, channel, delta):
        current_val = self.afm.afm_status.dac_values[channel]
        new_val = current_val + delta
        new_val = max(0, min(65535, new_val))  # Enforce 16-bit range
        self.afm.set_dac(channel, new_val)
        # Update the GUI display for the channel
        if channel == "F":
            self.f_dac_value.set(str(new_val))
        elif channel == "T":
            self.t_dac_value.set(str(new_val))

    def pause_plot(self):
        if hasattr(self, 'realtime_plot'):
            self.realtime_plot.paused = not self.realtime_plot.paused

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_combobox['values'] = [port.device for port in ports]

    def connect_afm(self):
        port = self.port_combobox.get()
        baud_rate = int(self.baud_entry.get())
        self.afm.connect(port, baud_rate)

    def start_scan(self):
        start = int(self.start_entry.get())
        end = int(self.end_entry.get())
        steps = int(self.steps_entry.get())
        self.scan_plot = ScanPlot(self.afm)
        self.afm.start_scan(start, end, steps)
        self.scan_plot.start_real_time_plot()

    def stop_scan(self):
        self.afm.stop_scan()

    def start_realtime_plot(self):
        self.realtime_plot = RealtimePlot(self.afm)
        self.realtime_plot.start_plot()


if __name__ == "__main__":
    root = Tk()
    app = AFM_GUI(root)
    root.mainloop()
