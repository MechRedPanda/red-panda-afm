import sys
import time
from PyQt6.QtCore import QEvent, QTimer, Qt
from PyQt6.QtGui import QIntValidator, QDoubleValidator
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox, QCheckBox, QWidget, QVBoxLayout
import pyqtgraph as pg
from afm import AFM, AFMState
from ui.focus_widget import Ui_FocusWidget
from ui.main_window import Ui_MainWindow
from ui.approach_widget import Ui_ApproachWidget
from ui.scan_widget import Ui_ScanWidget
import numpy as np
from pyqtgraph import ImageView
from datetime import datetime
import traceback
import os


class FocusWidget(QMainWindow, Ui_FocusWidget):
    def __init__(self, afm: AFM):
        super().__init__()
        self.setupUi(self)
        self.afm = afm
        self.f_start.setText("37000")
        self.f_end.setText("50000")
        self.step_size.setText("2")
        self.has_shown_results = False  # Flag to track if we've shown results

        self.start_focus_button.clicked.connect(self.start_focus)
        self.stop_focus_button.clicked.connect(self.stop_focus)
        self.set_auto_focus.clicked.connect(self.set_to_optimal_focus)

        self.focus_plot_widget.setTitle("ADC vs DAC_F", color="r", size="12pt")
        self.focus_plot_widget.setLabel(
            'left', 'adc_0 Value', color='black', size=14)
        self.focus_plot_widget.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.focus_plot_widget.showGrid(x=True, y=True)
        self.focus_plot_widget.setBackground('white')
        self.focus_plot_widget.addLegend()

        self.adc_0_plot = self.focus_plot_widget.plot(
            pen=pg.mkPen('b', width=5), name="ADC 0")

        # Add vertical line for optimal focus value
        self.optimal_line = self.focus_plot_widget.addLine(x=None, angle=90, pen=pg.mkPen(
            'g', width=2, style=Qt.PenStyle.DashLine), name="Optimal Focus")
        self.optimal_line.setVisible(False)  # Initially hidden

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def start_focus(self):
        self.focusing_button.setChecked(True)
        self.has_shown_results = False  # Reset the flag when starting new focus
        self.afm.focus(
            start=int(self.f_start.text()),
            end=int(self.f_end.text()),
            step_size=int(self.step_size.text()),
        )

    def stop_focus(self):
        self.afm.stop_focus()
        self.focusing_button.setChecked(False)

    def set_to_optimal_focus(self):
        """Ramp DAC to the optimal focus value."""
        optimal_value = self.afm.get_optimal_focus()
        if optimal_value is not None:
            if self.afm.set_dac("F", optimal_value):
                QMessageBox.information(
                    self, "Success", f"Moving to optimal focus point: {optimal_value}")
            else:
                QMessageBox.warning(
                    self, "Error", "Failed to move to optimal focus point")
        else:
            QMessageBox.warning(
                self, "Error", "No optimal focus point available")

    def update_plot(self):
        if self.afm.focus_results:
            dac_f_values = [result[0] for result in self.afm.focus_results]
            adc_0_values = [result[1] for result in self.afm.focus_results]

            self.adc_0_plot.setData(dac_f_values, adc_0_values)

            # Check if focus has stopped or finished and we haven't shown results yet
            if not self.afm.is_focus_running() and not self.has_shown_results:
                self.focusing_button.setChecked(False)
                self.has_shown_results = True  # Mark that we've shown the results

                # Show auto-focus results and update label
                optimal_value = self.afm.get_optimal_focus()
                if optimal_value is not None:
                    self.auto_focus_value.setText(str(optimal_value))
                    # Update optimal line position and make it visible
                    self.optimal_line.setValue(optimal_value)
                    self.optimal_line.setVisible(True)
                    QMessageBox.information(
                        self,
                        "Auto-Focus Results",
                        f"Optimal focus point found at DAC value: {optimal_value}\n\n"
                        f"You can move to this position using the DAC controls."
                    )
                else:
                    self.auto_focus_value.setText("None")
                    # Hide the line if no optimal value
                    self.optimal_line.setVisible(False)
                    QMessageBox.warning(
                        self,
                        "Auto-Focus Results",
                        "No optimal focus point could be determined."
                    )

    def closeEvent(self, event):
        """Ensure timer is stopped when the window closes."""
        print("Stopping FocusWidget timer...")
        self.timer.stop()
        # Optionally stop AFM focus operation if it's running
        if self.afm.is_focus_running():
             self.afm.stop_focus()
        super().closeEvent(event) # Accept the close event


# Assuming you have a corresponding UI file
class ApproachWidget(QMainWindow, Ui_ApproachWidget):
    def __init__(self, afm: AFM):
        super().__init__()
        self.setupUi(self)
        self.afm = afm

        # Set default values
        self.motor_number.setValue(1)  # Default to motor 1
        self.step_size.setText("100")
        self.max_steps.setText("500000")  # Positive for forward direction
        self.threshold.setText("100")
        self.step_delay_input.setText("100")  # Set default step delay (speed)

        # Set up input validation for step delay
        step_delay_validator = QIntValidator()
        step_delay_validator.setBottom(1) # Ensure delay is at least 1us
        self.step_delay_input.setValidator(step_delay_validator)

        # Connect buttons
        self.start_approach_button.clicked.connect(self.start_approach)
        self.stop_approach_button.clicked.connect(self.stop_approach)

        # Setup plot
        self.setup_plot()

        # Timer for updating the display
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)  # Update every 100ms

        # Data storage for plotting
        self.approach_data = []  # Store all approach data points
        self.initial_adc = None  # Store initial ADC value for threshold lines

    def setup_plot(self):
        """Initialize the plot widget."""
        self.approach_plot_widget.setTitle(
            "ADC Value During Approach", color="r", size="12pt")
        self.approach_plot_widget.setLabel(
            'left', 'ADC Value', color='black', size=14)
        self.approach_plot_widget.setLabel(
            'bottom', 'Steps Moved', color='black', size=14)
        self.approach_plot_widget.showGrid(x=True, y=True)
        self.approach_plot_widget.setBackground('white')
        self.approach_plot_widget.addLegend()

        self.adc_plot = self.approach_plot_widget.plot(
            pen=pg.mkPen('b', width=2), name="ADC Value")
        self.threshold_line_upper = self.approach_plot_widget.plot(
            pen=pg.mkPen('r', width=1, style=Qt.PenStyle.DashLine), name="Upper Threshold")
        self.threshold_line_lower = self.approach_plot_widget.plot(
            pen=pg.mkPen('r', width=1, style=Qt.PenStyle.DashLine), name="Lower Threshold")

    def start_approach(self):
        """Start the approach procedure."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to AFM first")
            return

        if self.afm.is_approach_running():
            QMessageBox.warning(self, "Already Running", "Approach is already running")
            return

        try:
            # Get parameters from UI
            motor = self.motor_number.value()
            step_size = int(self.step_size.text())
            max_steps = int(self.max_steps.text())
            threshold = int(self.threshold.text())
            speed = int(self.step_delay_input.text()) # Read speed from step_delay_input

            # Validate step_size (must be positive)
            if step_size <= 0:
                QMessageBox.warning(self, "Invalid Input", "Step size must be positive")
                return

            # Validate speed (must be positive)
            if speed <= 0:
                QMessageBox.warning(self, "Invalid Input", "Step Speed (uS) must be positive")
                return

            # Clear previous data
            self.approach_data = []
            self.initial_adc = None

            # Start approach - Now passing speed directly
            self.approaching_button.setChecked(True)
            success = self.afm.approach(
                motor=motor,
                step_size=step_size,
                threshold=threshold,
                max_steps=max_steps,
                speed=speed # Pass speed parameter
            )

            if not success:
                self.approaching_button.setChecked(False)
                QMessageBox.warning(self, "Error", "Failed to start approach")

        except ValueError as e:
            self.approaching_button.setChecked(False)
            QMessageBox.warning(self, "Invalid Input", str(e))

    def stop_approach(self):
        """Stop the approach procedure."""
        if not self.afm.is_connected():
            return

        if not self.afm.is_approach_running():
            return

        success = self.afm.stop_approach()
        if success:
            self.approaching_button.setChecked(False)
        else:
            QMessageBox.warning(self, "Error", "Failed to stop approach")

    def update_display(self):
        """Update the display with current approach status."""
        if not self.afm.is_connected():
            return

        # Get approach data from AFM
        if self.afm.is_approach_running():
            new_data = self.afm.get_approach_data()
            if new_data:
                # Append new data to our history
                self.approach_data.extend(new_data)

                # Store initial ADC value if we don't have it yet
                if self.initial_adc is None and len(new_data) > 0:
                    self.initial_adc = new_data[0]['adc']

                # Extract steps and ADC values from all data
                steps_data = [point['steps'] for point in self.approach_data]
                adc_data = [point['adc'] for point in self.approach_data]

                if len(steps_data) > 1:
                    # Update plot with all data
                    self.adc_plot.setData(steps_data, adc_data)

                    # Update threshold lines if we have initial ADC value
                    if self.initial_adc is not None:
                        threshold = int(self.threshold.text())
                        upper_thresh = self.initial_adc + threshold
                        lower_thresh = self.initial_adc - threshold

                        # Set threshold lines to span the same range as the data
                        self.threshold_line_upper.setData([steps_data[0], steps_data[-1]],
                                                        [upper_thresh, upper_thresh])
                        self.threshold_line_lower.setData([steps_data[0], steps_data[-1]],
                                                        [lower_thresh, lower_thresh])

        # Update button state based on approach status
        self.approaching_button.setChecked(self.afm.is_approach_running())

    def closeEvent(self, event):
        """Ensure timer is stopped and approach is halted when the window closes."""
        print("Stopping ApproachWidget timer...")
        self.timer.stop()
        # Optionally stop AFM approach operation if it's running
        if self.afm.is_approach_running():
            print("Stopping active approach...")
            self.afm.stop_approach()
        super().closeEvent(event) # Accept the close event


class ScanWidget(QMainWindow, Ui_ScanWidget):
    def __init__(self, afm: AFM):
        super().__init__()
        self.setupUi(self)
        self.afm = afm

        # --- UI Setup ---
        # Scan parameters defaults
        self.x_start_input.setText("10000")
        self.x_end_input.setText("60000")
        self.y_start_input.setText("10000")
        self.y_end_input.setText("60000")
        self.scan_resolution_input.setText("128")
        self.y_microstep_input.setText("64")  # Default to 1 microstep
        self.step_delay_us_input.setText("20")  # Default to 0 delay

        # --- Initialize Plots and Images ---
        # Titles and labels for the plots
        self.adc_0_plot_curve.setTitle("ADC0 Last Line", color="b", size="10pt")
        self.adc_0_plot_curve.setLabel('left', 'ADC0 Value')
        self.adc_0_plot_curve.setLabel('bottom', 'Pixel Index')
        self.adc_0_plot_curve.showGrid(x=True, y=True)
        self.adc_0_line_plot = self.adc_0_plot_curve.plot(pen=pg.mkPen('b', width=2), name="Trace")
        self.adc_0_retrace_plot = self.adc_0_plot_curve.plot(pen=pg.mkPen('r', width=2, style=Qt.PenStyle.DashLine), name="Retrace")

        self.dac_z_plot_curve.setTitle("DAC Z Last Line", color="g", size="10pt")
        self.dac_z_plot_curve.setLabel('left', 'DAC Z Value')
        self.dac_z_plot_curve.setLabel('bottom', 'Pixel Index')
        self.dac_z_plot_curve.showGrid(x=True, y=True)
        self.dac_z_line_plot = self.dac_z_plot_curve.plot(pen=pg.mkPen('g', width=2), name="Trace")
        self.dac_z_retrace_plot = self.dac_z_plot_curve.plot(pen=pg.mkPen('r', width=2, style=Qt.PenStyle.DashLine), name="Retrace")

        # Customize ImageView appearance
        self.adc_0_scan_image.ui.histogram.hide()
        self.adc_0_scan_image.ui.roiBtn.hide()
        self.adc_0_scan_image.ui.menuBtn.hide()
        self.adc_0_scan_image.getView().setAspectLocked(True)
        self.adc_0_scan_image.getView().invertY(False)

        self.dac_z_scan_image.ui.histogram.hide()
        self.dac_z_scan_image.ui.roiBtn.hide()
        self.dac_z_scan_image.ui.menuBtn.hide()
        self.dac_z_scan_image.getView().setAspectLocked(True)
        self.dac_z_scan_image.getView().invertY(False)

        # Connect buttons
        self.start_scan_button.clicked.connect(self.start_scan)
        self.stop_scan_button.clicked.connect(self.stop_scan_wrapper)
        self.save_data_button.clicked.connect(self.save_scan_data)

        # Timer for updating the display
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)  # Update every 100ms

        # Internal state
        self.last_scan_data_count = 0
        self.total_scan_points = 0
        self.current_resolution = 0

        # Create data directory if it doesn't exist
        self.data_dir = "data"
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        # Initialize scan data tracking
        self._last_trace_count = 0
        self._last_retrace_count = 0

    def start_scan(self):
        """Start the scan procedure."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to AFM first")
            return

        if self.afm.is_scan_running():
            QMessageBox.warning(self, "Already Running", "Scan is already running")
            return

        try:
            # Get parameters from UI
            x_start = int(self.x_start_input.text())
            x_end = int(self.x_end_input.text())
            y_start = int(self.y_start_input.text())
            y_end = int(self.y_end_input.text())
            resolution = int(self.scan_resolution_input.text())
            y_micro_steps = int(self.y_microstep_input.text())
            y_micro_step_wait_us = int(self.step_delay_us_input.text())

            # Validation
            if not all(0 <= v <= 65535 for v in [x_start, x_end, y_start, y_end]):
                QMessageBox.warning(self, "Invalid Input", "DAC coordinates must be between 0 and 65535")
                return
            if x_start >= x_end or y_start >= y_end:
                QMessageBox.warning(self, "Invalid Input", "Start coordinates must be less than End coordinates")
                return
            if resolution < 2:
                QMessageBox.warning(self, "Invalid Input", "Resolution must be at least 2")
                return
            if y_micro_steps < 1:
                QMessageBox.warning(self, "Invalid Input", "Y micro-steps must be at least 1")
                return
            if y_micro_step_wait_us < 0:
                QMessageBox.warning(self, "Invalid Input", "Y micro-step wait time cannot be negative")
                return

            # Prepare for new scan
            self.last_scan_data_count = 0
            self.total_scan_points = resolution * resolution
            self.current_resolution = resolution
            self.adc_0_scan_image.clear()
            self.dac_z_scan_image.clear()
            self.adc_0_line_plot.clear()
            self.dac_z_line_plot.clear()
            self.scanning_button.setChecked(True)
            self.timer.start(100)

            # Start scan in AFM class
            success = self.afm.start_scan(
                x_start=x_start,
                x_end=x_end,
                y_start=y_start,
                y_end=y_end,
                resolution=resolution,
                y_micro_steps=y_micro_steps,
                y_micro_step_wait_us=y_micro_step_wait_us
            )

            if not success:
                self.scanning_button.setChecked(False)
                self.total_scan_points = 0
                self.current_resolution = 0
                self.timer.stop()
                QMessageBox.warning(self, "Error", "Failed to start scan (check AFM state or connection)")

        except ValueError:
            self.scanning_button.setChecked(False)
            self.timer.stop()
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers for scan parameters.")

    def stop_scan_wrapper(self):
        """Wrapper to stop the scan procedure."""
        self.timer.stop()
        print("GUI: Stopping update timer.")

        if not self.afm.is_connected():
            self.scanning_button.setChecked(False)
            return

        if not self.afm.is_scan_running():
            self.scanning_button.setChecked(False)
            return

        print("GUI: Attempting to stop scan...")
        success = self.afm.stop_scan()

        self.scanning_button.setChecked(False)
        if not success:
            QMessageBox.warning(self, "Stop Scan", "Sent stop command, but confirmation failed or hardware issue suspected.")
        else:
            print("GUI: Scan stop confirmed or initiated.")

    def update_display(self):
        """Update the display with the latest scan data."""
        try:
            # Get the current scan data counts
            trace_count = self.afm.get_scan_data_count("trace")
            retrace_count = self.afm.get_scan_data_count("retrace")
            
            # Only update if we have new data
            if trace_count == self._last_trace_count and retrace_count == self._last_retrace_count:
                return
                
            self._last_trace_count = trace_count
            self._last_retrace_count = retrace_count
            
            # Get images for both channels
            adc0_image = self.afm.get_scan_image(channel_index=0, scan_type="trace")
            dacz_image = self.afm.get_scan_image(channel_index=1, scan_type="trace")
            
            if adc0_image is not None:
                # Calculate min and max values, handling NaN values
                valid_values = adc0_image[~np.isnan(adc0_image)]
                if len(valid_values) > 0:
                    min_val = np.min(valid_values)
                    max_val = np.max(valid_values)
                    if min_val == max_val:
                        min_val -= 1
                        max_val += 1
                else:
                    min_val = 0
                    max_val = 1
                self.adc_0_scan_image.setImage(adc0_image)
                self.adc_0_scan_image.setLevels(min_val, max_val)
                
            if dacz_image is not None:
                valid_values = dacz_image[~np.isnan(dacz_image)]
                if len(valid_values) > 0:
                    min_val = np.min(valid_values)
                    max_val = np.max(valid_values)
                    if min_val == max_val:
                        min_val -= 1
                        max_val += 1
                else:
                    min_val = 0
                    max_val = 1
                self.dac_z_scan_image.setImage(dacz_image)
                self.dac_z_scan_image.setLevels(min_val, max_val)
                
            # Clear all line plots before updating
            self.adc_0_line_plot.clear()
            self.dac_z_line_plot.clear()
            self.adc_0_retrace_plot.clear()
            self.dac_z_retrace_plot.clear()
                
            # Get the last complete line for both trace and retrace
            last_trace_line = self.afm.get_scan_line(scan_type="trace")
            last_retrace_line = self.afm.get_scan_line(scan_type="retrace")
            
            if last_trace_line is not None:
                self.adc_0_line_plot.setData(last_trace_line[:, 1], last_trace_line[:, 2], pen='b')
                self.dac_z_line_plot.setData(last_trace_line[:, 1], last_trace_line[:, 3], pen='g')
                
            if last_retrace_line is not None:
                self.adc_0_retrace_plot.setData(last_retrace_line[:, 1], last_retrace_line[:, 2], pen='r', style='--')
                self.dac_z_retrace_plot.setData(last_retrace_line[:, 1], last_retrace_line[:, 3], pen='r', style='--')
                
        except Exception as e:
            print(f"Error updating display: {e}")
            traceback.print_exc()

    def save_scan_data(self):
        """Save current scan data to TIFF files."""
        if not self.afm.scan_data:
            QMessageBox.warning(self, "No Data", "No scan data available to save")
            return

        try:
            # Get filename from input or generate default
            filename = self.save_data_filename.text().strip()
            if not filename:
                # Generate default folder name
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                resolution = self.afm.scan_resolution
                folder_name = f"{self.data_dir}/{timestamp}_{resolution}x{resolution}"
            else:
                # Use user-provided filename as base
                folder_name = os.path.join(self.data_dir, os.path.splitext(filename)[0])

            # Create the folder
            os.makedirs(folder_name, exist_ok=True)

            # Export all channels (both ADC0 and DACZ)
            try:
                success = self.afm.export_tiff(folder_name, channels=[0, 1])
                if success:
                    QMessageBox.information(self, "Success", "Scan data saved successfully")
                else:
                    QMessageBox.warning(self, "Error", "Failed to save scan data")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Error saving scan data: {str(e)}")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error preparing to save scan data: {str(e)}")

    def closeEvent(self, event):
        """Ensure timer is stopped and scan is stopped when window closes."""
        print("Stopping ScanWidget timer...")
        self.timer.stop()
        if self.afm.is_scan_running():
             print("Scan is running, attempting to stop it...")
             self.afm.stop_scan()
        super().closeEvent(event)


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.afm = AFM()

        # DAC control state
        self.dac_update_lock = False
        self.focused_dac_control = None
        self.last_dac_update_time = 0

        # References to child widgets
        self.focus_widget = None
        self.approach_widget = None
        self.scan_widget = None

        self.init_ui()

    def init_ui(self):
        self.connected_button.setEnabled(False)
        self.connected_button.setChecked(False)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.handle_connect)
        self.reset_button.clicked.connect(self.afm_reset)
        self.close_button.clicked.connect(self.afm_close)
        self.restore_button.clicked.connect(self.restore_afm)

        # Setup PID control
        self.setup_pid_control()

        self.setup_plots()
        self.setup_dac_controls()
        self.setup_motor_ui()

        self.focus_widget_button.clicked.connect(self.open_focus_widget)
        self.approach_widget_button.clicked.connect(self.open_approach_widget)
        self.scan_widget_button.clicked.connect(self.open_scan_widget)

        # Connect target ADC value input
        self.target_adc_value_input.editingFinished.connect(self.on_target_adc_changed)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_all)
        self.update_timer.start(100)

        # Connect ADC Average controls
        self.avg_points_refresh.clicked.connect(self.set_adc_average_window)
        self.avg_points_input.editingFinished.connect(self.set_adc_average_window) # Also apply on edit finish

        # Connect Plot history
        self.plot_history_input.editingFinished.connect(self.update_plot_history)
        self._plot_time_window_s = float(self.plot_history_input.text()) # Initial value

        # Setup XY Range combo box
        self.xy_range_input_box.clear()
        self.xy_range_input_box.addItems(['10V', '3V'])

        # Connect XY Range control
        self.xy_range_set_button.clicked.connect(self.set_dac_range)
        self.xy_range_input_box.currentIndexChanged.connect(self.set_dac_range)

    def setup_pid_control(self):
        """Initialize PID control elements and connect signals"""
        # Set up input validation for PID parameters
        pid_validator = QDoubleValidator()
        pid_validator.setBottom(0.0)
        pid_validator.setTop(1000.0)
        pid_validator.setDecimals(4)

        # Apply validators to input boxes
        self.p_input_box.setValidator(pid_validator)
        self.i_input_box.setValidator(pid_validator)
        self.d_input_box.setValidator(pid_validator)

        # Set default values
        self.p_input_box.setText("10.0")
        self.i_input_box.setText("1.0")
        self.d_input_box.setText("0.0")

        # Connect signals
        self.pid_enable_button.clicked.connect(self.enable_pid_control)
        self.pid_disable_button.clicked.connect(self.disable_pid_control)
        self.pid_reverse_button.stateChanged.connect(self.apply_pid_parameters)
        self.pid_refresh_button.clicked.connect(self.refresh_pid_parameters)
        self.target_adc_value_input.editingFinished.connect(self.on_target_adc_changed)
        self.p_input_box.editingFinished.connect(self.apply_pid_parameters)
        self.i_input_box.editingFinished.connect(self.apply_pid_parameters)
        self.d_input_box.editingFinished.connect(self.apply_pid_parameters)

        # Add status update timer
        self.pid_status_timer = QTimer(self)
        self.pid_status_timer.timeout.connect(self.update_pid_status)
        self.pid_status_timer.start(100)  # Update every 100ms

    def enable_pid_control(self):
        """Handle the PID Enable button click."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to AFM first")
            return

        try:
            # Get target ADC value, use current value if empty
            target = None
            if self.target_adc_value_input.text():
                try:
                    target = int(self.target_adc_value_input.text())
                except ValueError:
                    QMessageBox.warning(self, "Invalid Input", "Target ADC must be a valid integer.")
                    return
            else:
                # Fetch current ADC if target is empty
                status = self.afm.get_status()
                if status:
                    target = status.adc_0
                    self.target_adc_value_input.setText(str(target))
                else:
                    QMessageBox.warning(self, "PID Error", "Could not get current ADC value for target.")
                    return

            # Ensure parameters are sent/refreshed before enabling
            self.apply_pid_parameters()

            # Now attempt to enable
            enable_response = self.afm.enable_pid(target=target)
            if enable_response and enable_response.get("status") == "success":
                self.pid_enabled_radio.setChecked(True)
                print("PID Enabled.")
            else:
                self.pid_enabled_radio.setChecked(False)
                QMessageBox.warning(self, "PID Error", f"Failed to enable PID: {enable_response.get('message', 'Unknown error')}")
                self.refresh_pid_parameters_from_hw() # Sync GUI

        except Exception as e:
            QMessageBox.warning(self, "PID Error", f"Failed to enable PID: {str(e)}")
            self.refresh_pid_parameters_from_hw() # Sync GUI on error

    def disable_pid_control(self):
        """Handle the PID Disable button click."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to AFM first")
            return

        try:
            success = self.afm.disable_pid()
            if success:
                self.pid_enabled_radio.setChecked(False)
                print("PID Disabled.")
            else:
                self.pid_enabled_radio.setChecked(True) # Revert UI if disable failed
                QMessageBox.warning(self, "PID Error", "Failed to disable PID.")
                self.refresh_pid_parameters_from_hw() # Sync GUI
        except Exception as e:
            QMessageBox.warning(self, "PID Error", f"Failed to disable PID: {str(e)}")
            self.refresh_pid_parameters_from_hw() # Sync GUI on error

    def apply_pid_parameters(self):
        """Apply current PID parameters to the AFM"""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected",
                                "Please connect to AFM first")
            return

        try:
            # Get values from input boxes
            kp = float(self.p_input_box.text())
            ki = float(self.i_input_box.text())
            kd = float(self.d_input_box.text())
            pid_reverse = self.pid_reverse_button.isChecked()
            
            # Get target ADC value, use current value if empty
            if self.target_adc_value_input.text():
                target = int(self.target_adc_value_input.text())
            else:
                target = self.afm.status_queue[-1].adc_0
                self.target_adc_value_input.setText(str(target))

            # Apply parameters
            print(f"Applying PID parameters: kp={kp}, ki={ki}, kd={kd}, invert={pid_reverse}, target={target}")
            success = self.afm.set_pid_parameters(
                kp=kp, ki=ki, kd=kd, invert=pid_reverse, target=target)

            if not success:
                QMessageBox.warning(self, "PID Error", "Failed to apply PID parameters")
                # Refresh from hardware to ensure GUI matches reality
                self.refresh_pid_parameters_from_hw()
            else:
                 # If PID is enabled, re-enable it with new parameters to ensure they take effect immediately
                 if self.pid_enabled_radio.isChecked():
                    enable_success = self.afm.enable_pid(target=target)
                    if not enable_success:
                         QMessageBox.warning(self, "PID Error", "Failed to re-enable PID after parameter change.")
                         self.refresh_pid_parameters_from_hw() # Sync GUI

        except ValueError:
            QMessageBox.warning(self, "Invalid Input",
                                "Please enter valid numbers for PID parameters")
        except Exception as e:
            QMessageBox.warning(self, "PID Error",
                                f"Failed to apply PID parameters: {str(e)}")

    def update_pid_status(self):
        """Update PID status display"""
        if not self.afm.is_connected() or self.afm.is_busy:
            return

        try:
            status = self.afm.get_pid_status()
            if status:
                # Update checkbox state based on PID state
                self.pid_enabled_radio.setChecked(status.get('enabled', False))
                self.pid_reverse_button.setChecked(status.get('invert', False))

                # Don't update the target_adc_value_input or P, I, D parameters from status
                # These should only be set by user input

        except Exception as e:
            print(f"Error updating PID status: {str(e)}")

    def refresh_pid_parameters(self):
        """Send the current PID parameter values from the GUI to the AFM, without enabling/disabling."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to AFM first")
            return

        try:
            kp = float(self.p_input_box.text())
            ki = float(self.i_input_box.text())
            kd = float(self.d_input_box.text())
            invert = self.pid_reverse_button.isChecked()
            target = None
            if self.target_adc_value_input.text():
                try:
                    target = int(self.target_adc_value_input.text())
                except ValueError:
                    QMessageBox.warning(self, "Invalid Input", "Target ADC must be a valid integer for refresh.")
                    return

            print(f"Refreshing PID parameters: kp={kp}, ki={ki}, kd={kd}, invert={invert}, target={target}")
            success = self.afm.set_pid_parameters(kp=kp, ki=ki, kd=kd, invert=invert, target=target)

            if success:
                print("PID parameters sent successfully.")
                self.statusbar.showMessage("PID parameters refreshed.", 2000)
            else:
                QMessageBox.warning(self, "PID Error", "Failed to send PID parameters to AFM.")

        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers for PID parameters.")
        except Exception as e:
            QMessageBox.warning(self, "PID Error", f"Failed to refresh PID parameters: {str(e)}")

    def refresh_pid_parameters_from_hw(self):
        """Fetch PID parameters from hardware and update the GUI."""
        if not self.afm.is_connected():
            return
        try:
            status = self.afm.get_pid_status()
            if status:
                # Update GUI elements, blocking signals temporarily
                self.p_input_box.blockSignals(True)
                self.i_input_box.blockSignals(True)
                self.d_input_box.blockSignals(True)
                self.pid_reverse_button.blockSignals(True)
                self.target_adc_value_input.blockSignals(True)

                self.p_input_box.setText(f"{status.get('kp', 0.0):.4f}")
                self.i_input_box.setText(f"{status.get('ki', 0.0):.4f}")
                self.d_input_box.setText(f"{status.get('kd', 0.0):.4f}")
                self.pid_reverse_button.setChecked(status.get('invert', False))
                self.target_adc_value_input.setText(str(status.get('target', 0)))
                self.pid_enabled_radio.setChecked(status.get('enabled', False))

                self.p_input_box.blockSignals(False)
                self.i_input_box.blockSignals(False)
                self.d_input_box.blockSignals(False)
                self.pid_reverse_button.blockSignals(False)
                self.target_adc_value_input.blockSignals(False)
                print("PID GUI refreshed from hardware.")
            else:
                QMessageBox.warning(self, "PID Error", "Could not get PID status from hardware to refresh GUI.")
        except Exception as e:
            QMessageBox.warning(self, "PID Error", f"Error refreshing PID GUI from hardware: {e}")

    def setup_plots(self):
        self.adc_plot_widget.setTitle(
            "Real-time AFM Data", color="r", size="12pt")
        self.adc_plot_widget.setLabel(
            'left', 'ADC Value', color='black', size=14)
        self.adc_plot_widget.setLabel(
            'bottom', 'Time (s)', color='black', size=14)
        self.adc_plot_widget.showGrid(x=True, y=True)
        self.adc_plot_widget.setBackground('white')
        self.adc_plot_widget.addLegend()
        self.adc_0_plot_curve = self.adc_plot_widget.plot(
            pen=pg.mkPen('b', width=2), name="ADC 0 Avg")
        self.adc_1_plot_curve = self.adc_plot_widget.plot(
            pen=pg.mkPen('r', width=2), name="ADC 1 Avg")

        self.dac_plot_widget.setTitle("DAC Values", color="r", size="12pt")
        self.dac_plot_widget.setLabel(
            'left', 'DAC Value', color='black', size=14)
        self.dac_plot_widget.setLabel(
            'bottom', 'Time (s)', color='black', size=14)
        self.dac_plot_widget.showGrid(x=True, y=True)
        self.dac_plot_widget.setBackground('white')
        self.dac_plot_widget.addLegend()
        self.dac_f_plot = self.dac_plot_widget.plot(
            pen=pg.mkPen('b', width=2), name="DAC F")
        self.dac_t_plot = self.dac_plot_widget.plot(
            pen=pg.mkPen('g', width=2), name="DAC T")

    def refresh_ports(self):
        current_port = self.port_list_box.currentText()

        self.port_list_box.clear()
        ports = self.afm.get_serial_ports()

        if not ports:
            self.port_list_box.addItem("No ports available")
            self.port_list_box.setDisabled(True)
        else:
            for port in ports:
                self.port_list_box.addItem(
                    f"{port.device} - {port.description}")
            self.port_list_box.setDisabled(False)

            # Try to restore previous selection
            if current_port in [self.port_list_box.itemText(i) for i in range(self.port_list_box.count())]:
                self.port_list_box.setCurrentText(current_port)
            elif ports:  # Select first port if none was selected
                self.port_list_box.setCurrentIndex(0)

    def handle_connect(self):
        """Handle connect button click by calling afm.connect()"""
        if self.port_list_box.currentIndex() == -1:
            QMessageBox.warning(self, "No Port Selected",
                                "Please select a port from the list.")
            return
        port = self.afm.serial_ports[self.port_list_box.currentIndex()]

        try:
            # Call the existing AFM connect method
            success = self.afm.connect(port=port.device)

            if success:
                # Update UI on successful connection
                self.connected_button.setChecked(True)
                print(f"Connected to {port.device} - {port.description}")
            else:
                QMessageBox.warning(self, "Connection Failed",
                                    f"Failed to connect to {port}")

        except Exception as e:
            QMessageBox.critical(self, "Connection Error",
                                 f"Error connecting to {port}:\n{str(e)}")

    def afm_close(self):
        """Closes the AFM connection."""
        try:
            self.afm.close()
            self.connected_button.setChecked(False)
            print("AFM connection closed.")
        except Exception as e:
            print(f"Failed to close AFM connection: {e}")

    def afm_reset(self):
        """Resets the AFM device."""
        try:
            self.afm.reset()
            print("AFM reset command sent.")
        except Exception as e:
            print(f"Failed to send reset command: {e}")

    def setup_dac_controls(self):
        """Initialize DAC controls with update toggle support"""
        self.max_dac_value = 2**16 - 1
        self.dac_controls = {
            'f': (self.f_slider, self.f_input_box, self.f_set_button),
            't': (self.t_slider, self.t_input_box, self.t_set_button),
            'x': (self.x_slider, self.x_input_box, self.x_set_button),
            'y': (self.y_slider, self.y_input_box, self.y_set_button),
            'z': (self.z_slider, self.z_input_box, self.z_set_button)
        }

        for key, (slider, lineedit, button) in self.dac_controls.items():
            slider.setRange(0, self.max_dac_value)
            lineedit.setValidator(QIntValidator(0, self.max_dac_value))

            # Default value
            default_val = 2**15
            slider.setValue(default_val)
            lineedit.setText(str(default_val))

            # Connect signals
            slider.valueChanged.connect(
                lambda val, k=key: self.on_dac_slider_changed(k, val))
            lineedit.editingFinished.connect(
                lambda k=key: self.on_dac_text_edit_finished(k))
            button.clicked.connect(
                lambda _, k=key: self.send_dac_value(k))

    def send_dac_value(self, key):
        """Send value to DAC with update toggle awareness"""
        if not self.afm.is_connected():
            return

        # Get current value (always from slider for consistency)
        slider, lineedit, button = self.dac_controls[key]
        value = slider.value()
        button_old_text = button.text()
        # Visual feedback
        button.setText("Sending...")
        button.setEnabled(False)
        QApplication.processEvents()

        try:
            # Send to hardware
            success = self.afm.set_dac(key.upper(), value)

            if success:
                button.setText("Sent!")
                # If updates are off, force one update to reflect new value
                if not self.dac_enable_toggle.isChecked():
                    self.manual_status_refresh(key, value)
            else:
                button.setText("Error!")
        except Exception as e:
            print(f"DAC send error: {str(e)}")
            button.setText("Error!")

        # Reset button after delay
        QTimer.singleShot(1500, lambda: (
            button.setText(button_old_text),
            button.setEnabled(True)
        ))

    def manual_status_refresh(self, key, expected_value):
        """Force update when automatic updates are disabled"""
        self.afm.get_status()
        if self.afm.status_queue:
            latest = list(self.afm.status_queue)[-1]
            hw_value = getattr(latest, f'dac_{key}')
            if hw_value != expected_value:
                print(
                    f"HW value mismatch: sent {expected_value}, got {hw_value}")
            # Update just this control
            self.update_single_dac_control(key, hw_value)

    def update_single_dac_control(self, key, value):
        """Update one DAC control without affecting others"""
        if self.dac_update_lock:
            return

        slider, lineedit, _ = self.dac_controls[key]

        slider.blockSignals(True)
        lineedit.blockSignals(True)

        slider.setValue(value)
        lineedit.setText(str(value))

        slider.blockSignals(False)
        lineedit.blockSignals(False)

    def on_dac_slider_changed(self, key, value):
        """Handle DAC slider changes with update toggle awareness"""
        if self.dac_update_lock:
            return

        # Lock to prevent recursive updates
        self.dac_update_lock = True
        _, lineedit, button = self.dac_controls[key]

        try:
            # Update lineedit to match slider
            lineedit.blockSignals(True)
            lineedit.setText(str(value))
            lineedit.blockSignals(False)

            # If updates are disabled and user moved slider,
            # visually update but don't send to hardware
            if not self.dac_enable_toggle.isChecked():
                button.setStyleSheet("background-color: #FFA500;")  # Orange
                button.setText("Click to Send")
            else:
                # If auto-updates are on, send immediately
                self.send_dac_value(key)

        except Exception as e:
            print(f"Error handling slider change: {str(e)}")
            # Revert to previous value if error occurs
            lineedit.setText(str(lineedit.text()))

        finally:
            self.last_dac_update_time = time.time()
            self.dac_update_lock = False

    def on_dac_text_edit_finished(self, key):
        """
        Handle text edit completion with update toggle awareness
        Called when user finishes editing text (presses Enter or loses focus)
        """
        if self.dac_update_lock:
            return

        slider, lineedit, button = self.dac_controls[key]
        self.dac_update_lock = True

        try:
            # Get and validate input
            text = lineedit.text()
            try:
                value = int(text)
                # Clamp to valid range
                value = max(0, min(self.max_dac_value, value))

                # Update slider to match
                slider.blockSignals(True)
                slider.setValue(value)
                slider.blockSignals(False)

                # Ensure text shows clamped value
                lineedit.blockSignals(True)
                lineedit.setText(str(value))
                lineedit.blockSignals(False)

                # Visual feedback based on update mode
                if not self.dac_enable_toggle.isChecked():
                    button.setStyleSheet(
                        "background-color: #FFA500; color: black;")
                    button.setText("Click to Send")
                else:
                    self.send_dac_value(key)

            except ValueError:
                # Revert to slider value if invalid input
                lineedit.setText(str(slider.value()))

        finally:
            self.last_dac_update_time = time.time()
            self.dac_update_lock = False

    def update_dac_controls(self, status):
        """Update all DAC controls respecting the toggle switch"""
        if not self.dac_enable_toggle.isChecked():
            return

        if (self.dac_update_lock or
                (time.time() - self.last_dac_update_time < 1.0)):
            return

        for key, (slider, lineedit, _) in self.dac_controls.items():
            if getattr(self, f'dac_{key}_locked', False):
                continue

            new_value = getattr(status, f'dac_{key}')
            if new_value != slider.value():
                slider.blockSignals(True)
                lineedit.blockSignals(True)

                slider.setValue(new_value)
                lineedit.setText(str(new_value))

                slider.blockSignals(False)
                lineedit.blockSignals(False)

    def update_all(self):
        if not self.afm.is_connected() or self.afm.busy:
            return

        self.afm.get_status()

        if not self.afm.status_queue:
            return

        latest_status = list(self.afm.status_queue)[-1]

        if self.dac_enable_toggle.isChecked():
            self.update_dac_controls(latest_status)
        self.update_plots(latest_status)
        self.update_motor_status(latest_status)

    def update_plots(self, status):
        # Use the plot history window size
        history_duration_ms = self._plot_time_window_s * 1000

        # Filter the queue based on the time window
        current_time_ms = status.timestamp # Timestamp of the *latest* status
        cutoff_time_ms = current_time_ms - history_duration_ms

        # Filter AFM status queue efficiently
        relevant_statuses = [s for s in self.afm.status_queue if s.timestamp >= cutoff_time_ms]

        if not relevant_statuses:
            # Clear plots if no data in window
            self.adc_0_plot_curve.clear()
            self.adc_1_plot_curve.clear()
            self.dac_f_plot.clear()
            self.dac_t_plot.clear()
            return

        # Calculate time relative to the *latest* timestamp (current_time_ms)
        # Timestamps will now be 0 or negative
        seconds = [(s.timestamp - current_time_ms) / 1000.0 for s in relevant_statuses]

        # Check plot visibility flags
        adc0_enabled = self.adc_0_enable_checkbox.isChecked()
        adc1_enabled = self.adc_1_enable_checkbox.isChecked()

        # Update ADC plots
        if adc0_enabled:
            # Plot averaged data instead of raw
            adc0_avg = [s.adc_0_avg for s in relevant_statuses]
            self.adc_0_plot_curve.setData(seconds, adc0_avg)
            self.adc_0_plot_curve.setVisible(True)
        else:
            self.adc_0_plot_curve.clear()
            self.adc_0_plot_curve.setVisible(False)

        if adc1_enabled:
            # Plot averaged data instead of raw
            adc1_avg = [s.adc_1_avg for s in relevant_statuses]
            self.adc_1_plot_curve.setData(seconds, adc1_avg)
            self.adc_1_plot_curve.setVisible(True)
        else:
            self.adc_1_plot_curve.clear()
            self.adc_1_plot_curve.setVisible(False)

        # Update DAC plots
        dacf_enabled = self.dac_f_enable_checkbox.isChecked()
        dact_enabled = self.dac_t_enable_checkbox.isChecked()

        if dacf_enabled:
            dacf = [s.dac_f for s in relevant_statuses]
            self.dac_f_plot.setData(seconds, dacf)
            self.dac_f_plot.setVisible(True)
        else:
            self.dac_f_plot.clear()
            self.dac_f_plot.setVisible(False)

        if dact_enabled:
            dact = [s.dac_t for s in relevant_statuses]
            self.dac_t_plot.setData(seconds, dact)
            self.dac_t_plot.setVisible(True)
        else:
            self.dac_t_plot.clear()
            self.dac_t_plot.setVisible(False)

        # Adjust x-axis range dynamically
        if seconds:
             # Range should be from -history_window to 0
             # Actual earliest time relative to t=0 (will be negative or zero)
             actual_earliest_time = seconds[0]
             # Desired earliest time based on history window
             window_earliest_time = -self._plot_time_window_s

             # Determine the minimum time to display
             # Use the later (less negative) of the actual earliest time and the window start time
             min_time = max(actual_earliest_time, window_earliest_time)
             max_time = 0 # Latest time is always 0

             self.adc_plot_widget.setXRange(min_time, max_time, padding=0.05)
             self.dac_plot_widget.setXRange(min_time, max_time, padding=0.05)

    def setup_motor_ui(self):
        """Initialize motor control UI elements for all three motors"""
        # Set up input validation for all motors
        motor_validator = QIntValidator()
        # Custom range for all motors
        motor_validator.setRange(-100000, 100000)

        # Motor 1 controls
        self.step_motor_1_input_box.setValidator(motor_validator)
        self.step_motor_1_input_box.setText("10000")
        self.step_motor_1_button.clicked.connect(
            lambda: self.start_motor_movement(1))

        # Motor 2 controls
        self.step_motor_2_input_box.setValidator(motor_validator)
        self.step_motor_2_input_box.setText("10000")
        self.step_motor_2_button.clicked.connect(
            lambda: self.start_motor_movement(2))

        # Motor 3 controls
        self.step_motor_3_input_box.setValidator(motor_validator)
        self.step_motor_3_input_box.setText("10000")
        self.step_motor_3_button.clicked.connect(
            lambda: self.start_motor_movement(3))

        # Common controls
        self.stop_motors_button.clicked.connect(self.stop_all_motors)
        self.motor_running_radio_button.setChecked(False)

        # Status indicators
        self.motor_status_labels = {
            1: self.motor_1_status_label,
            2: self.motor_2_status_label,
            3: self.motor_3_status_label
        }

        self.motor_set_fast_button.clicked.connect(self.motor_set_fast)
        self.motor_set_slow_button.clicked.connect(self.motor_set_slow)

    def motor_set_fast(self):
        self.afm.motor_delay = 100  # Set motor delay to 100us

    def motor_set_slow(self):
        self.afm.motor_delay = 500

    def start_motor_movement(self, motor_num: int):
        """Handle motor movement start for any motor"""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected",
                                "Please connect to AFM first")
            return

        try:
            # Get the correct input box for this motor
            input_box = getattr(self, f"step_motor_{motor_num}_input_box")
            steps = int(input_box.text())

            if steps == 0:
                raise ValueError("Steps cannot be zero")

            # Update UI feedback
            status_label = self.motor_status_labels[motor_num]
            status_label.setText("Starting..." if steps >
                                 0 else "Starting (CCW)...")
            status_label.setStyleSheet("color: orange;")

            # Start movement - direction determined by step sign
            response = self.afm.move_motor(
                motor=motor_num,
                steps=steps,  # Sign determines direction
            )

            if response.get("status") != "started":
                status_label.setText("Error!")
                status_label.setStyleSheet("color: red;")
                QMessageBox.warning(self, "Movement Error",
                                    f"Failed to start motor {motor_num}: {response.get('message', 'Unknown error')}")
            else:
                status_label.setText("Running" if steps >
                                     0 else "Running (CCW)")
                status_label.setStyleSheet("color: green;")

        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input", str(e))
            if motor_num in self.motor_status_labels:
                self.motor_status_labels[motor_num].setText("Invalid Input")
                self.motor_status_labels[motor_num].setStyleSheet(
                    "color: red;")

    def stop_all_motors(self):
        """Emergency stop all motors"""
        if not self.afm.is_connected():
            return

        response = self.afm.stop_motor()

        # Update all status indicators
        for motor_num in range(1, 4):
            if motor_num in self.motor_status_labels:
                self.motor_status_labels[motor_num].setText("Idle")
                self.motor_status_labels[motor_num].setStyleSheet(
                    "color: gray;")

        if response.get("status") != "stopped":
            QMessageBox.warning(self, "Stop Error", "Failed to stop motors")

    def update_motor_status(self, status):
        self.motor_running_radio_button.setChecked(
            status.motor_1.is_running or
            status.motor_2.is_running or
            status.motor_3.is_running)

        for i in range(1, 4):
            motor_status = getattr(status, f'motor_{i}')
            label = getattr(self, f'motor_{i}_status_label')

            if motor_status.is_running:
                label.setText("RUNNING")
                label.setStyleSheet("color: green; font-weight: bold;")
            else:
                label.setText("IDLE")
                label.setStyleSheet("color: gray;")

    def open_focus_widget(self):
        """Opens the focus widget."""
        self.focus_widget = FocusWidget(self.afm)
        self.focus_widget.show()

    def open_approach_widget(self):
        """Opens the approach widget."""
        self.approach_widget = ApproachWidget(self.afm)
        self.approach_widget.show()

    def open_scan_widget(self):
        """Opens the scan widget."""
        self.scan_widget = ScanWidget(self.afm)
        self.scan_widget.show()

    def restore_afm(self):
        """Restore AFM to default settings."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Error", "AFM is not connected")
            return

        try:
            if self.afm.restore():
                QMessageBox.information(
                    self, "Success", "AFM restored to default settings")
            else:
                QMessageBox.warning(
                    self, "Error", "Failed to restore AFM settings")
        except Exception as e:
            QMessageBox.critical(
                self, "Error", f"Error during restore: {str(e)}")

    def on_target_adc_changed(self):
        """Handle changes to the target ADC value input - Apply parameters if PID is enabled"""
        if not self.afm.is_connected():
            return

        try:
            # Get the new target value
            target = None
            if self.target_adc_value_input.text():
                try:
                    target = int(self.target_adc_value_input.text())
                except ValueError:
                    QMessageBox.warning(self, "Invalid Input", "Target ADC must be a valid integer.")
                    # Optionally revert the text or clear it
                    # self.target_adc_value_input.setText("")
                    return # Don't proceed if invalid

            # Apply parameters only if PID is currently enabled
            if self.pid_enabled_radio.isChecked():
                 # Get other parameters from GUI
                 kp = float(self.p_input_box.text())
                 ki = float(self.i_input_box.text())
                 kd = float(self.d_input_box.text())
                 invert = self.pid_reverse_button.isChecked()

                 print(f"Target ADC changed while PID enabled. Applying: target={target}")
                 success = self.afm.set_pid_parameters(kp=kp, ki=ki, kd=kd, invert=invert, target=target)
                 if success:
                    # Re-enable to make sure target takes effect (firmware might require this)
                    enable_success = self.afm.enable_pid(target=target)
                    if not enable_success:
                         QMessageBox.warning(self, "PID Error", "Failed to re-enable PID after target change.")
                         self.refresh_pid_parameters_from_hw()
                 else:
                    QMessageBox.warning(self, "PID Error", "Failed to set new PID target.")
                    self.refresh_pid_parameters_from_hw()
            else:
                # If PID is not enabled, just store the value, don't send anything yet.
                # The value will be used when enable_pid_control is called.
                print(f"Target ADC changed (PID disabled): {target}")

        except ValueError:
            # This handles potential float conversion errors for kp,ki,kd if they are invalid
            # The target int conversion error is handled above.
            QMessageBox.warning(self, "Invalid Input",
                              "Please enter valid numbers for PID parameters (P, I, D).")
        except Exception as e:
            QMessageBox.warning(self, "PID Error", f"Error handling target ADC change: {e}")
            self.refresh_pid_parameters_from_hw() # Sync GUI on unexpected error

    def set_adc_average_window(self):
        """Read the value from avg_points_input and send it to the AFM."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to AFM first.")
            return

        try:
            window_size = int(self.avg_points_input.text())
            if window_size < 1:
                raise ValueError("Window size must be at least 1")

            success = self.afm.set_adc_avg_window(window_size)

            if success:
                self.statusbar.showMessage(f"ADC average window set to {window_size}.", 2000)
            else:
                QMessageBox.warning(self, "ADC Avg Error", "Failed to set ADC average window size on AFM.")
                # Optional: Read back the actual value if the command supported it

        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid positive integer for average points.")
            # Reset to a default or last known good value? For now, just warn.
            # self.avg_points_input.setText("3")
        except Exception as e:
            QMessageBox.critical(self, "ADC Avg Error", f"Error setting ADC average window: {str(e)}")

    def update_plot_history(self):
        """Update the time window used for plotting."""
        try:
            new_window_s = float(self.plot_history_input.text())
            if new_window_s <= 0:
                raise ValueError("Plot history must be positive")
            self._plot_time_window_s = new_window_s
            print(f"Plot history window updated to: {self._plot_time_window_s} s")
            # Force plot update maybe? Or let the regular timer handle it.
            # self.update_plots(list(self.afm.status_queue)[-1]) # Example immediate update
        except ValueError:
             QMessageBox.warning(self, "Invalid Input", "Please enter a valid positive number for plot history (seconds).")
             # Revert to previous value
             self.plot_history_input.setText(str(self._plot_time_window_s))

    def set_dac_range(self):
        """Handle DAC range setting."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected", "Please connect to AFM first.")
            return

        try:
            range_mode = self.xy_range_input_box.currentText()
            if range_mode not in ['10V', '3V']:
                QMessageBox.warning(self, "Invalid Selection", "Range must be either '10V' or '3V'")
                return

            success = self.afm.set_dac_range(range_mode)

            if success:
                QMessageBox.information(self, "Success", f"DAC range set to {range_mode}")
            else:
                QMessageBox.warning(self, "Error", "Failed to set DAC range")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error setting DAC range: {str(e)}")

    def closeEvent(self, event):
        """Ensure child widgets and AFM connection are closed."""
        print("MainWindow close event triggered.")

        # Close child widgets if they exist
        if self.focus_widget:
            print("Closing FocusWidget...")
            self.focus_widget.close()
        if self.approach_widget:
            print("Closing ApproachWidget...")
            self.approach_widget.close()
        if self.scan_widget:
            print("Closing ScanWidget...")
            self.scan_widget.close()

        # Close AFM connection
        self.afm_close()

        # Accept the close event
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec()) # This is crucial!
