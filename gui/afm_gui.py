import sys
import time
from PyQt6.QtCore import QEvent, QTimer, Qt
from PyQt6.QtGui import QIntValidator
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox, QCheckBox, QWidget, QVBoxLayout
import pyqtgraph as pg
from afm import AFM, AFMState
from ui.focus_widget import Ui_FocusWidget
from ui.main_window import Ui_MainWindow
from ui.approach_widget import Ui_ApproachWidget


class FocusWidget(QMainWindow, Ui_FocusWidget):
    def __init__(self, afm: AFM):
        super().__init__()
        self.setupUi(self)
        self.afm = afm
        self.f_start.setText("10000")
        self.f_end.setText("50000")
        self.step_size.setText("10")
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
        self.adc_1_plot = self.focus_plot_widget.plot(
            pen=pg.mkPen('r', width=5), name="ADC 1")
            
        # Add vertical line for optimal focus value
        self.optimal_line = self.focus_plot_widget.addLine(x=None, angle=90, pen=pg.mkPen('g', width=2, style=Qt.PenStyle.DashLine), name="Optimal Focus")
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
                QMessageBox.information(self, "Success", f"Moving to optimal focus point: {optimal_value}")
            else:
                QMessageBox.warning(self, "Error", "Failed to move to optimal focus point")
        else:
            QMessageBox.warning(self, "Error", "No optimal focus point available")

    def update_plot(self):
        if self.afm.focus_results:
            dac_f_values = [result[0] for result in self.afm.focus_results]
            adc_0_values = [result[1] for result in self.afm.focus_results]
            adc_1_values = [result[2] for result in self.afm.focus_results]

            self.adc_0_plot.setData(dac_f_values, adc_0_values)
            self.adc_1_plot.setData(dac_f_values, adc_1_values)
            
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
                    self.optimal_line.setVisible(False)  # Hide the line if no optimal value
                    QMessageBox.warning(
                        self,
                        "Auto-Focus Results",
                        "No optimal focus point could be determined."
                    )


# Assuming you have a corresponding UI file
class ApproachWidget(QMainWindow, Ui_ApproachWidget):
    def __init__(self, afm: AFM):
        super().__init__()
        self.setupUi(self)
        self.afm = afm

        # Set default values
        self.motor_number.setValue(1)  # Default to motor 1
        self.step_size.setText("1000")
        self.max_steps.setText("500000")
        self.adc_channel.addItem("ADC 0")
        self.adc_channel.addItem("ADC 1")
        self.adc_channel.setCurrentIndex(0)  # Default to ADC 0
        self.threshold.setText("10")

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
        self.steps_data = []
        self.adc_data = []

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
            print("AFM not connected")
            return

        if self.afm.is_approach_running():
            print("Approach already running")
            return

        # Get parameters from UI
        motor = self.motor_number.value()
        step_size = int(self.step_size.text())
        max_steps = int(self.max_steps.text()
                        ) if self.max_steps.text() else None
        adc_channel = self.adc_channel.currentIndex()  # 0 or 1
        threshold = int(self.threshold.text())

        # Get initial status
        status = self.afm.get_status()
        if not status:
            print("Failed to get initial status")
            return

        # Store initial values
        initial_position = getattr(status, f"motor_{motor}").position
        initial_adc = status.adc_0 if adc_channel == 0 else status.adc_1

        # Clear previous data
        self.steps_data = []
        self.adc_data = []

        # Plot threshold lines
        upper_thresh = initial_adc + threshold
        lower_thresh = initial_adc - threshold
        self.threshold_line_upper.setData([0, max_steps if max_steps else 10],
                                          [upper_thresh, upper_thresh])
        self.threshold_line_lower.setData([0, max_steps if max_steps else 10],
                                          [lower_thresh, lower_thresh])

        # Start approach
        self.approaching_button.setChecked(True)
        success = self.afm.approach(
            motor=motor,
            step_size=step_size,
            adc_channel=adc_channel,
            threshold=threshold,
            max_steps=max_steps,
            polling_interval=0.05
        )

        if not success:
            self.approaching_button.setChecked(False)
            print("Failed to start approach")

    def stop_approach(self):
        """Stop the approach procedure."""
        print("Stopping approach")
        self.afm.stop_approach()
        self.approaching_button.setChecked(False)

    def update_display(self):
        """Update the display with current approach status."""
        if self.afm.approach_data:
            # Extract steps and ADC values from the stored data
            steps_data = [point['steps'] for point in self.afm.approach_data]
            adc_data = [point['adc'] for point in self.afm.approach_data]
            if len(steps_data) > 1:
                # Update plot with the stored data
                self.adc_plot.setData(steps_data, adc_data)
                
                # Update threshold lines to match the data range
                if len(steps_data) > 0:
                    # Get initial ADC value for threshold lines
                    initial_adc = adc_data[0]
                    threshold = int(self.threshold.text())
                    upper_thresh = initial_adc + threshold
                    lower_thresh = initial_adc - threshold
                    
                    # Set threshold lines to span the same range as the data
                    self.threshold_line_upper.setData([steps_data[0], steps_data[-1]], 
                                                     [upper_thresh, upper_thresh])
                    self.threshold_line_lower.setData([steps_data[0], steps_data[-1]], 
                                                     [lower_thresh, lower_thresh])
            
        else:
            if self.approaching_button.isChecked():
                self.approaching_button.setChecked(False)



class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.afm = AFM()

        # DAC control state
        self.dac_update_lock = False
        self.focused_dac_control = None
        self.last_dac_update_time = 0

        self.init_ui()

    def init_ui(self):
        self.connected_button.setEnabled(False)
        self.connected_button.setChecked(False)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.handle_connect)
        self.reset_button.clicked.connect(self.afm_reset)
        self.close_button.clicked.connect(self.afm_close)
        self.restore_button.clicked.connect(self.restore_afm)

        self.setup_plots()
        self.setup_dac_controls()
        self.setup_motor_ui()
        self.setup_update_toggle()

        self.focus_widget_button.clicked.connect(self.open_focus_widget)
        self.approach_widget_button.clicked.connect(self.open_approach_widget)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_all)
        self.update_timer.start(100)

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
            pen=pg.mkPen('b', width=2), name="ADC 0")
        self.adc_1_plot_curve = self.adc_plot_widget.plot(
            pen=pg.mkPen('r', width=2), name="ADC 1")

        self.dac_plot_widget.setTitle("DAC Values", color="r", size="12pt")
        self.dac_plot_widget.setLabel(
            'left', 'DAC Value', color='black', size=14)
        self.dac_plot_widget.setLabel(
            'bottom', 'Time (s)', color='black', size=14)
        self.dac_plot_widget.showGrid(x=True, y=True)
        self.dac_plot_widget.setBackground('white')
        self.dac_plot_widget.addLegend()
        self.dac_f_plot = self.dac_plot_widget.plot(pen=pg.mkPen('b', width=2), name="DAC F")
        self.dac_t_plot = self.dac_plot_widget.plot(pen=pg.mkPen('g', width=2), name="DAC T")

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

    def setup_update_toggle(self):
        """Add a toggle switch for enabling/disabling automatic updates"""
        self.update_toggle.setChecked(True)
        self.update_toggle.setStyleSheet("""
            QCheckBox {
                padding: 5px;
                font-weight: bold;
            }
            QCheckBox::indicator {
                width: 20px;
                height: 20px;
            }
        """)

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
                if not self.update_toggle.isChecked():
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
            if not self.update_toggle.isChecked():
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
                if not self.update_toggle.isChecked():
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
        if not self.update_toggle.isChecked():
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

        if self.update_toggle.isChecked():
            self.update_dac_controls(latest_status)
        self.update_plots(latest_status)
        self.update_motor_status(latest_status)

    def update_plots(self, status):
        if len(self.afm.status_queue) < 2:
            return

        timestamps = [s.timestamp - self.afm.status_queue[0].timestamp
                      for s in self.afm.status_queue]
        seconds = [t/1000 for t in timestamps]

        adc0 = [s.adc_0 for s in self.afm.status_queue]
        adc1 = [s.adc_1 for s in self.afm.status_queue]
        self.adc_0_plot_curve.setData(seconds, adc0)
        self.adc_1_plot_curve.setData(seconds, adc1)

        dacf = [s.dac_f for s in self.afm.status_queue]
        dact = [s.dac_t for s in self.afm.status_queue]
        self.dac_f_plot.setData(seconds, dacf)
        self.dac_t_plot.setData(seconds, dact)

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

    def restore_afm(self):
        """Restore AFM to default settings."""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Error", "AFM is not connected")
            return
            
        try:
            if self.afm.restore():
                QMessageBox.information(self, "Success", "AFM restored to default settings")
            else:
                QMessageBox.warning(self, "Error", "Failed to restore AFM settings")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error during restore: {str(e)}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
