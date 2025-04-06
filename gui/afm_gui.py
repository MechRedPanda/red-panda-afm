import sys

import pyqtgraph as pg

from PyQt6.QtCore import QEvent, QObject, QTimer, Qt
from PyQt6.QtGui import QIntValidator
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox

from afm import AFM
from ui.focus_widget import Ui_FocusWidget
from ui.main_window import Ui_MainWindow


class FocusWidget(QMainWindow, Ui_FocusWidget):
    def __init__(self, afm: AFM):
        super().__init__()
        self.setupUi(self)
        self.afm = afm  # Store the AFM instance
        self.f_start.setText("10000")
        self.f_end.setText("50000")
        self.step_size.setText("10")

        self.start_focus_button.clicked.connect(self.start_focus)
        self.stop_focus_button.clicked.connect(self.stop_focus)

        self.focus_plot_widget.setTitle(
            "ADC vs DAC_F", color="r", size="12pt")
        self.focus_plot_widget.setLabel(
            'left', 'adc_0 Value', color='black', size=14)
        self.focus_plot_widget.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.focus_plot_widget.showGrid(x=True, y=True)
        self.focus_plot_widget.setBackground('white')
        self.adc_0_plot = self.focus_plot_widget.plot(
            pen=pg.mkPen('b', width=5))  # Green line
        self.adc_1_plot = self.focus_plot_widget.plot(
            pen=pg.mkPen('r', width=5))  # Green line

        # refresh timer
        # Timer to periodically update QLabel
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update every 100ms

    def start_focus(self):
        self.focusing_button.setChecked(True)
        self.afm.focus(
            start=int(self.f_start.text()),
            end=int(self.f_end.text()),
            step_size=int(self.step_size.text()),
        )

    def stop_focus(self):
        self.afm.stop_focus()
        self.focusing_button.setChecked(False)

    def update_plot(self):
        """Update plot with the latest focus data"""
        if self.afm.focus_results:
            # Extract DAC_F and ADC values
            dac_f_values = [result[0] for result in self.afm.focus_results]
            adc_0_values = [result[1] for result in self.afm.focus_results]
            adc_1_values = [result[2] for result in self.afm.focus_results]

            # Update the plot
            self.adc_0_plot.setData(dac_f_values, adc_0_values)
            self.adc_1_plot.setData(dac_f_values, adc_1_values)


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        # AFM init
        self.afm = AFM()

        # Initialize UI state
        self.connected_button.setEnabled(False)
        self.connected_button.setChecked(False)

        self.refresh_button.clicked.connect(self.refresh_ports)

        # Connect button signals to functions
        self.connect_button.clicked.connect(self.handle_connect)
        self.reset_button.clicked.connect(self.afm_reset)
        self.close_button.clicked.connect(self.afm_close)

        # refresh timer
        # Timer to periodically update QLabel
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update every 100ms

        self.adc_plot_widget.setTitle(
            "Real-time AFM Data (adc_0)", color="r", size="12pt")
        self.adc_plot_widget.setLabel(
            'left', 'adc_0 Value', color='black', size=14)
        self.adc_plot_widget.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.adc_plot_widget.showGrid(x=True, y=True)
        self.adc_plot_widget.setBackground('white')
        self.adc_0_plot_curve = self.adc_plot_widget.plot(
            pen=pg.mkPen('b', width=5))  # Green line
        self.adc_1_plot_curve = self.adc_plot_widget.plot(
            pen=pg.mkPen('r', width=5))  # Green line

        self.dac_plot_widget.setTitle(
            "Real-time AFM Data DACs", color="r", size="12pt")
        self.dac_plot_widget.setLabel(
            'left', 'adc_1 Value', color='black', size=14)
        self.dac_plot_widget.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.dac_plot_widget.showGrid(x=True, y=True)
        self.dac_plot_widget.setBackground('white')
        self.dac_x_plot_curve = self.dac_plot_widget.plot(
            pen=pg.mkPen('b', width=5))
        self.dac_y_plot_curve = self.dac_plot_widget.plot(
            pen=pg.mkPen('r', width=5))
        self.dac_z_plot_curve = self.dac_plot_widget.plot(
            pen=pg.mkPen('g', width=5))

        # Initialize plot curve
        # focus widget
        self.focus_widget_button.clicked.connect(self.open_focus_widget)

        # Setup DAC UI
        self.setup_dac_ui()

        # Setup motor UI
        self.setup_motor_ui()

    def setup_dac_ui(self):
        # DAC sliders and input boxes
        self.max_value = 2 ** 16 - 1  # Max value for DAC (16-bit)
        self.dac_input_pairs = {
            "f": (self.f_slider, self.f_input_box),
            "t": (self.t_slider, self.t_input_box),
            "x": (self.x_slider, self.x_input_box),
            "y": (self.y_slider, self.y_input_box),
            "z": (self.z_slider, self.z_input_box),
        }

        # DAC set buttons
        self.dac_set_buttons = {
            "f": self.f_set_button,
            "t": self.t_set_button,
            "x": self.x_set_button,
            "y": self.y_set_button,
            "z": self.z_set_button,
        }

        for key, (slider, lineedit) in self.dac_input_pairs.items():
            slider.setRange(0, self.max_value)
            lineedit.setValidator(QIntValidator(0, self.max_value))

            # Set default value
            default_val = 2 ** 15
            slider.setValue(default_val)
            lineedit.setText(str(default_val))

            # Sync logic
            slider.valueChanged.connect(
                lambda val, k=key: self.sync_lineedit_from_slider(k, val))
            lineedit.editingFinished.connect(
                lambda k=key: self.sync_slider_from_lineedit(k))

        for key, button in self.dac_set_buttons.items():
            button.clicked.connect(lambda _, k=key: self.send_dac_value(k))

    def sync_lineedit_from_slider(self, key, value):
        _, lineedit = self.dac_input_pairs[key]
        lineedit.blockSignals(True)
        lineedit.setText(str(value))
        lineedit.blockSignals(False)

    def sync_slider_from_lineedit(self, key):
        slider, lineedit = self.dac_input_pairs[key]
        text = lineedit.text()
        try:
            value = int(text)
            if 0 <= value <= self.max_value:
                slider.blockSignals(True)
                slider.setValue(value)
                slider.blockSignals(False)
            else:
                value = max(0, min(self.max_value, value))
                lineedit.setText(str(value))
                slider.setValue(value)
        except ValueError:
            lineedit.setText(str(slider.value()))

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

    def send_dac_value(self, key):
        _, lineedit = self.dac_input_pairs[key]
        value = int(lineedit.text())
        # Replace with real communication logic
        print(f"[AFM] Set {key.upper()} to {value}")
        self.afm.set_dac(key.upper(), value)

    def setup_motor_ui(self):
        """Initialize motor control UI elements"""
        # Set up input validation
        int_validator = QIntValidator()
        self.step_motor_1_input_box.setValidator(int_validator)

        # Default values
        self.step_motor_1_input_box.setText("100")  # Default step count
        self.motor_running_radio_button.setChecked(False)

        # Connect signals
        self.step_motor_1_button.clicked.connect(self.start_motor_1_movement)
        self.stop_motors_button.clicked.connect(self.stop_all_motors)

    def start_motor_1_movement(self):
        """Handle motor 1 movement start"""
        if not self.afm.is_connected():
            QMessageBox.warning(self, "Not Connected",
                                "Please connect to AFM first")
            return

        try:
            steps = int(self.step_motor_1_input_box.text())
            if steps <= 0:
                raise ValueError("Steps must be positive")

            # Start movement (CW direction by default)
            response = self.afm.move_motor(
                motor=1,
                steps=steps,
                speed=300  # Medium speed
            )

            if response.get("status") != "started":
                QMessageBox.warning(self, "Movement Error",
                                    f"Failed to start motor: {response.get('message', 'Unknown error')}")

        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input", str(e))

    def stop_all_motors(self):
        """Emergency stop all motors"""
        if not self.afm.is_connected():
            return

        response = self.afm.stop_motor()
        if response.get("status") != "stopped":
            QMessageBox.warning(self, "Stop Error",
                                "Failed to stop motors")

    def update_plot(self):
        """Update plot with the latest AFM data"""
        if not self.afm.is_connected() or self.afm.busy:
            return
        print(self.afm.get_status())  # Update the AFM status

        if self.afm.status_queue:
            latest_data = list(self.afm.status_queue)  # Get the queue contents

            # Extract timestamp and adc_0 values
            x_data = [(entry.timestamp - self.afm.status_queue[0].timestamp) /
                      1000 for entry in latest_data]
            adc_0_data = [entry.adc_0 for entry in latest_data]
            adc_1_data = [entry.adc_1 for entry in latest_data]
            dac_x_data = [entry.dac_x for entry in latest_data]
            dac_y_data = [entry.dac_y for entry in latest_data]
            dac_z_data = [entry.dac_z for entry in latest_data]

            # Update the plot
            self.adc_0_plot_curve.setData(x_data, adc_0_data)
            self.adc_1_plot_curve.setData(x_data, adc_1_data)
            self.dac_x_plot_curve.setData(x_data, dac_x_data)
            self.dac_y_plot_curve.setData(x_data, dac_y_data)

    def open_focus_widget(self):
        """Opens the focus widget."""
        self.focus_widget = FocusWidget(self.afm)
        self.focus_widget.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
