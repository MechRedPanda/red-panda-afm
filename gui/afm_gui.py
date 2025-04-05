import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import Qt, QObject, QEvent
from PyQt6.QtCore import QTimer
import pyqtgraph as pg
from ui.main_window import Ui_MainWindow  # Import the generated UI class
from ui.focus_widget import Ui_FocusWidget  # Import the generated UI class
from afm import AFM


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
            adc_1_values = [result[2]+100 for result in self.afm.focus_results]

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

        # Connect button signals to functions
        self.connect_button.clicked.connect(self.connect_action)
        self.F_slider.valueChanged.connect(self.F_slider_changed)
        self.T_slider.valueChanged.connect(self.T_slider_changed)
        self.F_value.setText(str(self.F_slider.value()))
        self.T_value.setText(str(self.T_slider.value()))
        self.reset_button.clicked.connect(self.afm_reset)
        self.close_button.clicked.connect(self.afm_close)

        # refresh timer
        # Timer to periodically update QLabel
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update every 100ms

        self.adc_0_plot_widget.setTitle(
            "Real-time AFM Data (adc_0)", color="r", size="12pt")
        self.adc_0_plot_widget.setLabel(
            'left', 'adc_0 Value', color='black', size=14)
        self.adc_0_plot_widget.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.adc_0_plot_widget.showGrid(x=True, y=True)
        self.adc_0_plot_widget.setBackground('white')
        self.adc_0_plot_curve = self.adc_0_plot_widget.plot(
            pen=pg.mkPen('b', width=5))  # Green line

        self.adc_1_plot_widget.setTitle(
            "Real-time AFM Data (adc_1)", color="r", size="12pt")
        self.adc_1_plot_widget.setLabel(
            'left', 'adc_1 Value', color='black', size=14)
        self.adc_1_plot_widget.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.adc_1_plot_widget.showGrid(x=True, y=True)
        self.adc_1_plot_widget.setBackground('white')
        self.adc_1_plot_curve = self.adc_1_plot_widget.plot(
            pen=pg.mkPen('b', width=5))  # Green line

        # Initialize plot curve
        # focus widget
        self.focus_widget_button.clicked.connect(self.open_focus_widget)

    def connect_action(self):
        """Handles the Connect button click."""
        try:
            # Attempt to connect to the AFM device
            self.afm.connect(host=self.ip_box.text(),
                             port=int(self.port_box.text()))
            self.connected_button.setChecked(True)
            print("Connected!")
        except Exception as e:
            print(f"Connection failed: {e}")
            return

    def afm_reset(self):
        """Resets the AFM device."""
        try:
            self.afm.reset()
            print("AFM reset command sent.")
        except Exception as e:
            print(f"Failed to send reset command: {e}")

    def afm_close(self):
        """Closes the AFM connection."""
        try:
            self.afm.close()
            self.connected_button.setChecked(False)
            print("AFM connection closed.")
        except Exception as e:
            print(f"Failed to close AFM connection: {e}")

    def F_slider_changed(self, value):
        """Handles the first slider value change."""
        print(f"Slider 1 Value: {value}")
        self.F_value.setText(str(value))

    def T_slider_changed(self, value):
        """Handles the second slider value change."""
        print(f"Slider 2 Value: {value}")
        self.T_value.setText(str(value))

    def update_plot(self):
        """Update plot with the latest AFM data"""

        if self.afm.status_queue:
            latest_data = list(self.afm.status_queue)  # Get the queue contents

            # Extract timestamp and adc_0 values
            x_data = [(entry.timestamp - latest_data[0].timestamp).total_seconds()
                      for entry in latest_data]
            adc_0_data = [entry.adc_0 for entry in latest_data]
            adc_1_data = [entry.adc_1 for entry in latest_data]

            # Update the plot
            self.adc_0_plot_curve.setData(x_data, adc_0_data)
            self.adc_1_plot_curve.setData(x_data, adc_1_data)

    def open_focus_widget(self):
        """Opens the focus widget."""
        self.focus_widget = FocusWidget(self.afm)
        self.focus_widget.show()

        self.focus_widget.setWindowFlags(
            Qt.WindowStaysOnTopHint)  # Optional: Keep it on top


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
