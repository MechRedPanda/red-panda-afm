import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import Qt
from PyQt6.QtCore import QTimer
import pyqtgraph as pg
from ui.main_window import Ui_MainWindow  # Import the generated UI class
from afm_wifi import AFM


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        # AFM init
        self.afm = AFM()

        # Initialize UI state
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

        self.plot_widget_0.setTitle(
            "Real-time AFM Data (adc_0)", color="r", size="12pt")
        self.plot_widget_0.setLabel(
            'left', 'adc_0 Value', color='black', size=14)
        self.plot_widget_0.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.plot_widget_0.showGrid(x=True, y=True)
        self.plot_widget_0.setBackground('white')
        self.plot_curve_0 = self.plot_widget_0.plot(
            pen=pg.mkPen('b', width=5))  # Green line

        self.plot_widget_1.setTitle(
            "Real-time AFM Data (adc_1)", color="r", size="12pt")
        self.plot_widget_1.setLabel(
            'left', 'adc_1 Value', color='black', size=14)
        self.plot_widget_1.setLabel(
            'bottom', 'Time (seconds)', color='black', size=14)
        self.plot_widget_1.showGrid(x=True, y=True)
        self.plot_widget_1.setBackground('white')
        self.plot_curve_1 = self.plot_widget_1.plot(
            pen=pg.mkPen('b', width=5))  # Green line

        # Initialize plot curve

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

    def update_label(self):
        if not self.afm.status_queue:
            return
        afm_status = self.afm.status_queue[-1]
        self.textBrowser.setText(
            f"afm_status:\n"
            f"adc_0: {afm_status.adc_0}\n"
            f"timestamp: {afm_status.timestamp}\n")

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
            self.plot_curve_0.setData(x_data, adc_0_data)
            self.plot_curve_1.setData(x_data, adc_1_data)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
