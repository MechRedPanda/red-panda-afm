import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import Qt
from ui.main_window import Ui_MainWindow  # Import the generated UI class
from afm import AFM


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Initialize UI state
        self.port_connected.setChecked(False)

        # Connect button signals to functions
        self.port_connect.clicked.connect(self.connect_action)
        self.port_close.clicked.connect(self.close_action)
        self.port_refresh.clicked.connect(self.refresh_ports)
        self.F_slider.valueChanged.connect(self.F_slider_changed)
        self.T_slider.valueChanged.connect(self.T_slider_changed)

        # AFM init
        self.afm = AFM()

    def refresh_ports(self):
        list_ports = self.afm.get_list_ports()
        print("Available serial ports:", list_ports)
        self.port_lists_box.clear()
        for port in list_ports:
            self.port_lists_box.addItem(port)

    def connect_action(self):
        """Handles the Connect button click."""
        self.afm.connect(self.port_lists_box.currentText(), 115200)
        self.port_connected.setChecked(True)
        print("Connected!")

    def close_action(self):
        """Handles the Close button click."""
        self.afm.close()
        self.port_connected.setChecked(False)
        print("Closed!")

    def F_slider_changed(self, value):
        """Handles the first slider value change."""
        print(f"Slider 1 Value: {value}")
        self.F_value.setText(str(value))

    def T_slider_changed(self, value):
        """Handles the second slider value change."""
        print(f"Slider 2 Value: {value}")
        self.T_value.setText(str(value))


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
