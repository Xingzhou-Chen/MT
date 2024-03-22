import sys
import serial
import numpy as np
import pyqtgraph as pg
from serial.tools import list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QCheckBox, QPushButton, QComboBox, QMessageBox, QFileDialog
from PyQt5.QtCore import QTimer

class SerialPlotter(QMainWindow):
    def __init__(self, port='COM7', baudrate=115200):
        super().__init__()
        
        self.serial_port = None
        self.data = {}
        self.x_data = {}
        self.active_lines = {}
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.plot_running = False
        
        if port != None:
            self.open_serial_port(port, baudrate)
        
        self.init_ui()

    def init_ui(self):
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)

        self.curves = {}
        
        self.line_select_layout = QVBoxLayout()
        self.line_select_widget = QWidget()
        self.line_select_widget.setLayout(self.line_select_layout)
        
        self.central_layout = QVBoxLayout()
        self.central_layout.addWidget(self.plot_widget)
        self.central_layout.addWidget(self.line_select_widget)
        
        self.start_stop_button = QPushButton("Start")
        self.start_stop_button.clicked.connect(self.toggle_plot)

        self.save_button = QPushButton("Save to CSV")
        self.save_button.clicked.connect(self.save_to_csv)
        
        self.port_dropdown = QComboBox()
        self.populate_ports()
        self.port_dropdown.currentIndexChanged.connect(self.selected_port_changed)
        
        self.central_layout.addWidget(self.port_dropdown)
        
        for i in range(5):  # Assuming there are at most 5 lines
            checkbox = QCheckBox(f'Line {i+1}')
            checkbox.setChecked(True)
            checkbox.stateChanged.connect(lambda state, index=i: self.toggle_line(index, state == 2))
            self.line_select_layout.addWidget(checkbox)
        
        self.central_layout.addWidget(self.start_stop_button)
        self.central_layout.addWidget(self.save_button)
        
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.central_layout)
        self.setCentralWidget(self.central_widget)
        
    def toggle_plot(self):
        if not self.plot_running:
            if not self.serial_port:
                QMessageBox.warning(self, "Error", "Please select a COM port.")
                return
            self.start_stop_button.setText("Stop")
            self.timer.start(50)  # Update plot every 100 ms
            self.plot_running = True
        else:
            self.start_stop_button.setText("Start")
            self.timer.stop()
            self.plot_running = False

    def populate_ports(self):
        ports = [port.device for port in list_ports.comports()]
        ports.append("Dummy")
        self.port_dropdown.addItems(ports)

    def selected_port_changed(self):
        port = self.port_dropdown.currentText()
        self.open_serial_port(port)

    def open_serial_port(self, port, baudrate=115200):
        try:
            if self.serial_port:
                self.serial_port.close()
            self.serial_port = serial.Serial(port, baudrate)
        except serial.SerialException as e:
            QMessageBox.warning(self, "Error", str(e))
            self.serial_port = None

    def toggle_line(self, line_index, state):
        self.active_lines[line_index] = state

        # Clear the plot
        self.plot_widget.clear()

        # Replot active lines
        for i in self.active_lines:
            if self.active_lines[i]:
                self.curves[i] = self.plot_widget.plot(self.x_data[i], self.data[i], pen=(i, len(self.curves)))


    def update_plot(self):
        while self.serial_port != None and self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            try:
                values = line.split(',')
                for i, val in enumerate(values):    
                    if i not in self.data:
                        self.data[i] = np.zeros(500)  # Initialize with zeros
                        self.x_data[i] = np.arange(500)
                        self.curves[i] = self.plot_widget.plot(self.x_data[i], self.data[i], pen=(i, len(values)))
                        self.active_lines[i] = True  # Initially, all lines are active

                    data_point = float(val)
                    self.data[i] = np.roll(self.data[i], 1)  # Roll data to the left
                    self.data[i][0] = data_point  # Update the last value with new data
                    if self.active_lines[i]:
                        self.curves[i].setData(self.x_data[i], self.data[i])
            except ValueError:
                print("Invalid data:", line)

    def save_to_csv(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save to CSV", "", "CSV Files (*.csv)")
        if filename:
            with open(filename, 'w') as f:
                # Write header
                f.write("Time,")
                for i in range(len(self.active_lines)):
                    if self.active_lines[i]:
                        f.write(f"Line {i+1},")
                f.write("\n")

                # Write data
                for j in range(len(self.x_data[0])):
                    f.write(f"{self.x_data[0][j]},")
                    for i in range(len(self.active_lines)):
                        if self.active_lines[i]:
                            f.write(f"{self.data[i][j]},")
                    f.write("\n")

    def closeEvent(self, event):
        if self.serial_port:
            self.serial_port.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = SerialPlotter()
    window.setWindowTitle('Serial Plotter')
    window.setGeometry(100, 100, 800, 600)
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
