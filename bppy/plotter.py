import serial
import matplotlib.pyplot as plt
from drawnow import drawnow

# Initialize serial communication
ser = serial.Serial('/dev/cu.usbmodem14201', 9600)  # Replace 'COM3' with your serial port

# Initialize plot
plt.ion()
data = []

# Function to plot data
def plot_data():
    plt.title('Serial Plotter')
    plt.xlabel('Time')
    plt.ylabel('Data')
    plt.grid(True)
    plt.plot(data, 'r-')  # 'ro-' means red color, circles, solid line
    # plt.pause(0.05)

# Main loop to continuously read data from serial port and plot it
while True:
    while ser.inWaiting() == 0:
        pass
    value = float(ser.readline().decode('utf-8').strip())
    data.append(value)
    drawnow(plot_data)
