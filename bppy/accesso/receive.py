import serial

# # Open the serial port (change the port name as per your system)
ser = serial.Serial('/dev/cu.usbmodem14201', 9600)  # COM3 is just an example, replace it with the actual port

# import serial


def main():
    s = serial.Serial(port="/dev/cu.usbmodem14201", parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=1)
    s.flush()

    # s.write("data\r".encode())
    mes = s.read_until().strip()
    print(mes.decode())


if __name__ == "__main__":
    while True:
        main()