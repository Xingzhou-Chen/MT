import select
import sys
import time

# import machine

# # Define UART pins (change pin numbers as per your setup)
# uart = machine.UART(0, baudrate=9600)  # UART0, baudrate 9600, TX pin GP3, RX pin GP4


# Set up the poll object
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

# Loop indefinitely
while True:
    # Wait for input on stdin
    poll_results = poll_obj.poll(1) # the '1' is how long it will wait for message before looping again (in microseconds)
    if poll_results:
        # Read the data from stdin (read data coming from PC)
        data = sys.stdin.readline().strip()
        # Write the data to the input file
        sys.stdout.write("received data: " + data + "\r")
    #     print("testing")
    else:
        # do something if no message received (like feed a watchdog timer)
        continue
    # print("testiing")
    # uart.write("Hello, PC!\n")