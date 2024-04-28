import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate(i, yList, xList,ser):
    # ser.write(b'g')                                     # Transmit the char 'g' to receive the Arduino data point
    data_string = ser.readline().decode('ascii') # Decode receive Arduino data as a formatted string
    # data_string = ser.read(100000).decode('ascii')
    # data_string = ser.read().decode('ascii')
    print(data_string)
    # print(type(data_string))
    # print(1)                                           # 'i' is a incrementing variable based upon frames = x argument
    data_string = data_string.split(',')
    # print(data_string)
    x_string=data_string[0]
    y_string=data_string[1]
    try:
        # print(dataList)
        y_float = float(y_string)   # Convert to float
        yList.append(y_float)              # Add to the list holding the fixed number of points to animate
        # print(dataList)
        # print(x_string)
        x_float = float(x_string)
        # print(dataList)
        xList.append(x_float)
        # print(f'Time:{x_float},Y:{y_float}')

    except:                                             # Pass if data point is bad                               
        pass

    yList = yList[-50000:]                           # Fix the list size so that the animation plot 'window' is x number of points
    xList = xList[-50000:]

    # print(dataList)
    ax.clear()                                          # Clear last data frame
    ax.plot(xList,yList)                                   # Plot new data frame
    
    # ax.set_ylim([0, 12000])                              # Set Y axis limit of plot
    # ax.set_xlim([0, x_string]) 
    ax.set_title("Arduino Data")                        # Set title of figure
    ax.set_ylabel("Value")                              # Set title of y axis 

yList = []                                           # Create empty list variable for later use
xList = []

fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                               # Add subplot to main fig window

# ser = serial.Serial("/dev/tty.usbserial-14420", 9600)  #/dev/cu.usbmodem14201                     # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
# ser = serial.Serial("/dev/tty.usbmodem14401", 115200)  #/dev/cu.usbmodem14201
ser = serial.Serial('/dev/cu.usbserial-14210', baudrate=115200) # Set buffer size to 2048 bytes
time.sleep(2)                                           # Time delay for Arduino Serial initialization 

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=1000, fargs=(yList,xList, ser), interval=10) 

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()                                             # Close Serial connection when plot is closed



