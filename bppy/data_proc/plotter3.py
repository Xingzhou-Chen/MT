import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
port='/dev/tty.usbmodem14401'
baud_rate=115200

ser=serial.Serial(port,baud_rate)

x_vals=[]
y_vals=[]

def read_an_proc():
    line=ser.readline().decode('utf-8').strip()
    values=line.split('T')
    # digit_list = [''.join(filter(str.isdigit, string)) for string in values]
    print(values)
read_an_proc()
#     x_vals.append(float(values[0]))
#     y_vals.append(float(values[1]))

#     print(f'Time:{values[0]},X:{values[0]},Y:{values[1]}')

# def update_plot(frame):
#     read_an_proc()
#     plt.cla()

#     plt.plot(x_vals,x_vals,label='N01')
#     plt.plot(x_vals,y_vals,label='N02')
#     plt.xlabel('Time')
#     plt.ylabel('Vals')
#     plt.legend()

# fig,ax=plt.subplots()

# ani =FuncAnimation(fig,update_plot,1000,interval=1)
# plt.show()


