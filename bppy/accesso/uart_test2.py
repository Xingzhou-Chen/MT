import time
from machine import Pin, I2C,UART

i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=pressure*0.75
    pressure=round(pressure,2)
    return pressure

start = time.ticks_ms()
delta = time.ticks_diff(time.ticks_ms(), start)/1000
while delta<1000:
#     uart.write("Hello, World!\n")
    current_pressure=read(0x28,0,400)
    delta = time.ticks_diff(time.ticks_ms(), start)/1000
#     time.sleep(0.1)
    uart.write(str(delta)+" , "+str(current_pressure)+"\n")
#     print(delta,',',current_pressure)


