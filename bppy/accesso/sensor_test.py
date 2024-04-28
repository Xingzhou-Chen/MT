from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 


# valve = PWM(Pin(9))
valve = PWM(Pin(11))
valve.freq(100000)

i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
sensor_address=0x30
range_max=400

def valve_on(release_speed):
    valve.duty_u16(release_speed)
    
def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

if __name__== '__main__':
    valve_on(65535)
    while True:
        current_pressure=read(sensor_address,0,range_max)
        print(current_pressure)
        sleep(0.1)

    

