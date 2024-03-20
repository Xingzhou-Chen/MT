from machine import I2C,Pin,PWM
from utime import sleep
import time

valve = PWM(Pin(16))
valve.freq(100000)
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
# pump = Pin(16,Pin.OUT)
sensor_address=0x28
range_max=400

def pump_on():
    pump.value(0)

def valve_on(pwm):
    valve.duty_u16(pwm)

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

if __name__== '__main__':
    current_pressure=read(sensor_address,0,range_max)
    while True:
        a=0
#         valve_on(65535)
        current_pressure=read(sensor_address,0,range_max)
        print(current_pressure)
        sleep(0.1)
#         while a<65535:
#             a=a+1000
# #             print(a)
#             valve_on(a)
#             sleep(0.5)
