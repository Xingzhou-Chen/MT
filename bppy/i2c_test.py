from machine import I2C,Pin,PWM
from utime import sleep
import time 

i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
print(i2c.scan())
print(hex(i2c.scan()[0]))
# print(hex(i2c.scan()[1]))
# print(hex(i2c.scan()[2]))

    
    