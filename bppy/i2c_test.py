from machine import I2C,Pin,PWM
from utime import sleep
import time 

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
print(hex(i2c.scan()[0]))
    
    