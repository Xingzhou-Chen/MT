from machine import I2C,Pin,PWM
from utime import sleep
import time 

i2c_hx = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
# i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
# print(i2c.scan())

print(i2c_hx.scan())
# print(hex(i2c.scan()[0]))
# print(hex(i2c.scan()[1]))
# print(hex(i2c.scan()[2]))

    
    