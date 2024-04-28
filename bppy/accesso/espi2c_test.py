from machine import I2C,Pin,PWM
from utime import sleep
Led = Pin(7,Pin.OUT)


i2c = I2C(0, scl=Pin(9), sda=Pin(8),freq=400000)

if __name__== '__main__':
#     i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
    print(hex(i2c.scan()[0]))