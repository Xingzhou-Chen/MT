from machine import I2C,Pin,PWM
from utime import sleep
import time 
import lwlp
import ams


i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
# i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
# print(i2c.scan())
lwlp=lwlp.LWLP(i2c,0x00,41000,-41000)
ams=ams.AMS(i2c,0x28,0,300)
if __name__== '__main__':
    while True:
        pressure=lwlp.read()
        # pressure=ams.read()
        print("pressure: ",pressure)
        sleep(0.5)