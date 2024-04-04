from machine import I2C,Pin,PWM
from utime import sleep
import time 

status=0
pressureH=0
pressureM=0
pressureL=0

tempratureH=0
tempratureM=0
tempratureL=0

MAX_PRESURE=41000 #<Maximum value of pressure range(Pa)
MIN_PRESURE=41000 #<Minimum value of pressure range(Pa)

MAX_TEMP=85.0 #<Highest value of temperature range(℃)
MIN_TEMP=40.0

pressure=0
temprature=0

i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
# i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
# print(i2c.scan())
def config():
    config=[b'\xaa',b'\x00',b'\x80']
    i2c.writeto(0x00,config[0])
    i2c.writeto(0x00,config[1])
    i2c.writeto(0x00,config[2])
    sleep(0.03)

def get():
    global status
    global pressureH
    global pressureM
    global pressureL

    global tempratureH
    global tempratureM
    global tempratureL

    global pressure
    global temperature
    config()
    data=i2c.readfrom(0x00,7)
    status=data[0]
    pressureH=data[1]
    pressureM=data[2]
    pressureL=data[3]

    tempratureH=data[4]
    tempratureM=data[5]
    tempratureL=data[6]
    pressureData = (pressureH<<8)+(pressureM)
    pressureData=pressureData>>2

    pressure=(pressureData/16384.0)*(MAX_PRESURE+MIN_PRESURE)-MIN_PRESURE
    pressure=pressure*0.0075-0.052
    tempData = 0 
    tempData = (tempratureH<<8)+(tempratureM)
    temperature=(tempData/65536.0)*(MAX_TEMP+MIN_TEMP)-MIN_TEMP

if __name__== '__main__':
    config()
    while True:
        get()
        print("pressure: ",pressure)
        print("temprature: ", temperature)
        sleep(0.5)