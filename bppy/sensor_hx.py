from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 



i2c0 = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
i2c1 = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
sensor_address=0x28
range_max=400*0.75

def read_1(address,pmin,pmax):
    data=i2c1.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75
    
def read(address):
    data=i2c0.readfrom(address,7)
#     sleep(0.01)
    pres=(data[1]<<16)+(data[2]<<8)+data[3]
#     pres=(pres-8608000)/ 5331.12
    return pres

def read_voltage():
    adccount=read(sensor_address)
    relative=(adccount)/8388608.0
    voltage=2.0*(relative-1)*0.813/8                                                                                                                                   /8
#     relative= adccount/16777216.0
#     voltage=relative * 0.813 / 8
    return voltage
    
def read_druck1():
    vol=read_voltage()
    druck=(vol/0.06)*3.5*750
    druck=round(druck,2)
    return druck


def read_druck():
    adccount=read(sensor_address)
    druck=(adccount-8620000)/5318
    druck=round(druck,2)
    return druck
    
if __name__== '__main__':
#     print(i2c.scan())
    i2c0.writeto(0x28, b'\xBD')
    i2c0.writeto(0x28, b'\xCE')
    while True:
#         current_pressure=read(sensor_address)
#         print(current_pressure)
#         print(read(sensor_address))
#         print(read_druck())
        print(read_voltage())
#         print(read_druck())
#         print(read_druck()-read_1(sensor_address,0,range_max))
#         print('hx: ',read_druck(),'AMS: ',read_1(sensor_address,0,range_max))
        sleep(1)

    


