from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 



i2c = I2C(0, scl=Pin(9), sda=Pin(8),freq=400000)

sensor_address=0x28
range_max=400 

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

def read0(address):
    data=i2c.readfrom(address,4)
    pres=(data[1]<<16)+(data[2]<<8)+data[3]
    return pres

def read_druck():
    adccount=read0(0x28)
    druck=(adccount-8620000)/5318
    druck=round(druck,2)
    return druck

if __name__== '__main__':
#     current_pressure=read(sensor_address,0,range_max)
    time.sleep_ms(1000)
    file_filtered=open("espconstant_pressure.csv","w")
    t=0
    i2c.writeto(0x28,'0xCE')
    start = time.ticks_ms()
    while t<500:
        current_pressure=read(sensor_address,0,range_max)
        print(current_pressure)
#         current_pressure=read_druck()
        print(current_pressure)
        t=t+1
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        file_filtered.write(str(delta)+","+str(current_pressure)+"\n")
        file_filtered.flush()
#     release()

    


