from machine import I2C,Pin
from utime import sleep 

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
 
def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = ((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin
    return pressure 
    
# Get the address of all devices under the I2C bus
while True:
    pre=read(0x28,0,350)
    pre=round(pre,2)
    print("{} mbar".format(pre))
    sleep(0.1)