from machine import I2C,Pin,PWM
from utime import sleep
Led = Pin(7,Pin.OUT)


i2c = I2C(0, scl=Pin(9), sda=Pin(8),freq=400000)
# led=Pin(10,Pin.OUT)     

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

if __name__== '__main__':
#     i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
    print(i2c.scan())
#     file_filtered=open("esp32test_pressure.csv","w")
#     while True:
#         print(read(0x28,0,400))
#         file_filtered.write("testing"+"\n")
#         file_filtered.flush()
#     while True:
#         Led.on()
#         sleep(1)
#         sleep(1)
    
