from machine import I2C,Pin,PWM
from time import sleep

i2c_hx = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=400000)
address=0x28

def read_adc_lin():
        data=i2c_hx.readfrom(address,4)
        pres=(data[1]<<16)+(data[2]<<8)+data[3]
        # pres=data[0]
        return pres

def read_adc_diff():
        data=i2c_hx.readfrom(address,4)
        adc=(data[0]<<16)+(data[1]<<8)+data[2]
        # pres=data[0]
        return adc

def read_diff():
    offset=0.00058
    adccount=read_adc_diff()
    v = adccount/(2**23)
    v = ((v - 1) * 3.2/8)# - offset # Spannung
    pressure = v * 10/(0.128-offset) # Pascal
    pressure=pressure*7.5
    pressure=round(pressure,2)
    return pressure

def read():
    adccount=read_adc_lin()
    pressure=(adccount-8518000)/5830.09
    pressure=round(pressure,2)
    return pressure
    
# i2c_hx.writeto(0x28,b'\xBC')
# i2c_hx.writeto(0x28,b'\xCD')
i2c_hx.writeto(0x28,b'\xBD')
i2c_hx.writeto(0x28,b'\xCE')
# i2c_hx.writeto(0x28,b'\xDA')
sleep(0.5)
while True:
    # adc=read_adc_diff()
    # pre=read_diff()
    # pre=read_adc_lin()
    pre=read()
#     adc=adc/64
#     adc=adc/16777216
#     volt=adc*3.3
#     volt=volt*1000
    
    print(pre)
    sleep(0.1)