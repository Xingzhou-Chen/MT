from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 



i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=400000)
# i2c0 = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
# i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
pump = Pin(13,Pin.OUT)
# valve = machine.Pin(27,machine.Pin.OUT)
max_pwm=30000
min_pwm=2000
Target_Pressure=4
read_delay=10
P_end=20
T_end=30

sensor_b=0x29
sensor_m=0x28
sensor_f=0x30

range_b_max=400
range_b_min=0

range_m_max=40
range_m_min=0

range_f_max=400
range_f_min=0

def pump_on():
    pump.value(1)

def pump_off():
    pump.value(0)
    
def valve_off():
    valve.duty_u16(65535)

def valve_on(release_speed):
    valve.duty_u16(release_speed)

def release():
    valve.duty_u16(0)
    

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75


if __name__== '__main__':
    current_pressure=read(sensor_f,range_f_min,range_f_max)
#     current_pressure=read_druck()
    pump_off()
    P_init=0
    release_speed=max_pwm
    while current_pressure< Target_Pressure:
        pump_on()
        current_pressure=read(sensor_f,range_f_min,range_f_max)
#         current_pressure=read_druck()
        print(current_pressure)
        sleep(0.1)
    
    pump_off()
#     time.sleep_ms(1000)
#     file_filtered=open("constant_pressure.csv","w")
#     t=0
    
    start = time.ticks_ms()
    while True:
        current_pressure=read(sensor_f,range_f_min,range_f_max)
        print(current_pressure)
        sleep(0.5)

    


