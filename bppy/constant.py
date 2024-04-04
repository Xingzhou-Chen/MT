from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 

H_co=0.4
L_co=0.9

# valve = PWM(Pin(9))
valve = PWM(Pin(11))
valve.freq(100000)

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)
# i2c0 = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
# i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
pump = Pin(10,Pin.OUT)
# valve = machine.Pin(27,machine.Pin.OUT)
max_pwm=30000
min_pwm=2000
Target_Pressure=130
read_delay=10
P_end=20
T_end=30
sensor_address=0x28
range_max=400
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
    current_pressure=read(sensor_address,0,range_max)
#     current_pressure=read_druck()
    pump_off()
    P_init=0
    release_speed=max_pwm
    valve_off()
    while current_pressure< Target_Pressure:
        pump_on()
        current_pressure=read(sensor_address,0,range_max)
#         current_pressure=read_druck()
#         print(current_pressure)
        sleep(0.1)
    
    pump_off()
    time.sleep_ms(1000)
#     file_filtered=open("constant_pressure.csv","w")
    t=0
    start = time.ticks_ms()
#     while True:
#         current_pressure=read(sensor_address,0,range_max)
#         print(current_pressure)
    while t<10000:
#         print(int(release_speed))
        valve_off()
#         valve_on(max_pwm)
        current_pressure=read(sensor_address,0,range_max)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        print(delta,",",current_pressure)
#         file_filtered.write(str(delta)+","+str(current_pressure)+"\n")
#         file_filtered.flush()
        t=t+1
# #         print(t)
        sleep(0.05)
#         time.sleep_us(1000)
#     release()

    

