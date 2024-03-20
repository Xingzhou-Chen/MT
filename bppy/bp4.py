from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 

valve = PWM(Pin(11))
valve.freq(10000)

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
pump = Pin(10,Pin.OUT)
# valve = machine.Pin(27,machine.Pin.OUT)

Target_Pressure=300
read_delay=1000
release_speed=65535
P_end=50
T_end=45
sensor_address=0x28
range_max=400
Kp=0.4
Ki=0.05
# Kd=0.015
Kd=0
last_err=0
next_err=0
err=0

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

def calculate_ref(P_init,T_end,current_t):
    k=(P_end-P_init)/T_end
    P_ref=k*current_t+P_init
    return P_ref

def value_calibrate(v):
    if(v>65535): return v
    else: return v
    
def PID_controll(Kp,Ki,Kd,set_point,current_p,current_t):
    global err
    global last_err
    global next_err
    
    err=current_p-set_point
    P=Kp*(err-last_err)
    I=Ki*err
    D=Kd*(err+next_err-2*last_err)
    u=P+I+D
    next_err=last_err
    last_err=err
    return u

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = ((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin
    pressure=round(pressure,2)
    return pressure

if __name__== '__main__':
    current_pressure=read(sensor_address,0,range_max)
    pump_off()
    time_release=0
    P_init=0
    
    valve_off()
    file=open("add.csv","w")
    while current_pressure< Target_Pressure:
        pump_on()
        current_pressure=read(sensor_address,0,range_max)
        print(current_pressure)
        sleep(0.1)
        
    pump_off()
    start = time.ticks_ms()
    P_init=current_pressure
    while current_pressure>P_end:
        valve_on(int(release_speed))
        current_pressure=read(sensor_address,0,range_max)
        file.flush()
#         sleep(read_delay)
        time.sleep_us(read_delay)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
#         print(delta)
        P_ref=calculate_ref(P_init,T_end,delta)
        file.write(str(delta)+","+str(current_pressure)+","+str(P_ref)+"\n")# data is written as a string in the C
#         file.write(str(delta)+","+str(current_pressure)+"\n")# data is written as a string in the C
        u=PID_controll(Kp,Ki,Kd,P_ref,current_pressure,delta)
        release_speed=release_speed-20000*u
        release_speed=value_calibrate(release_speed)
        print(0,current_pressure,P_ref,400)
    release()
    print(calculate_ref(P_init,T_end,T_end))


