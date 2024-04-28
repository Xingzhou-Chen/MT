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
pump = Pin(10,Pin.OUT)
max_pwm=40000
min_pwm=2000
Target_Pressure=180
read_delay=10
P_end=30
T_end=40
sensor_address=0x28
range_max=400
Kp=350
Ki=50
# Kd=0.015
Kd=0
last_err=0
next_err=0
err=0
peak=-20
mp=0
def pump_on():
    pump.value(1)

def pump_off():
    pump.value(0)
    
def valve_off():
    valve.duty_u16(max_pwm)

def valve_on(release_speed):
    valve.duty_u16(release_speed)

def release():
    valve.duty_u16(0)

def calculate_ref(P_init,T_end,current_t):
    k=(P_end-P_init)/T_end
    P_ref=k*current_t+P_init
    return P_ref

def value_calibrate(v):
    if(v>max_pwm or v<min_pwm): return max_pwm
    else: return v
    
def PID_control(Kp,Ki,Kd,set_point,current_p,current_t):
    global err
    global last_err
    global next_err
    err=set_point-current_p
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
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

def sigle_pole_hpf(current_p,filtered_p,previous_p):

    filtered_p=H_co*(filtered_p + current_p - previous_p)
    previous_p=current_p
    return filtered_p

def sigle_pole_lpf(unfiltered_p,filtered_p):
    return L_co*unfiltered_p+(1-L_co)*filtered_p


def peak_hb(current_p,filtered_p):
#     print(current_p,filtered_p)
    global peak
    global mp
    if(filtered_p>peak and current_p<300):
        peak=filtered_p
        mp=current_p
    return mp
   

if __name__== '__main__':
    current_pressure=read(sensor_address,0,range_max)
    pump_off()
    P_init=0
    release_speed=max_pwm
    valve_off()
    while current_pressure< Target_Pressure:
        pump_on()
        current_pressure=read(sensor_address,0,range_max)
#         current_pressure=read_druck()
        # print(current_pressure)
        sleep(0.1)
    
    pump_off()
    time.sleep_ms(1000)
    init_set=0
    while current_pressure>P_end:
#         print(int(release_speed))
        valve_on(int(release_speed))
#         valve_on(max_pwm)
#         valve_on(15000)
#         sleep(0.1)
        current_pressure=read(sensor_address,0,range_max)
        if(init_set==0):
            P_init=current_pressure
            f=current_pressure
            start = time.ticks_ms()
            init_set=1
            
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(P_init,T_end,delta)
        
        u=PID_control(Kp,Ki,Kd,P_ref,current_pressure,delta)
        release_speed=release_speed+u
        release_speed=value_calibrate(release_speed)

        # print(delta,current_pressure,P_ref)
        pulse=current_pressure-P_ref
        # print(delta,',',current_pressure,',',P_ref,',',pulse)
        print(delta,',',pulse)
    release()


    
