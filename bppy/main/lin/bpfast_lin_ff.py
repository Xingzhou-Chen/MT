from machine import I2C,Pin,PWM,UART
# import machine 
from utime import sleep
import time 

H_co=0.4
L_co=0.9

# valve = PWM(Pin(9))
valve = PWM(Pin(11))
valve.freq(100000)

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
# uart = UART(0, baudrate=512000, tx=Pin(0), rx=Pin(1))
# i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000000)
pump = Pin(10,Pin.OUT)
max_pwm=35000
min_pwm=10000
Target_Pressure=150
P_end=50
T_end=40
sensor_address=0x28
range_max=400
Kp=2000
Ki=10
# Kd=0.015
Kd=1000
last_err=0
next_err=0
err=0
int_err=0
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
    
# def PID_control(Kp,Ki,Kd,set_point,current_p,current_t):
#     global err
#     global last_err
#     global next_err
#     err=set_point-current_p
#     P=Kp*(err-last_err)
#     I=Ki*err
#     D=Kd*(err+next_err-2*last_err)
#     u=P+I+D
#     next_err=last_err
#     last_err=err
#     return u

def PID_control(Kp,Ki,Kd,set_point,current_p,current_t):
    global err
    global last_err
    global int_err
    int_set=0
    # if current_t>30 and int_set==0:
    #     # int_err=0
    #     Kp=8000
    #     Ki=2
    #     int_set=1
    err=set_point-current_p
    int_err=err+int_err
    P=Kp*err
    I=Ki*int_err
    D=Kd*(err-last_err)
    u=P+I+D
    last_err=err
    # print(current_t,",",current_t-prev_t)
    return u

# def feed_forward(t):
#     ff=30209-181*t
#     return ff

# def feed_forward(t):
#     if t<=14:
#         ff=29648-70*t
#     elif t>=14 and t<20:
#         ff=34276-404*t
#     elif t>=20 and t<=36.5:
#         ff=27900-84*t
#     else :
#         ff=42816-505*t
#     return ff

def feed_forward(t):
    # ff=-0.02435*t**4 + 2.059*t**3 - 57.24*t**2 + 409.5*t + 2.877e+04
    ff=-0.001342*t**5 + 0.12*t**4 - 3.535*t**3 + 36.21*t**2 - 195.2*t + 2.964e+04
    # ff=-0.0006677*t**4 + 0.3283*t**3 - 15.99*t**2 + 73.78*t + 2.937e+04
    return ff

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

def read_adc():
        data=i2c.readfrom(0x28,4)
        pres=(data[1]<<16)+(data[2]<<8)+data[3]
#         pres=data[0]
        return pres

def read_hx():
    adccount=read_adc()
    pressure=(adccount-8518000)/5830.09
    pressure=round(pressure,2)
    return pressure

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
#     current_pressure=read(sensor_address,0,range_max)
    current_pressure=read_hx()
    pump_off()
    P_init=0
    valve_off()
    i2c.writeto(0x28,b'\xBD')
    i2c.writeto(0x28,b'\xCE')
    while current_pressure< Target_Pressure:
        pump_on()
#         current_pressure=read(sensor_address,0,range_max)
        current_pressure=read_hx()
        print(current_pressure)
        sleep(0.1)
    
    pump_off()
    time.sleep_ms(1000)
    init_set=0
    while current_pressure>P_end:
        current_pressure=read_hx()
        if(init_set==0):
            P_init=current_pressure
            f=current_pressure
            start = time.ticks_ms()
            init_set=1
            
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(P_init,T_end,delta)
        
        Kf=feed_forward(delta)
        release_speed=PID_control(Kp,Ki,Kd,P_ref,current_pressure,delta)+Kf
        release_speed=value_calibrate(release_speed)

        valve_on(int(release_speed))
        # print(delta,',',release_speed)
        # print(delta,',',int_err,',',err)
        # print(delta,current_pressure,P_ref)
        pulse=current_pressure-P_ref+10
        uart.write(str(delta)+" , "+str(pulse)+"\n")
        # print(delta,',',current_pressure,',',P_ref,',',pulse)
        # print(delta,',',pulse)
        # print(delta,',',current_pressure,',',P_ref)
        # print(delta,',',pulse)
    release()


    


