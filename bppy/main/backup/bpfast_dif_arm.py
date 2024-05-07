from machine import I2C,Pin,PWM,UART
from utime import sleep
import time

H_co=0.9
L_co=0.5

# uart = UART(0, baudrate=512000, tx=Pin(0), rx=Pin(1))
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

valve_f = PWM(Pin(15))
valve_f.freq(100000)

valve_b = PWM(Pin(16))
valve_b.freq(100000)

valve_m = Pin(14,Pin.OUT)
pump = Pin(13,Pin.OUT)

max_pwm_f=40000
min_pwm_f=2000
init_pwm=32000

max_pwm_b=35000
min_pwm_b=30000

Target_Pressure=170


P_end=40
T_end=40

Kp_f=1000
Ki_f=1
Kd_f=0

Kp_b=1600
Ki_b=10
Kd_b=0

last_err_f=0
next_err_f=0
err_f=0

last_err_b=0
next_err_b=0
err_b=0


k=0


i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000000)
# pump = Pin(16,Pin.OUT)
sensor_b=0x29
sensor_m=0x28
sensor_f=0x30
# sensor_m=0x28
# sensor_f=0x28
# sensor_b=0x28

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

def midval_off():
    valve_m.value(0)

def midval_on():
    valve_m.value(1)
    
def frontvalve_on(pwm):
    valve_f.duty_u16(pwm)

def frontvalve_off():
    valve_f.duty_u16(65535)
    
def backvalve_on(pwm):
    valve_b.duty_u16(pwm)

def backvalve_off():
    valve_b.duty_u16(65535)
#     valve_b.duty_u16(0)
    
def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

def release():
    valve_f.duty_u16(0)
    valve_b.duty_u16(0)
    midval_on()

 
def calculate_ref(P_init,T_end,current_t):
    global k
    k=(P_end-P_init)/T_end
    P_ref=k*current_t+P_init
    return P_ref

def calculate_ref_b(k,P_init,current_t):
#     print(k,P_init,current_t)
    P_ref_b=k*current_t+P_init
#     print(P_ref_b)
    return P_ref_b


def value_calibrate_f(v):
    if(v>max_pwm_f or v<min_pwm_f): return max_pwm_f
    else: return v

def value_calibrate_b(v):
    if(v>max_pwm_b or v<min_pwm_b): return max_pwm_b
    else: return v
    
def PID_control_f(Kp,Ki,Kd,set_point,current_p):
    global err_f
    global last_err_f
    global next_err_f
    err_f=set_point-current_p
    P=Kp*(err_f-last_err_f)
    I=Ki*err_f
    D=Kd*(err_f+next_err_f-2*last_err_f)
    u=P+I+D
    next_err_f=last_err_f
    last_err_f=err_f
    return u

def PID_control_b(Kp,Ki,Kd,set_point,current_p):
    global err_b
    global last_err_b
    global next_err_b
   
    err_b=set_point-current_p
    P=Kp*(err_b-last_err_b)
    I=Ki*err_b
    D=Kd*(err_b+next_err_b-2*last_err_b)
    u=P+I+D
    next_err_b=last_err_b
    last_err_b=err_b
    return u

def sigle_pole_hpf(current_p,filtered_p,previous_p):
    
    filtered_p=H_co*(filtered_p + current_p - previous_p)
    previous_p=current_p
    return filtered_p

def sigle_pole_lpf(unfiltered_p,filtered_p):
    return L_co*unfiltered_p+(1-L_co)*filtered_p


def feed_forward(t):
    # print(t)
    ff=33075-10000*t
        # print(t,ff)
    return round(ff,0)
    
if __name__== '__main__':
    current_pressure_f=read(sensor_f,range_f_min,range_f_max)
    pump_off()
    P_init=0
    release_speed_f=init_pwm
    release_speed_b=32800
    frontvalve_off()
    backvalve_off()
#     release()
    while current_pressure_f< Target_Pressure:
        pump_on()
        midval_on()
        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
        current_pressure_b=read(sensor_b,range_b_min,range_b_max)
#         current_pressure_m=read(sensor_m,range_m_min,range_m_max)
        # print(current_pressure_f,current_pressure_b)
        sleep(0.1)
    
    pump_off()
    midval_off()
    
    current_pressure_f=read(sensor_f,range_f_min,range_f_max)
    current_pressure_b=read(sensor_b,range_b_min,range_b_max)
    current_pressure_m=read(sensor_m,range_m_min,range_m_max)
    # print(current_pressure_f,current_pressure_b)
    time.sleep_ms(1000)
    
#     while current_pressure_f< current_pressure_b+20:
    while current_pressure_m< 20:
        pump_on()
#         current_pressure_f=read(sensor_f,range_f_min,range_f_max)
#         current_pressure_b=read(sensor_b,range_b_min,range_b_max)
        current_pressure_m=read(sensor_m,range_m_min,range_m_max)
#         print(current_pressure_f,current_pressure_b)
#         print(current_pressure_m)
        sleep(0.1)
        
    pump_off()    
    time.sleep_ms(1000)
#     current_pressure_m=read(sensor_m,range_m_min,range_m_max)
    init_set=0

    frontvalve_off()
    backvalve_off()
    count=0   
    previous_p=0 
    hpf_filtered_p=0
    lpf_filtered_p=0
    while current_pressure_f>P_end:
#         print(int(release_speed))
        frontvalve_on(int(release_speed_f))
        backvalve_on(int(release_speed_b))
        # backvalve_on(max_pwm_b)
#         frontvalve_on(max_pwm_f)
        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
        current_pressure_b=read(sensor_b,range_b_min,range_b_max)
        current_pressure_m=read(sensor_m,range_m_min,range_m_max)
        
        if(init_set==0):
            P_init=current_pressure_f
            P_init_b=current_pressure_b
            start = time.ticks_ms()
            init_set=1
            
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(P_init,T_end,delta)
#         print(k,P_init_b)
        P_ref_b=calculate_ref_b(k,P_init_b,delta)

        u_f=PID_control_f(Kp_f,Ki_f,Kd_f,P_ref,current_pressure_f)
        release_speed_f=release_speed_f+u_f
        release_speed_f=value_calibrate_f(release_speed_f)
# #         

        # if delta<3:
        #     release_speed_b=33075-300*delta
        # else:
        u_b=PID_control_b(Kp_b,Ki_b,Kd_b,P_ref_b,current_pressure_b)
        release_speed_b=release_speed_b+u_b
        # release_speed_b=feed_forward(delta)+release_speed_b
        release_speed_b=value_calibrate_b(release_speed_b)
      
        
        # print(delta,",",release_speed_b)
        # if delta>3 and delta<5:
        #     release_speed_b=33075-200*delta
        # else :
        #     release_speed_b=33075-50*delta

        # release_speed_b=release_speed_b-100
        # print(delta,",",feed_forward(delta))
        # count=count+1
        # print(delta,",",current_pressure_f,",",current_pressure_b,",",current_pressure_m)
        # print(delta,",",current_pressure_f,",",P_ref)
        # print(delta,",",current_pressure_b,",",P_ref_b)
        # print(delta,",",current_pressure_m)
        # if(delta>20 and delta<30):
        # print(delta,",",err_b,",",release_speed_b)
        print(delta,",",release_speed_b)
        # pulse_f=current_pressure_f-P_ref
        # hpf_filtered_p=sigle_pole_hpf(current_pressure_m,hpf_filtered_p,previous_p)
        # previous_p=current_pressure_m
        # lpf_filtered_p=sigle_pole_lpf(current_pressure_m,lpf_filtered_p)
        pulse_b=current_pressure_b-P_ref_b+20
        # uart.write(str(delta)+" , "+str(current_pressure_f)+" , "+str(current_pressure_b)+" , "+str(current_pressure_m)+"\n")
        uart.write(str(delta)+" , "+str(pulse_b)+"\n")
        # print(delta,",",lpf_filtered_p)
        # uart.write(str(delta)+" , "+str(lpf_filtered_p)+"\n")
        # print(delta,",",current_pressure_f,",",P_ref,",",current_pressure_b,",",P_ref_b,",",current_pressure_m)
    release()
    # samplingrate=count/delta
    # print("max_sampling_rate(difference):Sampling time(s):",delta,"Samples:", count,"Sampling rate: " ,samplingrate )