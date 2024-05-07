from machine import I2C,Pin,PWM
import time 

valve = PWM(Pin(11))
valve.freq(100000)

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)
pump = Pin(10,Pin.OUT)

Target_Pressure=180
P_end=30
T_end=40

max_pwm=40000
min_pwm=2000
sensor_address=0x28
range_max=400

Kp=350
Ki=50
Kd=0

last_err=0
next_err=0
err=0

H_co=0.2
L_co=0.4

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
    pressure=round(pressure*0.75,2)
    return pressure

def pump_up(Target_Pressure):# pump to target pressure
    current_pressure=read(sensor_address,0,range_max)
    pump_off()    
    valve_off()
    while current_pressure< Target_Pressure+10:
        pump_on()
        current_pressure=read(sensor_address,0,range_max)
        time.sleep(0.1)

def set_init(current_pressure):
    P_init=current_pressure
    start = time.ticks_ms()
    return P_init,start

def regulation(P_init,start,release_speed):
    delta = time.ticks_diff(time.ticks_ms(), start)/1000
    P_ref=calculate_ref(P_init,T_end,delta)
    current_pressure=read(sensor_address,0,range_max)

    
    u=PID_control(Kp,Ki,Kd,P_ref,current_pressure,delta)
    release_speed=release_speed+u
    release_speed=value_calibrate(release_speed)

    return delta,P_ref,current_pressure,release_speed

def sigle_pole_hpf(current_p,filtered_p,previous_p):
    
    filtered_p=H_co*(filtered_p + current_p - previous_p)
    previous_p=current_p
    return round(filtered_p,2)

def sigle_pole_lpf(unfiltered_p,filtered_p):
    filtered_p=L_co*unfiltered_p+(1-L_co)*filtered_p
    return round(filtered_p,2)

if __name__== '__main__':
    # pump_up(Target_Pressure)# state 1: pump up 
    # release_speed=max_pwm
    # init_set=0
    # current_pressure=read(sensor_address,0,range_max)
    # while current_pressure>P_end:
    #     if(init_set==0):
    #         P_init,start=set_init(current_pressure)
    #         init_set=1

    #     valve_on(int(release_speed))# state3: release
    #     P_ref,delta,current_pressure,release_speed=regulation(P_init,start,release_speed)# state 2: regulation
        
    #     pulse=current_pressure-P_ref
    #     print(delta,',',pulse)
    # release()
    current_pressure_fitered=0
    current_pressure_previous=0
    start = time.ticks_ms()
    while 1:
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        current_pressure=read(sensor_address,0,range_max)
        current_pressure_fitered=sigle_pole_lpf(current_pressure,current_pressure_fitered)
        # current_pressure_fitered=sigle_pole_hpf(current_pressure,current_pressure_fitered,current_pressure_previous)
        # current_pressure_previous=current_pressure
        # print("current_pressure: ",",",current_pressure,",","current_pressure_filtered: ",",",current_pressure_fitered)
        print(delta,",",current_pressure,",",current_pressure_fitered)