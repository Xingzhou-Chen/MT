from machine import I2C,Pin,PWM,UART
import time

# uart = UART(0, baudrate=512000, tx=Pin(0), rx=Pin(1))
# uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000_000)
valve_f = PWM(Pin(11))
valve_f.freq(100000)
valve_b = PWM(Pin(16))
valve_b.freq(100000)
valve_m = Pin(14,Pin.OUT)
pump = Pin(10,Pin.OUT)

Target_Pressure=160                                                                     # target pressure: 160 mmHg 
P_end=Target_Pressure-120                                                               # deflation range: 120 mmHg
T_end=40                                                                                # measurement time: 40s

Kp_f=1000                                                                               # PID gains of measurement system wit simulator
Ki_f=5
Kd_f=0 

# Kp_f=2600                                                                             # PID gains of measurement system with arm cuff
# Ki_f=4
# Kd_f=0

# Kp_f=5300                                                                             # PID gains of measurement system with finger cuff
# Ki_f=3
# Kd_f=0

Kp_b=10000                                                                              # PID gains of baseline system  
Ki_b=50
Kd_b=0

last_err_f=0
next_err_f=0
err_f=0
int_err_f=0

last_err_b=0
next_err_b=0
err_b=0
int_err_b=0

sensor_b=0x29                                                                           # address of baseline system sensor 
sensor_m=0x28                                                                           # address of differential sensor 
sensor_f=0x30                                                                           # address of measurement system sensor 

range_b_max=400                                                                         # range of baseline system sensor: 0 - 400 mbar
range_b_min=0

range_m_max=40                                                                          # range of differential sensor: 0 - 40 mbar
range_m_min=0

range_f_max=400                                                                         # range of measurement system sensor: 0 - 400 mbar
range_f_min=0

max_pwm_f=60000
min_pwm_f=1000

max_pwm_b=60000
min_pwm_b=2000

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

## AMS 6915 reading function
## modified based on the ams arduino library     
def read(address,pmin,pmax):                           
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=pressure*0.75
    return pressure

def release():
    valve_f.duty_u16(0)
    valve_b.duty_u16(0)
    midval_on()

## calculate the slope
## parameter: P_init: inital pressure 
## P_end: final release pressure 
## T_end: measurement time 
def calculate_k(P_init,P_end,T_end):                                
    k=(P_end-P_init)/T_end
    return k

def calculate_ref(k,P_init,current_t):                             
    P_ref_f=k*current_t+P_init
    return P_ref_f

## calculate the slope
## P_init: inital pressure of baseline volume 
## current_t: current time  
## return reference pressure of baseline volume  
def calculate_ref_b(k,P_init,current_t):
    P_ref_b=k*current_t+P_init
    return P_ref_b

def value_calibrate_f(v,t):
    if(v>max_pwm_f or v<min_pwm_f):return feed_forward_f(t)
    else: return v

def value_calibrate_b(v):
    if(v>max_pwm_b or v<min_pwm_b): return max_pwm_b
    else: return v

## PID Controller of measurement system    
## return control magnitude  
def PID_control_f(Kp,Ki,Kd,set_point,current_p):
    global err_f
    global last_err_f
    global int_err_f
    err_f=set_point-current_p
    int_err_f=err_f+int_err_f
    P=Kp*err_f                                                                          # P term 
    I=Ki*int_err_f                                                                      # I term 
    D=Kd*(err_f-last_err_f)                                                             # D term
    u=P+I+D
    last_err_f=err_f
    return u

## PID Controller of baseline system
def PID_control_b(Kp,Ki,Kd,set_point,current_p):
    global err_b
    global last_err_b
    global int_err_b
    err_b=set_point-current_p
    int_err_b=err_b+int_err_b
    P=Kp*err_b
    I=Ki*int_err_b
    D=Kd*(err_b-last_err_b)
    u=P+I+D
    last_err_b=err_b
    return u

def feed_forward_f(t):
    # ff=-2*t**2-182.3*t+18220                                                          # feedforward signal for arm
    # ff=35271.6-61.6*t                                                                 # feedforward signal when baudrate 512000
    ff=-187.7*t + 1.272e+04                                                             # feedforward signal for simulator
    # ff=-0.1673*t**3 + 5.495*t**2 - 349.1*t + 3.726e+04                                # feedforward signal for finger
    return ff

## feedforward signal for baseline volume 
def feed_forward_b(t):
    ff=-2.42*t**2 - 152.8*t + 3.531e+04
    return ff

## inflation funtion: inflate the cuff to target pressure and create the pressure difference  
## inflate in 4 stages:
## stage 1: direct pumping 
## stage 2: step-wise pumping 
## stage 3: release cuff gradually to target pressure
## stage 4: release baseline volume gradually to target pressure
def pump_up(Target_Pressure):
    frontvalve_off()
    backvalve_off()
    midval_on()
    current_pressure_f=read(sensor_f,range_f_min,range_f_max)
    while current_pressure_f< Target_Pressure-40:                                       # stage 1: direct pumping 
        pump_on()
        time.sleep(0.2)
        pump_off()
        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
    
    current_pressure_b=read(sensor_b,range_b_min,range_b_max)
    while current_pressure_f< Target_Pressure or current_pressure_b<Target_Pressure-20: # stage 2: step-wise pumping, make sure pressure in both volume is high enough 
        pump_on()
        time.sleep(0.2)
        pump_off()
        time.sleep(0.3)
        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
        current_pressure_b=read(sensor_b,range_b_min,range_b_max)
        if(current_pressure_b>Target_Pressure-10):midval_off()                          # close the middle valve when the pressure difference is too small 
    
    while current_pressure_f> Target_Pressure+1:                                        # stage 3: release cuff volume 
        frontvalve_on(25000)
        current_pressure_f=read(sensor_f,range_f_min,range_f_max) 
    frontvalve_off() 

    current_pressure_b=read(sensor_b,range_b_min,range_b_max)
    while current_pressure_b> Target_Pressure-19:                                       # stage 4: release baseline volume
        backvalve_on(25000)
        current_pressure_b=read(sensor_b,range_b_min,range_b_max) 
    backvalve_off() 
    pump_off()
    midval_off()

## record the initial pressure and starting time
## return P_init: initial pressure 
## return start: initial time 
def set_init(current_pressure):                                                        
    P_init=current_pressure
    start = time.ticks_ms()
    return P_init,start

## control algorithm 
## PID controller computes 
## feedforward signal 
## parameter delta: current time 
## parameter P_ref: reference pressure
## return release_speed: control magnitude
def regulation_f(delta,P_ref_f,current_pressure_f):
    ff_f=feed_forward_f(delta)
    u_f=PID_control_f(Kp_f,Ki_f,Kd_f,P_ref_f,current_pressure_f)
    release_speed_f=u_f+ff_f
    release_speed_f=value_calibrate_f(release_speed_f,delta)
    return release_speed_f

def regulation_b(delta,P_ref_b,current_pressure_b):
    ff_b=feed_forward_b(delta)
    u_b=PID_control_b(Kp_b,Ki_b,Kd_b,P_ref_b,current_pressure_b)
    release_speed_b=u_b+ff_b
    release_speed_b=value_calibrate_b(release_speed_b)
    return release_speed_b

if __name__== '__main__':   
    
    pump_up(Target_Pressure)                                                             # state 1 : pump up to target pressure
    time.sleep_ms(1000)
    current_pressure_f=read(sensor_f,range_f_min,range_f_max)
    current_pressure_b=read(sensor_b,range_b_min,range_b_max)  
    P_init_f,start=set_init(current_pressure_f)
    P_init_b=current_pressure_b                                                          # determine the initial state
    k=calculate_k(P_init_f,P_end,T_end)                                                  # calculate the slope of reference pressure 
    difference=current_pressure_f-current_pressure_b                                     # record the offset  
    while current_pressure_f>P_end:                                                      # deflation starts
        current_pressure_b=read(sensor_b,range_b_min,range_b_max)                        # state 2: regulation of baseline volume 
        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref_b=calculate_ref_b(k,P_init_b,delta)
        release_speed_b=regulation_b(delta,P_ref_b,current_pressure_b)  
        backvalve_on(int(release_speed_b))                                               # state 3: release baseline volume

        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref_f=current_pressure_b+difference                                            # reference of measurement system
        release_speed_f=regulation_f(delta,P_ref_f,current_pressure_f)                   # state 4: regulation of measurement system
        frontvalve_on(int(release_speed_f))                                              # state 5: release cuff
        
        current_pressure_m=read(sensor_m,range_m_min,range_m_max)                        # read the pressure difference as OWM
        print(delta,",",current_pressure_m-difference)                                   # remove the offset
    release()                                                                            # measurement ends