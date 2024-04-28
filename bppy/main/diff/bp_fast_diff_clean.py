from machine import I2C,Pin,PWM,UART
import time

# uart = UART(0, baudrate=512000, tx=Pin(0), rx=Pin(1))
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000000)
valve_f = PWM(Pin(15)).freq(100000)
valve_b = PWM(Pin(16)).freq(100000)
valve_m = Pin(14,Pin.OUT)
pump = Pin(13,Pin.OUT)

P_end=40
T_end=40
Target_Pressure=170

Kp_f=1700
Ki_f=10
Kd_f=0

Kp_b=3000
Ki_b=10
Kd_b=0

last_err_f=0
next_err_f=0
err_f=0
int_err_f=0

last_err_b=0
next_err_b=0
err_b=0
int_err_b=0

sensor_b=0x29
sensor_m=0x28
sensor_f=0x30

range_b_max=400
range_b_min=0

range_m_max=40
range_m_min=0

range_f_max=400
range_f_min=0

max_pwm_f=40000
min_pwm_f=2000

max_pwm_b=40000
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
    
def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure*0.75,2)
    return pressure

def release():
    valve_f.duty_u16(0)
    valve_b.duty_u16(0)
    midval_on()

def calculate_k(P_init,P_end,T_end):
    k=(P_end-P_init)/T_end
    return k

def calculate_ref(k,P_init,current_t):
    P_ref_f=k*current_t+P_init
    return P_ref_f

def calculate_ref_b(k,P_init,current_t):
    P_ref_b=k*current_t+P_init
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
    global int_err_f

    err_f=set_point-current_p
    int_err_f=err_f+int_err_f
    P=Kp*err_f
    I=Ki*int_err_f
    D=Kd*(err_f-last_err_f)
    u=P+I+D
    last_err_f=err_f
    return u

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
    # ff=30445.33-239.64*t+6.65*t**2-0.13*t**3
    ff=30483.3-143.6*t
    return ff

def feed_forward_b(t):
    ff=33952.8-53.7*t
    return ff

def pump_up(Target_Pressure):
    frontvalve_off()
    backvalve_off()
    midval_on()
    current_pressure_f=read(sensor_f,range_f_min,range_f_max)
    while current_pressure_f< Target_Pressure:
        pump_on()
        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
        current_pressure_b=read(sensor_b,range_b_min,range_b_max)
        if(current_pressure_b>150):midval_off()
        # print(current_pressure_f,current_pressure_b)
        time.sleep(0.1)
    
    pump_off()
    midval_off()

def set_init(current_pressure):
    P_init=current_pressure
    start = time.ticks_ms()
    return P_init,start

def regulation_f(delta,P_ref_f,current_pressure_f):
    ff_f=feed_forward_f(delta)
    u_f=PID_control_f(Kp_f,Ki_f,Kd_f,P_ref_f,current_pressure_f)
    release_speed_f=u_f+ff_f
    release_speed_f=value_calibrate_f(release_speed_f)
    return release_speed_f

def regulation_b(delta,P_ref_b,current_pressure_b):
    ff_b=feed_forward_b(delta)
    u_b=PID_control_b(Kp_b,Ki_b,Kd_b,P_ref_b,current_pressure_b)
    release_speed_b=u_b+ff_b
    release_speed_b=value_calibrate_b(release_speed_b)
    return release_speed_b

if __name__== '__main__':
    
    pump_up(Target_Pressure)                                            #state 1 : pump up to target pressure
    time.sleep_ms(1000)

    current_pressure_f=read(sensor_f,range_f_min,range_f_max)
    current_pressure_b=read(sensor_b,range_b_min,range_b_max)  
    P_init_f,start=set_init(current_pressure_f)
    P_init_b=current_pressure_b
    k=calculate_k(P_init_f,P_end,T_end)
    difference=current_pressure_f-current_pressure_b                    # determine the initial state

    while current_pressure_f>P_end:                                     # deflation 
        current_pressure_f=read(sensor_f,range_f_min,range_f_max)
        current_pressure_b=read(sensor_b,range_b_min,range_b_max)
        
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref_f=current_pressure_b+difference
        release_speed_f=regulation_f(delta,P_ref_f,current_pressure_f)  # state 2:regulation of Front
        frontvalve_on(int(release_speed_f))                             # state 3: release Front

        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref_b=calculate_ref_b(k,P_init_b,delta)
        release_speed_b=regulation_b(delta,P_ref_b,current_pressure_b)  #state 2: regulation of Back 
        backvalve_on(int(release_speed_b))                              #state 3: release Back 

        current_pressure_m=read(sensor_m,range_m_min,range_m_max)
        uart.write(str(delta)+" , "+str(current_pressure_m)+"\n")

    release()                                                           #release
