from machine import I2C,Pin,PWM,UART
from utime import sleep
import time

valve_f = PWM(Pin(15))
valve_f.freq(100000)

valve_b = PWM(Pin(16))
valve_b.freq(100000)

valve_m = Pin(14,Pin.OUT)
pump = Pin(13,Pin.OUT)

max_pwm_f=40000
min_pwm_f=2000

max_pwm_b=40000
min_pwm_b=2000

Target_Pressure=200


P_end=40
T_end=40

Kp_f=900
Ki_f=20
Kd_f=0

Kp_b=3500
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

# k=0


i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000000)
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
    pressure=round(pressure,2)
    return pressure*0.75

def release():
    valve_f.duty_u16(0)
    valve_b.duty_u16(0)
    midval_on()

def calculate_k(P_init,T_end):
    k=(P_end-P_init)/T_end
    return k

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
    # ff=29110.60-12.17*t+5.15*t**2-0.11*t**3
    ff=29304.01-36.38*t-1.58*t**2-0.00*t**3
    return ff

def feed_forward_b(t):
    # ff=34628.3-51.1*t
    ff=34012.94-50.25*t+0.33*t**2-0.01*t**3
    return ff
    
if __name__== '__main__':
    while True:
        frontvalve_off()
        # midval_off()
        backvalve_off()