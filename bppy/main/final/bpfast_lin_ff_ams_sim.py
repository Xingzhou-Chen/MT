from machine import I2C,Pin,PWM,UART
import time

# uart = UART(0, baudrate=512000, tx=Pin(0), rx=Pin(1))
# uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000_000)
valve = PWM(Pin(11))
valve.freq(100000)
pump = Pin(10,Pin.OUT)

P_end=30
T_end=40
Target_Pressure=160

Kp=3200
Ki=35
Kd=0

last_err=0
next_err=0
err=0
int_err=0

sensor_address=0x28

range_max=400
range_min=0

max_pwm=35000
min_pwm=2000

def pump_on():
    pump.value(1)
    
def pump_off():
    pump.value(0)
    
def valve_on(pwm):
    valve.duty_u16(pwm)

def valve_off():
    valve.duty_u16(65535)
    
def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=pressure*0.75
    return pressure

def release():
    valve.duty_u16(0)

def calculate_k(P_init,P_end,T_end):
    k=(P_end-P_init)/T_end
    return k

def calculate_ref(k,P_init,current_t):
    P_ref=k*current_t+P_init
    return P_ref

def value_calibrate(v,t):
    if(v>max_pwm or v<min_pwm): return feedorward(t)
    else: return v

    
def PID_control(Kp,Ki,Kd,set_point,current_p):
    global err
    global last_err
    global int_err
    err=set_point-current_p
    int_err=err+int_err
    P=Kp*err
    I=Ki*int_err
    D=Kd*(err-last_err)
    u=P+I+D
    last_err=err
    return u

def feedorward(t):
    # ff=29304.01-36.38*t-1.58*t**2-0.00*t**3
    # ff=12639.8-214.1*t
    ff=12770.6-216*t
    return ff

def pump_up(Target_Pressure):
    valve_off()
    current_pressure=read(sensor_address,range_min,range_max)
    while current_pressure< Target_Pressure-40:
        pump_on()
        time.sleep(0.2)
        pump_off()
        current_pressure=read(sensor_address,range_min,range_max)

    while current_pressure< Target_Pressure:
        pump_on()
        time.sleep(0.2)
        pump_off()
        time.sleep(0.3)
        current_pressure=read(sensor_address,range_min,range_max)
    
    while current_pressure> Target_Pressure+1: # release f
        valve_on(29000)
        current_pressure=read(sensor_address,range_min,range_max) 
    
    valve_off() 
    pump_off()

def set_init(current_pressure):
    P_init=current_pressure
    start = time.ticks_ms()
    return P_init,start


def regulation(delta,P_ref,current_pressure):
    ff=feedorward(delta)
    u=PID_control(Kp,Ki,Kd,P_ref,current_pressure)
    release_speed=u+ff
    release_speed=value_calibrate(release_speed,delta)
    return release_speed

def single_pole_lpf(unfiltered_p,filtered_p,L_co):
    f=L_co*unfiltered_p+(1-L_co)*filtered_p
    return round(f,2)


if __name__== '__main__':   
    pump_up(Target_Pressure)                                            #state 1 : pump up to target pressure
    time.sleep_ms(1000)
    current_pressure=read(sensor_address,range_min,range_max) 
    current_pressure_filtered=current_pressure
    P_init,start=set_init(current_pressure)
    k=calculate_k(P_init,P_end,T_end)                   

    while current_pressure>P_end:                                     # deflation 
        current_pressure=read(sensor_address,range_min,range_max)
        # current_pressure_filtered=single_pole_lpf(current_pressure,current_pressure_filtered,0.5)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(k,P_init,delta)
        release_speed=regulation(delta,P_ref,current_pressure)  # state 2:regulation     
        valve_on(int(release_speed))                             # state 3: release
        pulse=current_pressure-P_ref
        pulse=round(pulse,2)
        print(delta,',',pulse,",",current_pressure,",",P_ref)
        # print(delta,",",release_speed)

    release()                                                           #release
