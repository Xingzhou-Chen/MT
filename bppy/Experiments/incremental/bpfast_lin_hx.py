from machine import I2C,Pin,PWM
import time 

valve = PWM(Pin(11))
valve.freq(100000)

i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000_000)
pump = Pin(10,Pin.OUT)

Target_Pressure=160
P_end=40
T_end=40

max_pwm=40000
min_pwm=2000
sensor_address=0x28
range_max=400
range_min=0

Kp=350
Ki=50
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
    return round(P_ref,2)

def value_calibrate(v):
    if(v>max_pwm or v<min_pwm): return max_pwm
    else: return v
    
def read_hx():                   # Hx sensor read
    data=i2c.readfrom(0x28,4)
    adccount=(data[1]<<16)+(data[2]<<8)+data[3]
    raw=((adccount/2**23)/8-0.12)*3.3
    pressure=(adccount-8518000)/5830.09
    pressure=round(pressure,2)
    return raw                        


def pump_up(Target_Pressure):
    valve_off()
    current_pressure=read(sensor_address,range_min,range_max)
    while current_pressure< Target_Pressure-50:
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

def PID_control(Kp,Ki,Kd,set_point,current_p): # incremental PID
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

def regulation(P_ref,current_pressure,release_speed):
    u=PID_control(Kp,Ki,Kd,P_ref,current_pressure)
    release_speed=u+release_speed
    release_speed=value_calibrate(release_speed)
    return release_speed

if __name__== '__main__':
    pump_up(Target_Pressure)# state 1: pump up 
    release_speed=max_pwm
    init_set=0
    current_pressure=read_hx()
    P_init,start=set_init(current_pressure)

    # i2c.writeto(0x28,b'\xBD')
    # i2c.writeto(0x28,b'\xCE')

    while current_pressure>P_end:
        valve_on(int(release_speed))# state3: release
        P_ref=calculate_ref(P_init,T_end,delta)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        current_pressure=read_hx()
        release_speed=regulation(P_ref,current_pressure,release_speed)# state 2: regulation
        pulse=current_pressure-P_ref
        pulse=round(pulse,2)
        print(delta,',',pulse)
    release()