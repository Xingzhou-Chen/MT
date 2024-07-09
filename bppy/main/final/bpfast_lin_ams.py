from machine import I2C,Pin,PWM
import time 

valve = PWM(Pin(11))
valve.freq(100000)

# i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000_000)
pump = Pin(10,Pin.OUT)

Target_Pressure=160 # finger and arm 160 sim is 150
P_end=Target_Pressure-120
T_end=40


valve_m = Pin(14,Pin.OUT)
valve_m.value(0)

max_pwm=45000
min_pwm=1000
# sensor_address=0x28
sensor_address=0x30
range_max=400
range_min=0

Kp=3200 # sim
Ki=10
Kd=0

# Kp=2800 # arm
# Ki=10
# Kd=0


# Kp=6000 # finger
# Ki=10
# Kd=0


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

def calculate_ref(k,P_init,current_t):
    P_ref=k*current_t+P_init
    return P_ref

def value_calibrate(v):
    if(v>max_pwm or v<min_pwm): return max_pwm
    else: return v
    
def read(address,pmin,pmax): # ams sensor read
    data=i2c.readfrom(address,2)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure*0.75,2)
    return pressure

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

def calculate_k(P_init,P_end,T_end):
    k=(P_end-P_init)/T_end
    return k

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
    current_pressure=read(sensor_address,range_min,range_max)
    P_init,start=set_init(current_pressure)
    k=calculate_k(P_init,P_end,T_end)
    
    while current_pressure>P_end:
        
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(k,P_init,delta)
        current_pressure=read(sensor_address,range_min,range_max)
        release_speed=regulation(P_ref,current_pressure,release_speed)# state 2: regulation
        valve_on(int(release_speed))# state3: release
        pulse=current_pressure-P_ref
        print(delta,',',pulse)
        print(delta,",",current_pressure,",",P_ref,",",pulse)
    release()