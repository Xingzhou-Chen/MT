from machine import I2C,Pin,PWM
import time 

valve = PWM(Pin(11))
valve.freq(100000)


i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000_000)
pump = Pin(10,Pin.OUT)

Target_Pressure=160                                                         
P_end=Target_Pressure-120                                                            # deflation range 120 mmHg
T_end=40                                                                             # deflation time 40 s


valve_m = Pin(14,Pin.OUT)
valve_m.value(0)                                                                     # turn off the middle valve to exclude the baseline volume

max_pwm=45000
min_pwm=1000
# sensor_address=0x28
sensor_address=0x30                                                         
range_max=400                                                                        # measurement range (mbar)
range_min=0

Kp=3200                                                                              # P gain            
Ki=10                                                                                # I gain 
Kd=0                                                                                 # D gain 

# Kp=2800                                                                            # P, I, D gains for arm cuff
# Ki=10
# Kd=0


# Kp=6000                                                                            # P,I,D gains for finger cuff
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

## Calibration function
## restrict the change of control magnitude within the certain range, so that the system can recover faster at dirsturbance
def value_calibrate(v):
    if(v>max_pwm or v<min_pwm): return max_pwm
    else: return v

## AMS 6915 reading function
## modified based on the ams arduino library 
def read(address,pmin,pmax):
    data=i2c.readfrom(address,2)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=pressure*0.75
    return pressure

## inflation funtion 
## inflate in 3 stages:
## stage 1: direct pumping 
## stage 2: step-wise pumping 
## stage 3: release gradually to target pressure
def pump_up(Target_Pressure):
    valve_off()
    current_pressure=read(sensor_address,range_min,range_max)
    while current_pressure< Target_Pressure-40:                                       # stage 1: direct pumping 
        pump_on()
        time.sleep(0.2)
        pump_off()
        current_pressure=read(sensor_address,range_min,range_max)

    while current_pressure< Target_Pressure:                                          # stage 2: step-wise pumping
        pump_on()
        time.sleep(0.2)
        pump_off()
        time.sleep(0.3)
        current_pressure=read(sensor_address,range_min,range_max)
    
    while current_pressure> Target_Pressure+1:                                        # stage 3: release gradually to target pressure
        valve_on(29000)
        current_pressure=read(sensor_address,range_min,range_max) 
    
    valve_off() 
    pump_off()

## record the initial pressure and starting time 
def set_init(current_pressure):
    P_init=current_pressure
    start = time.ticks_ms()
    return P_init,start

## calculate the slope
## parameter: P_init: inital pressure 
## P_end: final release pressure 
## T_end: measurement time 
def calculate_k(P_init,P_end,T_end):
    k=(P_end-P_init)/T_end
    return k

## incremental form PID
## return delta u 
def PID_control(Kp,Ki,Kd,set_point,current_p):
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

## control algorithm 
## PID controller computes the delta u 
## parameter release_speed: current control magnitude
## return release_speed: control magnitude for the next round 
def regulation(P_ref,current_pressure,release_speed):
    u=PID_control(Kp,Ki,Kd,P_ref,current_pressure)
    release_speed=u+release_speed
    release_speed=value_calibrate(release_speed)                                    # calibrate the computed value before it is applied
    return release_speed


if __name__== '__main__':
    
    pump_up(Target_Pressure)                                                        # state 1: pump up 
    release_speed=max_pwm
    init_set=0                                                                      # once the initial state is recorded, the flag turns to 1 
    current_pressure=read(sensor_address,range_min,range_max)
    P_init,start=set_init(current_pressure)
    k=calculate_k(P_init,P_end,T_end)
    
    while current_pressure>P_end:
        
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(k,P_init,delta)
        current_pressure=read(sensor_address,range_min,range_max)
        release_speed=regulation(P_ref,current_pressure,release_speed)               # state 2: regulation
        valve_on(int(release_speed))                                                 # state 3: release
        pulse=current_pressure-P_ref                                                 # state 4: extract the OWM    
        print(delta,',',pulse)                                                      
    release()                                                                        # measurement ends