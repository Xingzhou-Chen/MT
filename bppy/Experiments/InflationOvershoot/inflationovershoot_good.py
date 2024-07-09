from machine import I2C,Pin,PWM
import time 

valve = PWM(Pin(11))
valve.freq(100000)

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)
pump = Pin(10,Pin.OUT)

Target_Pressure=150
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

# def calculate_ref(P_init,T_end,current_t):
#     k=(P_end-P_init)/T_end
#     P_ref=k*current_t+P_init
#     return round(P_ref,2)

# def value_calibrate(v):
#     if(v>max_pwm or v<min_pwm): return max_pwm
#     else: return v
    
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

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure*0.75,2)
    return pressure

# def pump_up(Target_Pressure,start):# pump to target pressure
#     current_pressure=read(sensor_address,0,range_max)
#     delta = time.ticks_diff(time.ticks_ms(), start)/1000
#     while current_pressure< Target_Pressure:
#         pump_on()
#         current_pressure=read(sensor_address,0,range_max)
#         # time.sleep(0.1)
#         delta = time.ticks_diff(time.ticks_ms(), start)/1000
#         print(delta,",",current_pressure)
#     pump_off() 
#     return delta

def pump_up(Target_Pressure,start):
    valve_off()
    current_pressure=read(sensor_address,range_min,range_max)
    while current_pressure< Target_Pressure-50:
        pump_on()
        time.sleep(0.2)
        pump_off()
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        current_pressure=read(sensor_address,range_min,range_max)
        print(delta,",",current_pressure,",",Target_Pressure)

    while current_pressure< Target_Pressure:
        pump_on()
        time.sleep(0.2)
        pump_off()
        time.sleep(0.3)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        current_pressure=read(sensor_address,range_min,range_max)
        print(delta,",",current_pressure,",",Target_Pressure)
    
    while current_pressure> Target_Pressure+1: # release f
        valve_on(29000)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        current_pressure=read(sensor_address,range_min,range_max) 
        # print(delta,",",current_pressure)
        print(delta,",",current_pressure,",",Target_Pressure)
    
    valve_off() 
    pump_off()
    return delta

# def set_init(current_pressure):
#     P_init=current_pressure
#     start = time.ticks_ms()
#     return P_init,start

# def regulation(P_init,start,release_speed):
#     delta = time.ticks_diff(time.ticks_ms(), start)/1000
#     P_ref=calculate_ref(P_init,T_end,delta)
#     current_pressure=read(sensor_address,0,range_max)

    
#     u=PID_control(Kp,Ki,Kd,P_ref,current_pressure,delta)
#     release_speed=release_speed+u
#     release_speed=value_calibrate(release_speed)

#     return delta,P_ref,current_pressure,release_speed

if __name__== '__main__':
    start=time.ticks_ms()
    valve_off()
    delta=pump_up(Target_Pressure,start)# state 1: pump up 
    # valve_off()
    while delta<15:
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        current_pressure=read(sensor_address,0,range_max)
        print(delta,",",current_pressure,",",Target_Pressure)
    release()


    
