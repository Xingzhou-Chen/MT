from machine import I2C,Pin,PWM,UART 
import time 

valve = PWM(Pin(11))
valve.freq(100_000)

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000000)
i2c_ams = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)
pump = Pin(10,Pin.OUT)

max_pwm=35000
min_pwm=2000

Target_Pressure=150
P_end=50
T_end=40

sensor_address=0x28
range_max=400

Kp=2000
Ki=10
Kd=1000

last_err=0
next_err=0
err=0
int_err=0

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

def calculate_k(P_init,P_end,T_end):
    k=(P_end-P_init)/T_end
    return k 

def calculate_ref(k,P_init,current_t):
    P_ref=k*current_t+P_init
    return P_ref

def value_calibrate(v):
    if(v>max_pwm or v<min_pwm): return max_pwm
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

def feed_forward(t):
    # ff=-0.001342*t**5 + 0.12*t**4 - 3.535*t**3 + 36.21*t**2 - 195.2*t + 2.964e+04
    ff=18523.6-342.0*t # new
    return ff

def read(address,pmin,pmax):
    data=i2c_ams.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure*0.75,2)
    return pressure

def set_up_hx():
    i2c.writeto(0x28,b'\xBD')
    i2c.writeto(0x28,b'\xCE')
    time.sleep(1)

def read_adc():
    data=i2c.readfrom(0x28,4)
    pres=(data[1]<<16)+(data[2]<<8)+data[3]
    return pres

def read_hx():
    adccount=read_adc()
    pressure=(adccount-8518000)/5830.09
    pressure=round(pressure,2)
    return pressure

def pump_up(Target_Pressure):
    valve_off()
    current_pressure=read(0x28,0,range_max)

    while current_pressure< Target_Pressure-50:
        pump_on()
        # current_pressure=read_hx()
        time.sleep(0.2)
        pump_off()
        current_pressure=read(0x28,0,range_max)

    while current_pressure< Target_Pressure:
        pump_on()
        # current_pressure=read_hx()
        time.sleep(0.2)
        pump_off()
        time.sleep(0.3)
        current_pressure=read(0x28,0,range_max)
        print(current_pressure)

    current_pressure=read(0x28,0,range_max)

    while current_pressure> Target_Pressure+1:
        valve_on(29000)
        current_pressure=read(0x28,0,range_max) 
    valve_off()    
    
def set_init(current_pressure):
    P_init=current_pressure
    start = time.ticks_ms()
    return P_init,start

def regulation(delta,P_ref,current_pressure):
    Kf=feed_forward(delta)
    release_speed=PID_control(Kp,Ki,Kd,P_ref,current_pressure)+Kf
    release_speed=value_calibrate(release_speed)
    return release_speed

if __name__== '__main__':
    # set_up_hx()
    pump_up(Target_Pressure)                                        # state 1: pump up
    time.sleep_ms(5000)
    # current_pressure=read_hx()
    current_pressure=read(0x28,0,range_max)
    P_init,start=set_init(current_pressure)                         # determine the initial state
    k=calculate_k(P_init,P_end,T_end)

    while current_pressure>P_end:                                   # deflation      
        # current_pressure=read_hx()
        current_pressure=read(0x28,0,range_max)
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(k,P_init,delta)
        release_speed=regulation(delta,P_ref,current_pressure)      #state 2: regulation 

        valve_on(int(release_speed))                                #state3: release

        pulse=current_pressure-P_ref+10
        uart.write(str(delta)+" , "+str(pulse)+"\n")

    release()                                                       # final release


    


