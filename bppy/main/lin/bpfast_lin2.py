from machine import I2C,Pin,PWM
import time 
import sensor
import bpm

P_end=30
max_pwm=40000
Target_Pressure=180

valve = PWM(Pin(11)).freq(100000)

pump = Pin(10,Pin.OUT)
i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)

pid=bpm.PID_Controller(P_end=P_end,T_end=40,Kp=400,Ki=50,Kd=0,min_pwm=2000,max_pwm=max_pwm)
circuit=bpm.Circuit(pump,valve)
ams=sensor.AMS(i2c=i2c,address=0x28,pmin=0,pmax=400)   

if __name__== '__main__':
    current_pressure=ams.read()
    circuit.pump_off()
    P_init=0
    release_speed=max_pwm
    circuit.valve_off()
    while current_pressure< Target_Pressure:
        circuit.pump_on()
        current_pressure=ams.read()
        time.sleep_ms(100)
    
    circuit.pump_off()
    time.sleep(1)
    init_set=0
    while current_pressure>P_end:
        circuit.valve_on(int(release_speed))
        current_pressure=ams.read()
        if(init_set==0):
            P_init=current_pressure
            f=current_pressure
            start = time.ticks_ms()
            init_set=1
            
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=pid.calculate_ref(P_init,delta)
        u=pid.PID_control(P_init,current_pressure,delta)
        release_speed=release_speed+u
        release_speed=pid.calibrate(release_speed)

        pulse=current_pressure-P_ref
        print(delta,',',pulse)
    circuit.release()


    
