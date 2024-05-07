from machine import I2C,Pin,PWM,UART
import time 

valve = PWM(Pin(11))
valve.freq(100000)

# uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
uart = UART(0, baudrate=512000, tx=Pin(0), rx=Pin(1))
i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=1000000)
i2c_hx = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000000)
pump = Pin(10,Pin.OUT)

def valve_off():
    valve.duty_u16(65535)

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure*0.75,2)
    return pressure

def set_up_hx():
    i2c_hx.writeto(0x28,b'\xBD')
    i2c_hx.writeto(0x28,b'\xCE')
    time.sleep(1)

def read_adc():
    data=i2c_hx.readfrom(0x28,4)
    pres=(data[1]<<16)+(data[2]<<8)+data[3]
    return pres

def read_hx():
    adccount=read_adc()
    pressure=(adccount-8518000)/5830.09
    pressure=round(pressure,2)
    return pressure

if __name__== '__main__':
    start=time.ticks_ms()
    valve_off()
    set_up_hx()
    while 1:    
        delta=time.ticks_diff(time.ticks_ms(),start)/1000
        # current_pressure=read(0x28,0,400)
        current_pressure=read_hx()
        # pulse=current_pressure-P_ref
        print(delta,',',current_pressure)
        # uart.write(str(delta)+" , "+str(current_pressure)+"\n")
        # time.sleep(0.1)