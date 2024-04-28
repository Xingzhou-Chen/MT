from machine import I2C,Pin,PWM
from utime import sleep

valve = PWM(Pin(9))
valve.freq(10000)

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
pump = machine.Pin(26,machine.Pin.OUT)
# valve = machine.Pin(27,machine.Pin.OUT)

Target_Pressure=300
read_delay=0.2
release_speed=20000

def pump_on():
    pump.value(1)

def pump_off():
    pump.value(0)
    
def valve_off():
    valve.duty_u16(65536)

def valve_on():
    valve.duty_u16(release_speed)

def release():
    valve.duty_u16(0)
 
def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = ((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin
    pressure=round(pressure,2)
    return pressure

if __name__== '__main__':
    current_pressure=read(0x28,0,350)
    pump_off()
    time_release=0
    
    
    valve_off()
    file=open("add.csv","w")
    while current_pressure< Target_Pressure:
        pump_on()
        current_pressure=read(0x28,0,350)
        print(current_pressure)
        sleep(0.1)
        
    pump_off()
    while current_pressure>50:
        valve_on()
        current_pressure=read(0x28,0,350)
        print(0,current_pressure,350)
        file.write(str(time_release)+","+str(current_pressure)+"\n")# data is written as a string in the CSV file
        file.flush()
        sleep(read_delay)
        time_release=time_release+read_delay
    release() 
