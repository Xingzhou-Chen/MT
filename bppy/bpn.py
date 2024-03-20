from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 

valve = PWM(Pin(9))
valve.freq(100000)

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
pump = Pin(26,Pin.OUT)
# valve = machine.Pin(27,machine.Pin.OUT)
max_pwm=24700
min_pwm=8000
Target_Pressure=300
read_delay=10000
P_end=60
T_end=40
sensor_address=0x28
range_max=400
Kp=400
Ki=200
# Kd=0.015
Kd=0
last_err=0
next_err=0
err=0
# W=[-0.00538172154426763, 0.000851650071231246, -6.37302524645656e-18, -0.000892152217359604, 0.00590598934597558, 0.00698698963106446, -0.00620212695737074, -0.0137040609832883, 5.52707877453251e-18, 0.014432493265307, 0.00687936638544199, -0.0081632107061631, -0.00726939402547478, 0.00115711168211491, -6.53424460760387e-18, -0.00122757877241296, 0.00818245883509788, 0.00975064563179455, -0.00872210398291863, -0.0194297809634266, 7.12732121046067e-17, 0.020830952013038, 0.0100269381726881, -0.012023100584196, -0.0108268074922404, 0.00174408194289992, -6.65064558880939e-18, -0.00190019026208035, 0.0128554479944632, 0.0155673731790009, -0.0141699799274261, -0.032169562231005, 3.34581612461183e-17, 0.0360172914427287, 0.0177746780054562, -0.0219092368628671, -0.0203435756851267, 0.00339147616745318, -6.72098688064347e-18, -0.00401278440832404, 0.0285475039901129, 0.0366433623769002, -0.0357155764132216, -0.0879821582442563, 3.36931554226986e-17, 0.123246761698926, 0.0715143865440606, -0.110122295277928, -0.143070405397244, 0.044269358051324, 0.173017455817979, 0.044269358051324, -0.143070405397244, -0.110122295277928, 0.0715143865440606, 0.123246761698926, 3.36931554226986e-17, -0.0879821582442563, -0.0357155764132216, 0.0366433623769002, 0.0285475039901129, -0.00401278440832404, -6.72098688064347e-18, 0.00339147616745318, -0.0203435756851267, -0.0219092368628671, 0.0177746780054562, 0.0360172914427287, 3.34581612461183e-17, -0.032169562231005, -0.0141699799274261, 0.0155673731790009, 0.0128554479944632, -0.00190019026208035, -6.65064558880939e-18, 0.00174408194289992, -0.0108268074922404, -0.012023100584196, 0.0100269381726881, 0.020830952013038, 7.12732121046067e-17, -0.0194297809634266, -0.00872210398291863, 0.00975064563179455, 0.00818245883509788, -0.00122757877241296, -6.53424460760387e-18, 0.00115711168211491, -0.00726939402547478, -0.0081632107061631, 0.00687936638544199, 0.014432493265307, 5.52707877453251e-18, -0.0137040609832883, -0.00620212695737074, 0.00698698963106446, 0.00590598934597558, -0.000892152217359604, -6.37302524645656e-18, 0.000851650071231246, -0.00538172154426763]
W=[0.00175960610753333, -6.73469691476748e-18, -0.00193102055517429, 0.0131082001622462, 0.0159245905783258, -0.0145395462217419, -0.033104538838168, 3.45253276876892e-17, 0.0372626006169088, 0.0184341735713986, -0.022774184833192, -0.0211919416492861, 0.00353992982203357, -7.02807231982489e-18, -0.00420321352936376, 0.0299482636296203, 0.0384948269390895, -0.0375667958293087, -0.0926438557347975, 3.55120275370235e-17, 0.130004227486035, 0.0754848986085543, -0.11629558876423, -0.151145711744218, 0.0467782580487479, 0.182836263915027, 0.0467782580487479, -0.151145711744218, -0.11629558876423, 0.0754848986085543, 0.130004227486035, 3.55120275370235e-17, -0.0926438557347975, -0.0375667958293087, 0.0384948269390895, 0.0299482636296203, -0.00420321352936376, -7.02807231982489e-18, 0.00353992982203357, -0.0211919416492861, -0.022774184833192, 0.0184341735713986, 0.0372626006169088, 3.45253276876892e-17, -0.033104538838168, -0.0145395462217419, 0.0159245905783258, 0.0131082001622462, -0.00193102055517429, -6.73469691476748e-18, 0.00175960610753333]
# W=[0.000135962062921371, -5.41318240902894e-19, -0.000174481714897857, 0.00140198675075756, 0.00206740737086389, -0.00230501066463107, -0.00637489232520866, 7.98718259396769e-18, 0.0102176531891503, 0.00590735457731037, -0.00841154545500157, -0.0089025986664351, 0.0016704912220353, -3.68237789892765e-18, -0.00241871644932316, 0.0187350318566239, 0.0259296763739894, -0.0270006250483734, -0.070440417326252, 2.83298666664898e-17, 0.107958808088199, 0.0647551314842741, -0.102297149569726, -0.135339272998054, 0.0423340300757474, 0.166051931868918, 0.0423340300757474, -0.135339272998054, -0.102297149569726, 0.0647551314842741, 0.107958808088199, 2.83298666664898e-17, -0.070440417326252, -0.0270006250483734, 0.0259296763739894, 0.0187350318566239, -0.00241871644932316, -3.68237789892765e-18, 0.0016704912220353, -0.0089025986664351, -0.00841154545500157, 0.00590735457731037, 0.0102176531891503, 7.98718259396769e-18, -0.00637489232520866, -0.00230501066463107, 0.00206740737086389, 0.00140198675075756, -0.000174481714897857, -5.41318240902894e-19, 0.000135962062921371]
peak=0
mp=0
def pump_on():
    pump.value(1)

def pump_off():
    pump.value(0)
    
def valve_off():
    valve.duty_u16(max_pwm)

def valve_on(release_speed):
    valve.duty_u16(release_speed)

def release():
    valve.duty_u16(0)

def calculate_ref(P_init,T_end,current_t):
    k=(P_end-P_init)/T_end
    P_ref=k*current_t+P_init
    return P_ref

def value_calibrate(v):
    if(v>max_pwm or v<min_pwm): return max_pwm
    else: return v
    
def PID_control(Kp,Ki,Kd,set_point,current_p,current_t):
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

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = ((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin
    pressure=round(pressure,2)
    return pressure

# def read_weights():
#     W=[]
#     with open('weights.csv','r') as weights:
#         for weight in weights:
#             data=weight.rstrip('\n').rstrip('\r').split(delim)
#             W.append(float(data[0]))
#     return W

def filter(p,W):
    filtered=0
    i=len(p)
    if i < len(W):
        for j in range(i):
            filtered=filtered + W[j]*p[i-j-1]
    else:      
        for j in range(len(W)):
            filtered=filtered + W[j]*p[i-j-1]
                 
    return filtered

def peak_hb(current_p,filtered_p):
#     print(peak)
    global peak
    global mp
    if(filtered_p>peak and current_p<220):
#         print("inthecondition ")
#         print(filtered_p,peak)
        peak=filtered_p
        mp=current_p
    return mp


def sys_dias(mp):
    sys=mp*1.1*0.75
    dias=mp*0.8*0.75
    print("the systolic pressure is: ",sys,"mmHg")
    print("the diastolic pressure is: ",dias,"mmHg")
    

if __name__== '__main__':
    current_pressure=read(sensor_address,0,range_max)
    pump_off()
    P_init=0
    release_speed=max_pwm
    valve_off()
    delim = ','   
    while current_pressure< Target_Pressure:
        pump_on()
        current_pressure=read(sensor_address,0,range_max)
        print(current_pressure)
        sleep(0.1)
    
#     W=read_weights()
#     print(W)
    pump_off()
    time.sleep_ms(1000)
    init_set=0
    f=0
    file_pressure=open("pressure.csv","w")
    file_filtered=open("filtered_pressure.csv","w")
    while current_pressure>P_end:
#         print(unfiltered)
        unfiltered=[]
        valve_on(int(release_speed))
        current_pressure=read(sensor_address,0,range_max)
        if(init_set==0):
            P_init=current_pressure
            f=current_pressure
            start = time.ticks_ms()
            init_set=1
            
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(P_init,T_end,delta)
#         file.write(str(delta)+","+str(current_pressure)+","+str(P_ref)+"\n")# data is written as a string in the C
#         file.write(str(delta)+","+str(current_pressure)+"\n")# data is written as a string in the C
#         file.flush()
        u=PID_control(Kp,Ki,Kd,P_ref,current_pressure,delta)
        release_speed=release_speed+u
        release_speed=value_calibrate(release_speed)
        file_pressure.write(str(delta)+","+str(current_pressure)+"\n")
        file_pressure.flush()# save the pressure data
#         print(0,current_pressure,P_ref,range_max)
#         print(heart_beat_filtered*100)
#         file.write(str(delta)+","+str(current_pressure)+","+str(heart_beat_filtered*100)+"\n")
        with open('pressure.csv','r') as file:
            for line in file:
                data=line.rstrip('\n').rstrip('\r').split(delim)
                unfiltered.append(float(data[1]))
            
        f=filter(unfiltered,W)
        peak_hb(current_pressure,f)
        print(0,current_pressure,P_ref,range_max)
#         print(current_pressure,P_ref)
#         print(0,current_pressure,f,P_ref,range_max)
        file_filtered.write(str(delta)+","+str(current_pressure)+","+str(f)+"\n")# data is written as a string in the C
        file_filtered.flush()
#         print(0,current_pressure,f,range_max)
#         time.sleep_us(read_delay)
#         print(0,current_pressure,P_ref,range_max,P_ref-current_pressure,u)
    release()
#     print(mp)
    sys_dias(mp)
#     print(calculate_ref(P_init,T_end,T_end))

    