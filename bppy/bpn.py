from machine import I2C,Pin,PWM
# import machine 
from utime import sleep
import time 

valve = PWM(Pin(9))
valve.freq(100000)

i2c = I2C(id=1,scl=Pin(7),sda=Pin(6),freq=100000)
pump = Pin(26,Pin.OUT)
# valve = machine.Pin(27,machine.Pin.OUT)
heart_beat_filtered=0
alpha=0.72
filtered=[]
unfiltered=[]
current_pressure_filtered=0
max_pwm=24700
min_pwm=8000
Target_Pressure=350
read_delay=10
release_speed=max_pwm
P_end=80
T_end=40
sensor_address=0x28
range_max=400
Kp=1000
Ki=300
# Kd=0.015
Kd=0
last_err=0
next_err=0
err=0
W=[-0.00105165113702524, -0.00165384645609152, 3.33348757865651e-17, 0.00169208181361085, 0.00110084431621912, -0.00562326838828148, 0.00568978514627213, -0.00114037610621432, -0.00179457590220777, -8.22512089202998e-17, 0.00183861120689143, 0.00119703425019519, -0.00611912584685944, 0.00619618490113283, -0.0012428331680551, -0.00195736291095183, -1.07709754613391e-17, 0.00200870388971152, 0.00130889532305258, -0.00669683917275564, 0.00678730199309257, -0.00136266311602708, -0.00214813123835699, 7.49678788851956e-17, 0.00220885588807579, 0.0014408058611499, -0.00737958919270548, 0.00748745851083445, -0.00150492338335266, -0.00237514199491093, -7.42459650289962e-17, 0.00244820056196272, 0.00159894785837767, -0.00820024848976705, 0.00833129932342351, -0.00167685085986378, -0.00265027190321005, -4.63634281551997e-17, 0.00273999743996576, 0.00179234187839113, -0.00920703498375939, 0.00936991581974468, -0.00188917709250855, -0.0029912334281139, -1.10340670780107e-17, 0.00310425798058136, 0.00203468687973784, -0.0104736457020207, 0.0106819157395685, -0.00215852659102266, -0.0034256598144239, 3.48506627671151e-17, 0.00357266887273693, 0.00234784413879402, -0.0121186431170609, 0.0123948248403836, -0.00251210442404725, -0.00399918015924353, -1.11269084358189e-17, 0.00419855871511906, 0.00276897929031153, -0.0143457190709773, 0.0147301430302544, -0.00299770486870141, -0.00479291539953753, -7.59343015760529e-17, 0.00507917647313358, 0.00336679423595338, -0.0175368050912814, 0.0181094442838846, -0.0037077321675697, -0.00596631331295556, -1.11934657843859e-17, 0.00641262530991639, 0.00428397982585146, -0.02250160802521, 0.023446151399169, -0.00484708527409922, -0.00788175942506846, -1.12168086073187e-17, 0.00867435745778128, 0.00587351130675244, -0.031309395470507, 0.0331581660167061, -0.00697924005088687, -0.0115780984505995, -1.12334972277324e-17, 0.0133638832824895, 0.00931197783362033, -0.0512966891867985, 0.056433544690612, -0.0124207150532233, -0.0217301432893038, -1.12435164732488e-17, 0.0289784437816252, 0.0223648798665237, -0.141155739249505, 0.188215639848137, -0.0559193187365635, -0.173907561753855, 0.288516199836303, -0.173907561753855, -0.0559193187365635, 0.188215639848137, -0.141155739249505, 0.0223648798665237, 0.0289784437816252, -1.12435164732488e-17, -0.0217301432893038, -0.0124207150532233, 0.056433544690612, -0.0512966891867985, 0.00931197783362033, 0.0133638832824895, -1.12334972277324e-17, -0.0115780984505995, -0.00697924005088687, 0.0331581660167061, -0.031309395470507, 0.00587351130675244, 0.00867435745778128, -1.12168086073187e-17, -0.00788175942506846, -0.00484708527409922, 0.023446151399169, -0.02250160802521, 0.00428397982585146, 0.00641262530991639, -1.11934657843859e-17, -0.00596631331295556, -0.0037077321675697, 0.0181094442838846, -0.0175368050912814, 0.00336679423595338, 0.00507917647313358, -7.59343015760529e-17, -0.00479291539953753, -0.00299770486870141, 0.0147301430302544, -0.0143457190709773, 0.00276897929031153, 0.00419855871511906, -1.11269084358189e-17, -0.00399918015924353, -0.00251210442404725, 0.0123948248403836, -0.0121186431170609, 0.00234784413879402, 0.00357266887273693, 3.48506627671151e-17, -0.0034256598144239, -0.00215852659102266, 0.0106819157395685, -0.0104736457020207, 0.00203468687973784, 0.00310425798058136, -1.10340670780107e-17, -0.0029912334281139, -0.00188917709250855, 0.00936991581974468, -0.00920703498375939, 0.00179234187839113, 0.00273999743996576, -4.63634281551997e-17, -0.00265027190321005, -0.00167685085986378, 0.00833129932342351, -0.00820024848976705, 0.00159894785837767, 0.00244820056196272, -7.42459650289962e-17, -0.00237514199491093, -0.00150492338335266, 0.00748745851083445, -0.00737958919270548, 0.0014408058611499, 0.00220885588807579, 7.49678788851956e-17, -0.00214813123835699, -0.00136266311602708, 0.00678730199309257, -0.00669683917275564, 0.00130889532305258, 0.00200870388971152, -1.07709754613391e-17, -0.00195736291095183, -0.0012428331680551, 0.00619618490113283, -0.00611912584685944, 0.00119703425019519, 0.00183861120689143, -8.22512089202998e-17, -0.00179457590220777, -0.00114037610621432, 0.00568978514627213, -0.00562326838828148, 0.00110084431621912, 0.00169208181361085, 3.33348757865651e-17, -0.00165384645609152, -0.00105165113702524]

def pump_on():
    pump.value(1)

def pump_off():
    pump.value(0)
    
def valve_off():
    valve.duty_u16(max_pwm)

def valve_on(release_speed):
    valve.duty_u16(release_speed)

def low_pass_filter(prev_value,new_value):
    return alpha*prev_value+(1-alpha)*new_value

def release():
    valve.duty_u16(0)

def calculate_ref(P_init,T_end,current_t):
    k=(P_end-P_init)/T_end
    P_ref=k*current_t+P_init
    return P_ref

def value_calibrate(v):
    if(v>max_pwm or v<min_pwm): return max_pwm
#     if(v>max_pwm ): return max_pwm
#     if(v<0): return max_pwm
    else: return v
    
def PID_control(Kp,Ki,Kd,set_point,current_p,current_t):
    global err
    global last_err
    global next_err
    
#     err=current_p-set_point
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

def filter(p):
        for i in range(len(p)):
            filtered=0
            if i < len(W):
                for j in range(i):
                    filtered=filtered + W[j]*p[i-j]
            else:      
                for j in range(len(W)):
                    filtered=filtered + W[j]*p[i-j]
                
#             filtered.append(sum)   
        return filtered
peak=0
def peak_hb(current_p,filtered_p,mp):
#     print(peak)
    global peak
    if(filtered_p>peak):
#         print("inthecondition ")
#         print(filtered_p,peak)
        peak=filtered_p
        mp=current_p       
    return mp


def sys_dias(a):
    sys=mp*0.85*0.75
    dias=mp*0.55*0.75
    print("the systolic pressure is: ",sys,"mmHg")
    print("the diastolic pressure is: ",dias,"mmHg")
    
# def min_pwm(pwm):
#     current_pwm=max_pwm
#     if(pwm<0):pwm=-pwm
#     if(current_pwm>pwm):
#         current_pwm=pwm
#     return current_pwm

if __name__== '__main__':
    current_pressure=read(sensor_address,0,range_max)
    pump_off()
    P_init=0
    
    valve_off()
    file=open("add.csv","w")
    while current_pressure< Target_Pressure:
        pump_on()
        current_pressure=read(sensor_address,0,range_max)
        print(current_pressure)
        sleep(0.1)
        
    pump_off()
    time.sleep_ms(1000)
    init_set=0
    mp=0
    while current_pressure>P_end:
        valve_on(int(release_speed))
        current_pressure=read(sensor_address,0,range_max)
        if(init_set==0):
            P_init=current_pressure
            current_pressure_filtered=current_pressure
            start = time.ticks_ms()
            init_set=1
            
        delta = time.ticks_diff(time.ticks_ms(), start)/1000
        P_ref=calculate_ref(P_init,T_end,delta)
#         file.write(str(delta)+","+str(current_pressure)+","+str(P_ref)+"\n")# data is written as a string in the C
#         file.write(str(delta)+","+str(current_pressure)+"\n")# data is written as a string in the C
#         file.flush()
        u=PID_control(Kp,Ki,Kd,P_ref,current_pressure,delta)
        release_speed=release_speed+u
#         min_release_pwm(release_speed)
        release_speed=value_calibrate(release_speed)      
#         print(release_speed)
#         print(0,current_pressure,P_ref,range_max)
#         print(heart_beat_filtered*100)
#         file.write(str(delta)+","+str(current_pressure)+","+str(heart_beat_filtered*100)+"\n")
#         e_1=(P_ref-current_pressure)*100
#         if(e<0):
#             file.write(str(delta)+","+str(current_pressure)+","+str(current_pressure_filtered)+","+str(e)+"\n")# data is written as a string in the C
#             file.flush()
        unfiltered.append(current_pressure)
        f=filter(unfiltered)
        mp=peak_hb(current_pressure,f,mp)
        print(0,current_pressure,P_ref,range_max)
#         print(f,peak)
#         file.write(str(delta)+","+str(current_pressure)+","+str(f)+"\n")# data is written as a string in the C
#         file.flush()
#         print(0,current_pressure,current_pressure_filtered,range_max)
        time.sleep_us(read_delay)
#         print(0,current_pressure,P_ref,range_max,P_ref-current_pressure,u)
#     print(release_speed)
    release()
    sys_dias(mp)
#     print(calculate_ref(P_init,T_end,T_end))

