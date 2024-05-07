from machine import I2C,Pin,PWM,UART
import time

H_co=0.9
L_co=0.5
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=1000000)
            
def sigle_pole_hpf(current_p,filtered_p,previous_p):
    
    filtered_p=H_co*(filtered_p + current_p - previous_p)
    previous_p=current_p
    return filtered_p

def sigle_pole_lpf(unfiltered_p,filtered_p):
    return L_co*unfiltered_p+(1-L_co)*filtered_p

def read(address,pmin,pmax):
    data=i2c.readfrom(address,4)
    pressureM=data[0]
    pressureL=data[1]
    pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(pmax-pmin)/13107+pmin)
    pressure=round(pressure,2)
    return pressure*0.75

if __name__== '__main__':
#     a=0
#     signal=0
    filtered=0
    x=[]
    y=[]
    f=[]
    previous_p=0
    with open('owm_mat.csv','r') as file:
        tests=csv.reader(file)
        for test in tests:
            x.append(float(test[0]))
            y.append(float(test[1]))


    for i in range(len(y)):
#         signal=512+np.sin(a)*100
#         a+=0.1
        filtered=sigle_pole_hpf(y[i],filtered,previous_p)
        previous_p=y[i]
#         filtered=sigle_pole_lpf(y[i],filtered,previous_p)
        f.append(filtered)
        print(y[i],filtered)
#         hpf()
    plt.plot(x,f)
    plt.show()
        
        
        