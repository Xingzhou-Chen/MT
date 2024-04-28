import csv
import numpy as np
import matplotlib.pyplot as plt

alpha=0.9
belta=0.7

            
def sigle_pole_hpf(current_p,filtered_p,previous_p):

    filtered_p=belta*(filtered_p + current_p - previous_p)
#     previous_p=current_p
    return filtered_p

def sigle_pole_lpf(previous_p,filtered_p):
    return alpha*previous_p+(1-alpha)*filtered_p

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
        
        
        