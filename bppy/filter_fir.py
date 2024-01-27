import pandas as pd
import matplotlib.pyplot as plt 
import csv

x=[]
y=[]
w=[]
# filtered_array=[]
filtered=[]
# file_path = "owm_mat.csv"
# df_before = pd.read_csv(file_path)
# df_before.plot(x=df_before.index[0],title="Pressure_Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)

def filter(p,W):
        for i in range(len(p)):
            sum=0
            if i < len(W):
                for j in range(i):
                    sum=sum + W[j]*p[i-j]
            else:      
                for j in range(len(W)):
                    sum=sum + W[j]*p[i-j]
            filtered.append(sum)   
        return filtered

# def filter(p,W):
# #         for i in range(len(p)):
#     filtered=0
#     i=len(p)
#     print(len(p))
#     if i < len(W):
#         for j in range(i):
#             filtered=filtered + W[j]*p[i-j-1]
#     else:      
#         for j in range(len(W)):
#             filtered=filtered + W[j]*p[i-j-1]
#                  
#     return filtered

if __name__== '__main__':

    with open('owm_mat.csv','r') as file:
        tests=csv.reader(file)
        for test in tests:
#         print(test)
            x.append(float(test[0]))
            y.append(float(test[1]))
        
    with open('weights.csv','r') as file:
        weights=csv.reader(file)
        for weight in weights:
#         print(test)
            w.append(float(weight[0]))
            
    filtered=filter(y,w)
#     print(w)
#     plt.plot(x,y)
    
#     for i in range(len(y)):
# #         print(i)
#         filtered_array.append(filter(y[0:i],w))
        
#     print(filtered_array)   
    filtered_file_path = "filtered.csv"
    datafram=pd.DataFrame({'a':x,'b':filtered})
#     datafram=pd.DataFrame({'a':x,'b':filtered_array})
    datafram.to_csv(filtered_file_path,index=False,sep=',')
    df_filtered = pd.read_csv(filtered_file_path)
    df_filtered.plot(x=df_filtered.index[0],title="Filtered_Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)
    plt.show()





