import pandas as pd
import matplotlib.pyplot as plt 
import csv

x=[]
y=[]
file_path = "add.csv"
df_before = pd.read_csv(file_path)
# t=df.index[0]
# P_ref=P_init-k_ref*t
# print(df.index[0])
with open('add.csv','r') as file:
    tests=csv.reader(file)
    for test in tests:
#         print(test)
        x.append(test[0])
        y.append(test[1])
                 

datafram=pd.DataFrame({'a':x,'b':y})
datafram.to_csv("test.csv",index=False,sep=',')
file_path = "test.csv"
df_filtered = pd.read_csv(file_path)
df_filtered.plot(x=df.index[0],title="Test",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)
# # df.plot(x=df.index[0],title="Pressure Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)
plt.show()