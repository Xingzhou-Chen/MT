import pandas as pd
import matplotlib.pyplot as plt 
import csv
import numpy as np

# x=[]
# y=[]

file0_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/pwm10000_pressure.csv"
# file_path = "pressure.csv"
df0 = pd.read_csv(file0_path)
t0=df0['t']
d0=df0['pwm=10000']

file1_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/pwm12000_pressure.csv"
# file_path = "pressure.csv"
df1 = pd.read_csv(file1_path)
t1=df1['t']
d1=df1['pwm=12000']
# print(t1)

file2_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/pwm15000_pressure.csv"
# file_path = "pressure.csv"
df2 = pd.read_csv(file2_path)
t2=df2['t']
d2=df2['pwm=15000']
# print(t2)

plt.plot(t0, d0, label='PWM=10000')
plt.plot(t1, d1, label='PWM=12000')
plt.plot(t2, d2, label='PWM=15000')

# Add labels and title
plt.xlabel('Time/s')
plt.ylabel('Pressure/mmHg')
plt.title('Deflation Curves with Different Restriction')

# Add legend
plt.legend()

# Show plot
plt.show()
# df1.plot(x=df1.index[0],title="Deflation Curve with Different Restriction",xlabel="Time/s",ylabel="Pressure/mmHg",legend=True)

# file1_path = "pwm15000_pressure.csv"
# # file_path = "pressure.csv"
# df2 = pd.read_csv(file1_path)
# # df2.plot(x=df2.index[0],title="Deflation Curve with Different Restriction",xlabel="Time/s",ylabel="Pressure/mmHg",legend=True)
# 
# index1 = np.arange(0, max(len(df1), len(df2)), step=max(len(df1), len(df2)) / len(df1))
# index2 = np.arange(0, max(len(df1), len(df2)), step=max(len(df1), len(df2)) / len(df2))
# 
# # Set the new indices
# df1 = df1.set_index(keys=index1)
# df2 = df2.set_index(keys=index2)
# 
# plt.plot(df1.index[0], label="m1, df1")
# plt.plot(df2.index[0], label="m2, df2")
# 
# plt.show()