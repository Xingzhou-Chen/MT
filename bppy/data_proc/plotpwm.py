import pandas as pd
import matplotlib.pyplot as plt 
import csv
import numpy as np

# x=[]
# y=[]

file0_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/diffpwm/pwm3000.csv"
# file_path = "pressure.csv"
df0 = pd.read_csv(file0_path)
df0.columns=['t','pwm']
t0=df0['t']
d0=df0['pwm']

file1_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/diffpwm/pwm4000.csv"
# file_path = "pressure.csv"
df1 = pd.read_csv(file1_path)
df1.columns=['t','pwm']
t1=df1['t']
d1=df1['pwm']
# print(t1)

file2_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/diffpwm/pwm5000.csv"
# file_path = "pressure.csv"
df2 = pd.read_csv(file2_path)
df2.columns=['t','pwm']
t2=df2['t']
d2=df2['pwm']
# print(t2)

plt.plot(t0, d0, label='PWM=3000')
plt.plot(t1, d1, label='PWM=4000')
plt.plot(t2, d2, label='PWM=5000')

# Add labels and title
plt.xlabel('Time/s')
plt.ylabel('Pressure/mmHg')
# plt.title('Deflation Curves with Different Restriction')

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