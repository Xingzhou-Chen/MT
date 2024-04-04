import pandas as pd
import matplotlib.pyplot as plt 
import csv
import numpy as np

# x=[]
# y=[]

file_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/difference3.csv"
# file_path = "pressure.csv"
df0 = pd.read_csv(file_path)
t0=df0['t']
d0=df0['Front Pressure']

d1=df0['Front Reference']
# print(t1)

d2=df0['Back Pressure']
d3=df0['Back Reference']

# print(t0)
# plt.plot(t0, d0, d1,d2,d3)
# plt.plot(t0, d1, label='Front Reference')
# plt.plot(t0, d2, label='Back Pressure')
# plt.plot(t0, d2, label='Back Reference')

# Add labels and title
plt.xlabel('Time/s')
plt.ylabel('Pressure/mmHg')
plt.title('Deflation Curves with Differential Method')

# Add legend
plt.legend()

# Show plot
plt.show()
# df1.plot(x=df1.index[0],title="Deflation Curve with Different Restriction",xlabel="Time/s",ylabel="Pressure/mmHg",legend=True)

# file1_path = "pwm15000_pressure.csv"
# # file_path = "pressure.csv"
