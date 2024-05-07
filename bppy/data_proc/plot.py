import pandas as pd
import matplotlib.pyplot as plt 
import csv
import numpy as np

# x=[]
# y=[]

file_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/data/constant/constant_pressure.csv"
# file_path = "pressure.csv"
df0 = pd.read_csv(file_path)
t0=df0['t']
d0=df0['p']
# corona_date = (data['Date']).astype(str)
# d1=df0['f']
print(t0)

# print(d1)
# d2=df0['Back Pressure']
# d3=df0['Back Reference']

# print(t0)
# plt.plot(t0, d0, d1,d2,d3)
# plt.plot(t0, d0,d1)
# plt.plot(t0, d1, label='Front Reference')
# plt.plot(t0, d2, label='Back Pressure')
# plt.plot(t0, d2, label='Back Reference')
plt.plot(t0, d0, label='O')
plt.plot(t0, d1, label='F')
# Add labels and title
# plt.xlabel('Time/s')
# plt.ylabel('Pressure/mmHg')
# plt.title('Deflation Curves with Differential Method')

# # Add legend
plt.legend()

# Show plot
plt.show()

# file1_path = "pwm15000_pressure.csv"
# # file_path = "pressure.csv"
