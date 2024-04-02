import pandas as pd
import matplotlib.pyplot as plt 
import csv

x=[]
y=[]
file_path = "difference_pressure_fnb.csv"
# file_path = "pressure.csv"
df_before = pd.read_csv(file_path)
df_before.plot(x=df_before.index[0],title="Deflation Curves of Front and Back Volumes",xlabel="Time/s",ylabel="Pressure/mmHg",legend=True)




plt.show()



