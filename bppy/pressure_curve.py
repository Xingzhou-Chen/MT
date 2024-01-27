import pandas as pd
import matplotlib.pyplot as plt 
import csv

x=[]
y=[]
w=[]
file_path = "filtered_pressure.csv"
df_before = pd.read_csv(file_path)
df_before.plot(x=df_before.index[0],title="Pressure_Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)
plt.show()



