import pandas as pd
import matplotlib.pyplot as plt 

x=[]
y=[]
file_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/data/difference/difference_ba.csv"
# file_path = "pressure.csv"
df_before = pd.read_csv(file_path)
df_before.plot(x=df_before.index[0],title="Oscillometric Waveform on Simulator",xlabel="Time/s",ylabel="Pressure/mmHg",legend=False)




plt.show()



