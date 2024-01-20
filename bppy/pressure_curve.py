import pandas as pd
import matplotlib.pyplot as plt 

file_path = "add.csv"
df = pd.read_csv(file_path)
# t=df.index[0]
# P_ref=P_init-k_ref*t
print(df.index[0])
df.plot(x=df.index[0],title="Pressure Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)
# df.plot(x=df.index[0],title="Pressure Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)
plt.show()

    