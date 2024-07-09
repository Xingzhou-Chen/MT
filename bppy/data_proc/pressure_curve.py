import pandas as pd
import matplotlib.pyplot as plt 


file_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/diffpwm/pwm5000.csv" # pfad zu Daten
df_before = pd.read_csv(file_path)
# df_before.columns=['Time','Cuff Pressure','Target Pressure','a']
# df_before.plot(x=df_before.index[0],title="Oscillometric Waveform",xlabel="Time/s",ylabel="Pressure/mmHg",legend=False)
df_before.plot(x=df_before.index[0],xlabel="Time/s",ylabel="Pressure/mmHg",legend=False)
plt.show()



