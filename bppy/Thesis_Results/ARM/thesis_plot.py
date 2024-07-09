import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
inc = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/ARM/OWM/inc/arm_owm_final.csv'
ff = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/ARM/OWM/ff/arm_owm2_ff.csv'
ds = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/ARM/OWM/ds/arm_owm3.csv'

df1 = pd.read_csv(inc)
df2 = pd.read_csv(ff)
df3 = pd.read_csv(ds)

# Rename the columns for clarity
df1.columns = ['Time', 'Pulse Amplitude']
df2.columns = ['Time', 'Pulse Amplitude']
df3.columns = ['Time', 'Pulse Amplitude']

# Create subplots
fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 10),gridspec_kw={'hspace': 0})

# Plot the first dataset
axs[0].plot(df1['Time'], df1['Pulse Amplitude'], color='r', label='Incremental Form PID')
axs[0].legend(loc='upper right')
axs[0].grid(False)
axs[0].set_xlim(9,14.5)
axs[0].set_ylim(-0.8,2)
# axs[0].set_xlim(10.4,11)
# axs[0].set_ylim(-0.2,0.2)
# axs[0].set_ylim(-1.2,2.6)

# Plot the second dataset
axs[1].plot(df2['Time'], df2['Pulse Amplitude'], color='b', label='Feedforward PID')
# axs[1].set_ylabel('Pulse Amplitude/mmHg')
axs[1].legend(loc='upper right')
axs[1].grid(False)
# axs[1].set_xlim(9,15)
axs[1].set_ylim(-0.8,2)
# axs[1].set_xlim(10.4,10.8)
# axs[1].set_ylim(-0.2,0.2)
# axs[1].set_ylim(-1.2,2.6)


# Plot the third dataset
axs[2].plot(df3['Time'], df3['Pulse Amplitude'], color='y', label='Linear Differential Sensor System')
axs[2].legend(loc='upper right')
axs[2].grid(False)
# axs[2].set_xlim(9,15)
# axs[2].set_ylim(-1,2)
axs[2].set_ylim(-0.8,2)
# axs[2].set_xlim(10.4,10.8)
# axs[2].set_ylim(-0.2,0.2)
# axs[2].set_ylim(-1.2,2.6)

# Improve layout
fig.text(0.01, 0.5, 'Pulse Amplitude/mmHg', va='center', rotation='vertical')
# fig.suptitle('Noise of Pulse Waves on Arm')
plt.tight_layout()
plt.show()