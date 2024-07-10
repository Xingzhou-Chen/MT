import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
inc = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/OWM/inc/sim_owm.csv'
ff = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/OWM/ff/sim_owm7.csv'
# ds = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/OWM/ff/sim_owm7.csv'
ds = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/OWM/ds/sim_owm3.csv'
step = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/OWM/step/step_owm.csv'

df1 = pd.read_csv(inc)
df2 = pd.read_csv(ff)
df3 = pd.read_csv(ds)
df4 = pd.read_csv(step)

# Rename the columns for clarity
df1.columns = ['Time','OWM']
df2.columns = ['Time','OWM']
df3.columns = ['Time','OWM']
df4.columns = ['Time','OWM']

# Create subplots
fig, axs = plt.subplots(3, 1, sharex=True,figsize=(10, 10),gridspec_kw={'hspace': 0})

# Plot the first dataset
axs[0].plot(df1['Time'], df1['OWM'],color='r', label='Incremental PID')
# axs[0].text(0.5, 0.95, 'Incremental PID', horizontalalignment='center', verticalalignment='center', transform=axs[0].transAxes)
axs[0].legend(loc='upper right')
axs[0].grid(False)
axs[0].set_ylim(-1,2.2)
# axs[0].set_ylim(-2,3.8)

# Plot the second dataset
axs[1].plot(df2['Time'], df2['OWM'],color='b', label='Feedforward PID')
# axs[1].text(0.5, 0.95, 'Feedforward PID', horizontalalignment='center', verticalalignment='center', transform=axs[1].transAxes)
axs[1].legend(loc='upper right')
axs[1].grid(False)
# axs[1].set_xlabel('Time/s')
axs[1].set_ylim(-1,2.2)
# axs[1][0].set_ylim(-2,3.8)

# # Plot the third dataset
axs[2].plot(df3['Time'], df3['OWM'],color='y', label='Linear Differential Sensor System')
# axs[1].text(0.5, 0.95, 'Feedforward PID', horizontalalignment='center', verticalalignment='center', transform=axs[1].transAxes)
axs[2].legend(loc='upper right')
axs[2].grid(False)
axs[2].set_ylim(-1,2.2)
axs[2].set_xlim(11,14)
axs[2].set_xlabel('Time/s')
# axs[0][1].set_ylim(-2,3.8)

# axs[1][1].remove()
# Plot the forth dataset
# axs[1][1].plot(df4['Time'], df4['OWM']-50, color='g', label='Step-wise Differential Sensor System')
# axs[1][1].legend(loc='upper right')
# axs[1][1].set_xlabel('Time/s')
# axs[1][1].grid(False)
# axs[3].set_ylim(-1,2)

# Improve layout
fig.text(0.01, 0.5, 'Pulse Amplitude/mmHg', va='center', rotation='vertical')
# fig.text(0.5, 0.1, 'Time/s', va='center')
# fig.suptitle('Pulse Waves on Simulator')
plt.tight_layout()
plt.show()