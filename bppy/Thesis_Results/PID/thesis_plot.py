import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
p = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/PID/I_large.csv'
pi = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/PID/I_large3.csv'
# pid = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/PID/PID_arm.csv'
# step = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Experiments/feedforward/arm_owm1.csv'

df1 = pd.read_csv(p)
df2 = pd.read_csv(pi)
# df3 = pd.read_csv(pid)
# df4 = pd.read_csv(step)

# Rename the columns for clarityP_
df1.columns = ['Time', 'Cuff Pressure', 'Reference Pressure','OWM']
df2.columns = ['Time', 'Cuff Pressure', 'Reference Pressure','OWM']
# df3.columns = ['Time', 'Cuff Pressure', 'Reference Pressure','OWM']
# df4.columns = ['Time', 'Pulse Amplitude']

# Create subplots
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 10),gridspec_kw={'hspace': 0})

# Plot the first dataset
axs[0].plot(df1['Time'], df1['Cuff Pressure'],color='r', label='Cuff Pressure')
axs[0].plot(df1['Time'], df1['Reference Pressure'],color='b', label='Reference Pressure')
axs[0].legend(loc='upper right')
# axs[0].text(0.5, 0.95, 'P Controller', horizontalalignment='center', verticalalignment='center', transform=axs[0].transAxes)
axs[0].grid(False)

# # Plot the second dataset
# axs[1].plot(df2['Time'], df2['OWM'],color='r', label='Cuff Pressure')
# # axs[1].plot(df2['Time'], df2['OWM'],color='b', label='Reference Pressure')
# # axs[1].set_ylabel('Pulse Amplitude/mmHg')
# # axs[1].text(0.5, 0.95, 'PI Controller', horizontalalignment='center', verticalalignment='center', transform=axs[1].transAxes)
# axs[1].legend(loc='upper right')
# axs[1].grid(False)

# axs[2].plot(df3['Time'], df3['Cuff Pressure'],color='r', label='Cuff Pressure')
# axs[2].plot(df3['Time'], df3['Reference Pressure'],color='b', label='Reference Pressure')
# # axs[1].set_ylabel('Pulse Amplitude/mmHg')
# axs[2].text(0.5, 0.95, 'PID Controller', horizontalalignment='center', verticalalignment='center', transform=axs[2].transAxes)
# axs[2].legend(loc='upper right')
# axs[2].grid(False)

# # Plot the third dataset
# axs[2].plot(df3['Time'], df3['Pulse Amplitude'], color='y', label='Differential System')
# axs[2].legend(loc='upper right')
# axs[2].grid(False)

# Plot the forth dataset
# axs[3].plot(df4['Time'], df4['Pulse Amplitude'], color='g', label='Step Differential System')
# axs[3].legend(loc='upper right')
# axs[3].set_xlabel('Time')
# axs[3].grid(False)

# Improve layout
fig.text(0.01, 0.5, 'Pulse Amplitude/mmHg', va='center', rotation='vertical')
# fig.suptitle('OWMs on Simulator')
plt.tight_layout()
plt.show()