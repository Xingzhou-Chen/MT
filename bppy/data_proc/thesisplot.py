import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
file_path1 = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Experiments/incremental/arm_owm.csv'
file_path2 = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Experiments/feedforward/arm_owm2.csv'
file_path3 = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Experiments/feedforward/arm_owm.csv'
file_path4 = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Experiments/feedforward/arm_owm1.csv'

df1 = pd.read_csv(file_path1)
df2 = pd.read_csv(file_path2)
df3 = pd.read_csv(file_path3)
df4 = pd.read_csv(file_path4)

# Rename the columns for clarity
df1.columns = ['Time', 'Pulse Amplitude']
df2.columns = ['Time', 'Pulse Amplitude']
df3.columns = ['Time', 'Pulse Amplitude']
df4.columns = ['Time', 'Pulse Amplitude']

# Create subplots
fig, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 8),gridspec_kw={'hspace': 0})

# Plot the first dataset
axs[0].plot(df1['Time'], df1['Pulse Amplitude'], color='r', label='Incremental Form PID')
axs[0].legend(loc='upper left')
axs[0].grid(False)

# Plot the second dataset
axs[1].plot(df2['Time'], df2['Pulse Amplitude'], color='b', label='Feedforward PID')
# axs[1].set_ylabel('Pulse Amplitude/mmHg')
axs[1].legend(loc='upper left')
axs[1].grid(False)

# Plot the third dataset
axs[2].plot(df3['Time'], df3['Pulse Amplitude'], color='y', label='Differential System')
axs[2].legend(loc='upper left')
axs[2].grid(False)

# Plot the forth dataset
axs[3].plot(df4['Time'], df4['Pulse Amplitude'], color='g', label='Step Differential System')
axs[3].legend(loc='upper left')
axs[3].set_xlabel('Time')
axs[3].grid(False)

# Improve layout
fig.suptitle('OWM obtained on Simulator', fontsize=10)
fig.text(0, 0.5, 'Pulse Amplitude/mmHg', va='center', rotation='vertical')
# plt.tight_layout(rect=[0.03, 0.03, 1, 0.95])
plt.tight_layout()
plt.show()