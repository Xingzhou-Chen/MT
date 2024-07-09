import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
max = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/sysdia/15080.csv'
min = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/sysdia/13090.csv'
# ds = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/ARM/OWM/ds/arm_owm3.csv'

df1 = pd.read_csv(max)
df2 = pd.read_csv(min)
# df3 = pd.read_csv(ds)

# Rename the columns for clarity
# df1.columns = ['Time', 'Pulse Amplitude','a','b','a','owm']
# df2.columns = ['Time', 'Pulse Amplitude','a','b','a','owm']
# df3.columns = ['Time', 'Pulse Amplitude']
df1.columns = ['Time', 'owm']
df2.columns = ['Time', 'owm']

# Create subplots
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 10),gridspec_kw={'hspace': 0})

# Plot the first dataset
axs[0].plot(df1['Time'], df1['owm'], color='r', label='Dog')
axs[0].legend(loc='upper right')
axs[0].grid(False)
axs[0].set_xlim(10,15)
# axs[0].set_ylim(-1,2)
# axs[0].set_ylim(-1.5,3)
axs[0].set_ylim(-2.5,3.1)

# Plot the second dataset
axs[1].plot(df2['Time'], df2['owm'], color='b', label='Cat')
# axs[1].set_ylabel('Pulse Amplitude/mmHg')
axs[1].legend(loc='upper right')
axs[1].grid(False)
# axs[1].set_xlim(9,15)
# axs[1].set_ylim(-1,2)
# axs[1].set_ylim(-1.5,3)
axs[1].set_ylim(-2.5,3.1)

# Plot the third dataset
# axs[2].plot(df3['Time'], df3['Pulse Amplitude'], color='y', label='Differential System')
# axs[2].legend(loc='upper right')
# axs[2].grid(False)
# # axs[2].set_xlim(9,15)
# # axs[2].set_ylim(-1,2)
# axs[2].set_ylim(-1.5,3)

# Improve layout
fig.text(0.01, 0.5, 'Pulse Amplitude/mmHg', va='center', rotation='vertical')
# fig.suptitle('Noise of Pulse Waves on Arm')
plt.tight_layout()
plt.show()