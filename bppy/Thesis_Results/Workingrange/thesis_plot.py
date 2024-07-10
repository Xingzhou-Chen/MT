import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
max = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Workingrange/pv_max.csv'
min = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Workingrange/pv_min.csv'
# ds = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Finger/OWM/ds/finger_owm.csv'

df1 = pd.read_csv(max)
df2 = pd.read_csv(min)

# Rename the columns for clarity
df1.columns = ['Time', 'Pulse Amplitude']
df2.columns = ['Time', 'Pulse Amplitude']

# Create subplots
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 10),gridspec_kw={'hspace': 0})

# Plot the first dataset
axs[0].plot(df1['Time'], df1['Pulse Amplitude'], color='r', label='Max')
# axs[0].plot(df1['Time'], df1['Time1'], color='b', label='Reference Pressure')
axs[0].legend(loc='upper right')
axs[0].grid(False)
# axs[0].set_ylim(-1.5, 2.45)
# axs[0].set_xlim(11.5,13.5)
# axs[0].set_ylim(-3, 3.5)
# axs[0].set_ylim(-4.5, 6.8)
axs[0].set_ylim(-4, 5.9)
axs[0].set_xlim(10,12)

# Plot the second dataset
axs[1].plot(df2['Time'], df2['Pulse Amplitude'], color='b', label='Min')
# axs[1].set_ylabel('Pulse Amplitude/mmHg')
axs[1].legend(loc='upper right')
axs[1].grid(False)
# axs[1].set_ylim(-1.5, 2.45)
# axs[1].set_ylim(-3, 3.5)
# axs[1].set_ylim(-4.5, 6.8)
axs[1].set_ylim(-4, 5.9)
axs[1].set_xlabel("Time/s")

# # Plot the third dataset
# axs[2].plot(df3['Time'], df3['Pulse Amplitude'], color='y', label='Differential System')
# axs[2].legend(loc='upper right')
# axs[2].grid(False)


# Improve layout
# fig.text(0.01, 0.5, 'Pulse Amplitude/mmHg', va='center', rotation='vertical')
# fig.suptitle('Pulse Volume')
plt.tight_layout()
plt.show()

# df1 = pd.read_csv('/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Workingrange/df_max_finger.csv')
# df2 = pd.read_csv('/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Workingrange/finger_owm_dfmax_1.csv')