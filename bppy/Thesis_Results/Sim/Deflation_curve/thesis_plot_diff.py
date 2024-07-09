import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
inc = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/Deflation_curve/inc/sim2.csv'
ff = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/Deflation_curve/ff/sim2.csv'
ds = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/Deflation_curve/ds/sim3.csv'
step = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Sim/Deflation_curve/step/step2.csv'

df1 = pd.read_csv(inc)
df2 = pd.read_csv(ff)
df3 = pd.read_csv(ds)
df4 = pd.read_csv(step)

# Rename the columns for clarity
df1.columns = ['Time', 'Cuff Pressure', 'Reference Pressure','OWM']
df2.columns = ['Time', 'Cuff Pressure', 'Reference Pressure','OWM']
df3.columns = ['Time', 'Cuff Pressure','Baseline Pressure','Baseline Reference Pressure','Cuff Reference Pressure']
df4.columns = ['Time', 'Cuff Pressure','Baseline Pressure','Difference Pressure','Oscillometric Waveform']

# Create subplots
fig, axs = plt.subplots(2, 2, sharex=True, figsize=(10, 10),gridspec_kw={'hspace': 0})

# # Plot the first dataset
axs[0][1].plot(df3['Time'], df3['Cuff Pressure'],color='r', label='Cuff Pressure')
axs[0][1].plot(df3['Time'], df3['Baseline Pressure'],color='b', label='Cuff Reference Pressure')
axs[0][1].plot(df3['Time'], df3['Baseline Reference Pressure'],color='y', label='Baseline Volume Pressure')
axs[0][1].plot(df3['Time'], df3['Cuff Reference Pressure'],color='g', label='Baseline Reference Pressure')
# axs[0].plot(df1['Time'], df1['OWM'],color='r', label='OWM')
axs[0][1].text(0.5, 0.95, 'Linear Differential Sensor System', horizontalalignment='center', verticalalignment='center', transform=axs[0][1].transAxes)
axs[0][1].legend(loc='upper right')
axs[0][1].grid(False)

# # # Plot the second dataset
axs[1][1].plot(df4['Time'], df4['Cuff Pressure'],color='r', label='Cuff Pressure')
axs[1][1].plot(df4['Time'], df4['Baseline Pressure'],color='b', label='Baseline Volume Pressure')
axs[1][1].plot(df4['Time'], df4['Difference Pressure'],color='m', label='Differential Pressure')
# axs[1].plot(df1['Time'], df1['OWM'],color='r', label='OWM')
axs[1][1].text(0.5, 0.95, 'Step-wise Differential Sensor System', horizontalalignment='center', verticalalignment='center', transform=axs[1][1].transAxes)
axs[1][1].legend(loc='upper right')
axs[1][1].grid(False)

# Plot the third dataset
axs[0][0].plot(df1['Time'], df1['Cuff Pressure'],color='r', label='Cuff Pressure')
axs[0][0].plot(df1['Time'], df1['Reference Pressure'],color='b', label='Reference Pressure')
# axs[0].plot(df1['Time'], df1['Baseline Reference Pressure'],color='b', label='Baseline Reference Pressure')
# axs[0].plot(df1['Time'], df1['Cuff Reference Pressure'],color='g', label='Cuff Reference Pressure')
# axs[0].plot(1f1['Time'], df1['OWM'],color='r', label='OWM')
axs[0][0].text(0.5, 0.95, 'Incremental PID', horizontalalignment='center', verticalalignment='center', transform=axs[0][0].transAxes)
axs[0][0].legend(loc='upper right')
axs[0][0].grid(False)

# Plot the forth dataset
axs[1][0].plot(df2['Time'], df2['Cuff Pressure'],color='r', label='Cuff Pressure')
axs[1][0].plot(df2['Time'], df2['Reference Pressure'],color='b', label='Reference Pressure')
# axs[1].plot(df2['Time'], df2['Baseline Reference Pressure'],color='b', label='Baseline Reference Pressure')
# axs[1].plot(df2['Time'], df2['Cuff Reference Pressure'],color='g', label='Cuff Reference Pressure')
# ax1[0].plot(df1['Time'], df1['OWM'],color='r', label='OWM')
axs[1][0].text(0.5, 0.95, 'Feedforward PID', horizontalalignment='center', verticalalignment='center', transform=axs[1][0].transAxes)
axs[1][0].legend(loc='upper right')
axs[1][0].grid(False)

# Improve layout
fig.text(0.01, 0.5, 'Pulse Amplitude/mmHg', va='center', rotation='vertical')
# fig.suptitle('Deflation Curves on Simulator')
plt.tight_layout()
plt.show()