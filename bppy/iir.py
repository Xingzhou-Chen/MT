import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import iirfilter, lfilter, freqz
import csv

# Design parameters
fs = 31  # Sampling frequency in Hz
lowcut = 5  # Lower cutoff frequency in Hz
highcut = 15  # Upper cutoff frequency in Hz
order = 3  # Filter order
y=[]
x=[]
# Design the bandpass filter
b, a = iirfilter(order, [lowcut, highcut], btype='band', fs=fs)

with open('owm_mat.csv','r') as file:
        tests=csv.reader(file)
        for test in tests:
#         print(test)
            x.append(float(test[0]))
            y.append(float(test[1]))
# Generate a test signal (you can replace this with your own data)
# t = np.arange(0, 1, 1/fs)
# signal = np.sin(2 * np.pi * 100 * t) + 0.5 * np.sin(2 * np.pi * 300 * t)

# Apply the bandpass filter to the signal
filtered_signal = lfilter(b, a, y)

# Plot the original and filtered signals
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Original Signal')
plt.plot(x, filtered_signal, label='Filtered Signal')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.legend()
plt.show()
