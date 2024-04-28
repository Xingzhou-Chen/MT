import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import firwin, lfilter
import csv

# Design parameters
fs = 60  # Sampling frequency in Hz
lowcut = 10  # Lower cutoff frequency in Hz
highcut = 15  # Upper cutoff frequency in Hz
num_taps = 100  # Filter order
y=[]
x=[]
# Design the bandpass filter
fir_coefficients = firwin(num_taps, [lowcut, highcut], fs=fs, pass_zero=False)
print(type(fir_coefficients.tolist()))
list_weights=fir_coefficients.tolist()
print(list_weights)
# for i in range(len(list_weights))
#     print(list_weights[i])
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
filtered_signal = lfilter(fir_coefficients, 1.0, y)

# Plot the original and filtered signals
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Original Signal')
plt.plot(x, filtered_signal, label='Filtered Signal')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.legend()
plt.show()
