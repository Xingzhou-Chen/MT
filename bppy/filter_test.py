import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import firwin, lfilter
import csv
import pandas as pd

# Define filter specifications
fs = 16 # Sampling frequency
lowcut =2 # Low cutoff frequency
highcut = 7.5 # High cutoff frequency
# cutoff_frequency = 10
num_taps = 101  # Number of filter taps
y=[]
x=[]
# Design the bandpass FIR filter
fir_coefficients = firwin(num_taps, [lowcut, highcut], fs=fs, pass_zero=False)
# fir_coefficients = firwin(num_taps, cutoff=cutoff_frequency, fs=fs, pass_zero=False)
# b, a = iirfilter(order, [lowcut, highcut], btype='band', fs=fs)

# # Plot the frequency response of the filter
# freq_response = np.fft.fft(fir_coefficients, 10000)
# frequencies = np.fft.fftfreq(10000, d=1/fs)
# 
# plt.figure(figsize=(10, 6))
# plt.plot(frequencies, 20 * np.log10(np.abs(freq_response)))
# plt.title('FIR Bandpass Filter Frequency Response')
# plt.xlabel('Frequency (Hz)')
# plt.ylabel('Gain (dB)')
# plt.grid(True)
# plt.show()

# Apply the filter to a signal (example signal)
# t = np.linspace(0, 1, fs, endpoint=False)  # 1 second signal
# input_signal = np.sin(2 * np.pi * 100 * t) + 0.5 * np.sin(2 * np.pi * 300 * t)
with open('owm_mat.csv','r') as file:
        tests=csv.reader(file)
        for test in tests:
            x.append(float(test[0]))
            y.append(float(test[1]))
            
# Apply the FIR filter to the signal
filtered = lfilter(fir_coefficients, 1.0, y)
filtered_file_path = "filtered.csv"
datafram=pd.DataFrame({'a':x,'b':filtered})
datafram.to_csv(filtered_file_path,index=False,sep=',')
df_filtered = pd.read_csv(filtered_file_path)
df_filtered.plot(x=df_filtered.index[0],title="Filtered_Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)

# file_path = "owm_mat.csv"
# df_before = pd.read_csv(file_path)
# df_before.plot(x=df_before.index[0],title="Pressure_Curve",xlabel="Time/s",ylabel="Pressure/mbar",legend=False)

plt.show()
fft_result = np.fft.fft(filtered)
frequencies = np.fft.fftfreq(len(fft_result), 1/fs)
plt.plot(frequencies, np.abs(fft_result))
plt.title('Frequency Domain Representation')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.show()

