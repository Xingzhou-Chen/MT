import array

def highpass_filter(input_signal, alpha):
    output_signal = array.array('f', [0.0] * len(input_signal))

    for n in range(1, len(input_signal)):
        output_signal[n] = alpha * (output_signal[n - 1] + input_signal[n] - input_signal[n - 1])

    return output_signal

# Example usage
sampling_rate = 1000  # Hz
cutoff_frequency = 50  # Hz
alpha = 0.1  # Filter coefficient

# Generate a test signal (replace with your own data)
time_points = array.array('f', [i / sampling_rate for i in range(1000)])
input_signal = array.array('f', [0.5 * np.sin(2 * np.pi * 50 * t) for t in time_points])

# Apply the high-pass filter
output_signal = highpass_filter(input_signal, alpha)

# Print or further process the filtered signal
print(output_signal)
