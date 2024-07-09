import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
file_path = '/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/PID/I_large3.csv'  # Replace with the actual file path
data = pd.read_csv(file_path)

# Extract the first and third columns
x = data.iloc[:, 0]
y = data.iloc[:, 3]

# Create a plot
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Cuff Pressure')
plt.xlabel('Time/s')
plt.ylabel('Pressure/mmHg')
# plt.title('Plot of Column 1 vs Column 3')
plt.legend()
plt.grid(False)
plt.show()