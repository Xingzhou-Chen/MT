import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 

file0_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/pulse_D.csv"
# file_path = "pressure.csv"
df0 = pd.read_csv(file0_path)
x=df0['t']
y=df0['pwm']

# print(type(x.values))
x=x.to_numpy().reshape(len(x),1)
y=y.to_numpy().reshape(len(y),1)

# Perform polynomial regression
degree = 3  # Degree of the polynomial
X = np.hstack([x**i for i in range(degree + 1)])
coefficients = np.linalg.inv(X.T.dot(X)).dot(X.T).dot(y)

# Print coefficients
print("Coefficients:")
for i, coef in enumerate(coefficients):
    print("Coefficient for x^{}: {:.2f}".format(i, coef[0]))

# Predict using the model
x_new = np.linspace(0, 40, 100).reshape(-1, 1)
X_new = np.hstack([x_new**i for i in range(degree + 1)])
y_predict = X_new.dot(coefficients)

# Visualize the results
plt.scatter(x, y, label='Data')
plt.plot(x_new, y_predict, 'r-', label='Polynomial Regression')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Polynomial Regression Example')
plt.legend()
plt.show()
