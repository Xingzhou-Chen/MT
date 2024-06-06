# Importing necessary libraries
import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
import pandas as pd

file0_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/data/pwm/pwm_b_new.csv"
# file_path = "pressure.csv"
df0 = pd.read_csv(file0_path)
x=df0['t']
y=df0['pwm']

# print(type(x.values))
x=x.to_numpy().reshape(len(x),1)
y=y.to_numpy().reshape(len(y),1)
# B = np.reshape(x, (-1, 2))
# x.as_matrix()
# x.reshape((2,1))
# print(x.shape)
# Generating some sample data
# np.random.seed(0)
# X = 2 * np.random.rand(100, 1)
# y = 4 + 3 * X + np.random.randn(100, 1)

# X.reshape((1,100))
# print(X.shape)
# Creating a linear regression model
model = LinearRegression()

# Fitting the model to the data
model.fit(x, y)

# Printing the coefficients
print("Intercept:", model.intercept_[0])
print("Coefficient:", model.coef_[0][0])

# Making predictions
X_new = np.array([[0], [40]])
y_pred = model.predict(X_new)
print("Predictions for new data:", y_pred)

# Plotting the data and the linear regression line
plt.scatter(x, y)
plt.plot(X_new, y_pred, "r-")
plt.xlabel("Time/s")
plt.ylabel("PWM")
plt.title("Linear Regression")
plt.show()