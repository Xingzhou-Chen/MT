import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# 生成一些示例数据

file0_path = "/Users/chenxingzhou/Desktop/MT/MT/bppy/differential_board/pwm_sim.csv"
# file_path = "pressure.csv"
df0 = pd.read_csv(file0_path)
df0.columns = ['t', 'pwm']
# df0.columns = ['Time', 'a','b','b','d','e']
x=df0['t']
y=df0['pwm']

# print(type(x.values))
x=x.to_numpy()
y=y.to_numpy()

# 进行多项式拟合
degree = 1 # 多项式的次数
coefficients = np.polyfit(x, y, degree)

# 生成拟合的多项式函数
p = np.poly1d(coefficients)

# 计算拟合后的函数值
y_fit = p(x)

# 绘制原始数据和拟合曲线
plt.scatter(x, y, label='PWM Value')
plt.plot(x, y_fit, 'r-', label='Polynomial Fit')
plt.xlabel('Time/s')
plt.ylabel('PWM Value')
# plt.title('Polynomial Curve Fitting Example')
plt.legend()
plt.show()

# 打印拟合的多项式函数
print("Polynomial Function:")
print(p)
