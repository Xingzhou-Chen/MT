import pandas as pd
import matplotlib.pyplot as plt

# 读取用户上传的两个文件
df1 = pd.read_csv('/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Workingrange/df_max_finger.csv')
df2 = pd.read_csv('/Users/chenxingzhou/Desktop/MT/MT/bppy/Thesis_Results/Workingrange/finger_owm_dfmax_1.csv')

# 去除列名中的空格和不可见字符
df1.columns = df1.columns.str.strip()
df2.columns = df2.columns.str.strip()

# 提取第一个表格的第一、第二、第三列
df1_selected = df1.iloc[:, [0, 1, 2]]

# 第二个表格的第一、第二列
df2_selected = df2.iloc[:, [0, 1]]

# 合并数据集，根据共同的x轴长度
min_len = min(len(df1_selected), len(df2_selected))
df1_trimmed = df1_selected.iloc[:min_len]
df2_trimmed = df2_selected.iloc[:min_len]

# 合并数据集，将第一个表格的第一列作为x轴
df_combined = pd.concat([df1_trimmed.reset_index(drop=True), df2_trimmed.iloc[:, 1].reset_index(drop=True)], axis=1)
df_combined.columns = ['x_axis', 'Column1', 'Column2', 'Column3']

# 绘制图形
x = df_combined['x_axis']
y1 = df_combined['Column1']
y2 = df_combined['Column2']
y3 = df_combined['Column3']

# 创建图形和第一个 y 轴
fig, ax1 = plt.subplots()

ax1.set_xlabel('Time/s')
ax1.set_ylabel('Pressure/mmHg', color='tab:red')
ax1.plot(x, y1, label='Cuff Pressure', color='tab:red')
ax1.plot(x, y2, label='Reference Pressure', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:red')

# 创建第二个 y 轴
ax2 = ax1.twinx()
ax2.set_ylabel('Pressure/mmHg', color='tab:green')
ax2.plot(x, y3, label='OWM', color='tab:green')
ax2.tick_params(axis='y', labelcolor='tab:green')

# 添加图例
fig.tight_layout()
fig.legend(loc='upper right', bbox_to_anchor=(0.85,0.95))

# 显示图形
plt.show()