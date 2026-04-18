#!/home/dongyi/anaconda3/envs/YOLOnew/bin/python

# conda activate YOLOnew
# export PYTHONPATH=/home/dongyi/anaconda3/envs/YOLOnew/lib/python3.12/site-packages

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd

# 统一字体配置（全局）
FONT_SCALE = 1.4  # seaborn 整体字号缩放
BASE_FONTSIZE = 18
AXIS_LABEL_FONTSIZE = 24
TICK_LABEL_FONTSIZE = 20
VALUE_LABEL_FONTSIZE = 16

sns.set_style("whitegrid")
sns.set_context("talk", font_scale=FONT_SCALE)
plt.rcParams.update(
    {
        "font.size": BASE_FONTSIZE,
        "axes.labelsize": AXIS_LABEL_FONTSIZE,
        "xtick.labelsize": TICK_LABEL_FONTSIZE,
        "ytick.labelsize": TICK_LABEL_FONTSIZE,
        "legend.fontsize": BASE_FONTSIZE,
        "axes.titlesize": AXIS_LABEL_FONTSIZE,
    }
)

# 创建数据框
data = {
    'Case': ['C21', 'C22', 'C31', 'C32', 'C33', 'C34', 'C35', 'C41', 'C42', 'C43', 'C44', 'C51'],
    'Success_times': [15, 15, 9, 15, 15, 15, 15, 7, 14, 13, 15, 12]
}

df = pd.DataFrame(data)

# 设置图形大小和样式
plt.figure(figsize=(12, 6))

# 创建条形图
bar_plot = sns.barplot(data=df, x='Case', y='Success_times', palette='viridis') # viridis Blues_d

# 添加标题和标签
# plt.title('Success Times by Case', fontsize=16, fontweight='bold')
plt.xlabel('Case', fontsize=AXIS_LABEL_FONTSIZE)
plt.ylabel('Success Times', fontsize=AXIS_LABEL_FONTSIZE)

# 在每个条形上添加数值标签
for i, value in enumerate(df['Success_times']):
    bar_plot.text(i, value + 0.1, str(value), 
                  ha='center', va='bottom', fontsize=VALUE_LABEL_FONTSIZE, fontweight='bold')

# 调整y轴范围
plt.ylim(0, max(df['Success_times']) * 1.1)

# 旋转x轴标签以便更好显示
# plt.xticks(rotation=45)

# 调整布局
plt.tight_layout()

# 显示图形
plt.show()


