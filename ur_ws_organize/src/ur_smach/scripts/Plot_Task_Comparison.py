#!/home/dongyi/anaconda3/envs/YOLOnew/bin/python

# conda activate YOLOnew
# export PYTHONPATH=/home/dongyi/anaconda3/envs/YOLOnew/lib/python3.12/site-packages

# 两段独立绘图脚本共用的 import / 字体配置
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd

# 统一字体配置（全局）
FONT_SCALE = 1.4
BASE_FONTSIZE = 18
TITLE_FONTSIZE = 20
AXIS_LABEL_FONTSIZE = 24
TICK_LABEL_FONTSIZE = 20
VALUE_LABEL_FONTSIZE = 16

def apply_plot_style() -> None:
    """为当前脚本设置统一的 seaborn/matplotlib 字体与风格。"""
    sns.set_style("whitegrid")
    sns.set_context("talk", font_scale=FONT_SCALE)
    plt.rcParams.update(
        {
            "font.size": BASE_FONTSIZE,
            "axes.labelsize": AXIS_LABEL_FONTSIZE,
            "xtick.labelsize": TICK_LABEL_FONTSIZE,
            "ytick.labelsize": TICK_LABEL_FONTSIZE,
            "legend.fontsize": BASE_FONTSIZE,
            "axes.titlesize": TITLE_FONTSIZE,
        }
    )



# # with ruler and without ruler

# # 准备数据
# data = {
#     'Case': ['w ruler', 'w/o ruler'],
#     'Success rate': [0.7778, 1.0000]
# }
# df = pd.DataFrame(data)

# # 绘图 - 添加width参数缩小条形宽度
# apply_plot_style()
# plt.figure(figsize=(6, 6))
# ax = sns.barplot(x='Case', y='Success rate', data=df, 
#                  palette=['#1f77b4', '#ff7f0e'],
#                  width=0.5)  # 添加width参数，默认0.8，这里设为0.5

# # 自动添加数值标签
# for p in ax.patches:
#     ax.annotate(f'{p.get_height():.4f}', 
#                 (p.get_x() + p.get_width() / 2., p.get_height()), 
#                 ha='center', va='bottom', fontsize=VALUE_LABEL_FONTSIZE, color='black',
#                 fontweight='bold')

# # ax.set_title('Success Rate Comparison', fontsize=TITLE_FONTSIZE, pad=15)
# ax.set_ylabel('Success Rate', fontsize=AXIS_LABEL_FONTSIZE)
# ax.set_xlabel('Case', fontsize=AXIS_LABEL_FONTSIZE)
# plt.xticks(fontsize=TICK_LABEL_FONTSIZE)
# plt.tight_layout()
# plt.show()



# # straight ruler and triangle ruler

# 准备数据
data = {
    'Case': ['straight ruler', 'triangle ruler'],
    'Success rate': [0.925, 0.825]
}
df = pd.DataFrame(data)

# 绘图 - 设置 width 参数缩小条形宽度
apply_plot_style()
plt.figure(figsize=(6, 6))
ax = sns.barplot(x='Case', y='Success rate', data=df, 
                 palette=['#2ca02c', '#ff7f0e'],
                 width=0.5)  # 添加这行，默认是0.8，这里设为0.5让条形更窄

# 自动添加数值标签
for p in ax.patches:
    ax.annotate(f'{p.get_height():.4f}', 
                (p.get_x() + p.get_width() / 2., p.get_height()), 
                ha='center', va='bottom', fontsize=VALUE_LABEL_FONTSIZE, color='black',
                fontweight='bold')

# ax.set_title('Success Rate: Straight vs Triangle Ruler', fontsize=TITLE_FONTSIZE, pad=15)
ax.set_ylabel('Success Rate', fontsize=AXIS_LABEL_FONTSIZE)
ax.set_xlabel('Ruler Type', fontsize=AXIS_LABEL_FONTSIZE)
plt.xticks(fontsize=TICK_LABEL_FONTSIZE)
plt.tight_layout()
plt.show()