import csv
import numpy as np
import matplotlib.pyplot as plt


def plot_3d_curve_from_csv(file_path):
    # 读取CSV文件
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        header = next(reader)  # 跳过表头
        for row in reader:
            try:
                x = float(row[0])
                y = float(row[1])
                z = float(row[2])
                data.append([x, y, z])
            except ValueError:
                # 跳过无法转换为浮点数的行
                continue

    # 转换为NumPy数组
    data = np.array(data).T  # 转置数组，使其适合绘图

    # 创建三维图形
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制三维曲线
    ax.plot(data[0], data[1], data[2], label='Smoothed 3D Curve')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # 显示图形
    plt.show()

# 示例调用
plot_3d_curve_from_csv('/home/wangjunjie/PyDMPs_Chauby/code/demo_trajectory/data/raw/ur3e_tip_positions.csv')