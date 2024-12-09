import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

def smooth_data(data, window_length=5, polyorder=2):
    """
    使用 Savitzky-Golay 滤波器平滑数据
    :param data: 要平滑的原始数据
    :param window_length: 滤波器窗口长度，必须是奇数
    :param polyorder: 多项式阶数
    :return: 平滑后的数据
    """
    smoothed_data = savgol_filter(data, window_length, polyorder, axis=1)
    return smoothed_data

def expand_data(data, num_points=620):
    """
    使用插值方法将数据扩展到指定数量的点
    :param data: 原始数据 
    :param num_points: 目标数据点数量
    :return: 扩展后的数据
    """
    original_points = np.linspace(0, 1, data.shape[1])
    target_points = np.linspace(0, 1, num_points)
    interpolated_data = np.zeros((data.shape[0], num_points))

    for i in range(data.shape[0]):
        interpolator = interp1d(original_points, data[i], kind='linear')
        interpolated_data[i] = interpolator(target_points)

    return interpolated_data

def plot_3d_curve_from_csv(file_path, output_file_path):
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

    # 平滑数据
    smoothed_data = smooth_data(data)
    # 扩展数据到620组
    expanded_data = expand_data(smoothed_data, num_points=620)
    # 将平滑后的数据写入新的CSV文件
    with open(output_file_path, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)  # 写入表头
        for row in smoothed_data.T:  # 转置回原始形状
            writer.writerow(row)

    # 创建三维图形
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制三维曲线
    ax.plot(smoothed_data[0], smoothed_data[1], smoothed_data[2], label='Smoothed 3D Curve')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # 显示图形
    plt.show()

# 示例调用
plot_3d_curve_from_csv('/home/wangjunjie/PyDMPs_Chauby/code/demo_trajectory/data/raw/ur3e_tip_positions.csv',
                       '/home/wangjunjie/PyDMPs_Chauby/code/demo_trajectory/data/smoothed/smoothed_ur3e_tip_positions_W.csv')