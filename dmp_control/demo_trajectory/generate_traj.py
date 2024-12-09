import numpy as np
import csv

def generate_square_trajectory(total_points=620, z_height=0.46):
    """
    生成正方形轨迹坐标数组（包含 z 轴高度），四个角点分别为：
    (-0.33, 0.03), (-0.29, 0.03), (-0.29, -0.03), (-0.33, -0.03)
    
    参数:
        total_points (int): 总轨迹点数（默认为620）。
        z_height (float): 固定的 z 轴坐标值（默认为0.46）。
    
    返回:
        numpy.ndarray: 形状为 (620, 3) 的坐标数组。
    """
    # 定义正方形四个角点
    corners = [
        (-0.3, 0.3),  # 左上角
        (-0.2, 0.3),  # 右上角
        (-0.2, 0.4), # 右下角
        (-0.3, 0.4)  # 左下角
    ]
    
    # 每条边的点数
    points_per_edge = total_points // 4
    remainder = total_points % 4  # 分配剩余点数

    # 生成 4 条边上的点
    edge1_x = np.linspace(corners[0][0], corners[1][0], points_per_edge + (1 if remainder > 0 else 0))
    edge1_y = np.linspace(corners[0][1], corners[1][1], points_per_edge + (1 if remainder > 0 else 0))
    remainder -= 1

    edge2_x = np.linspace(corners[1][0], corners[2][0], points_per_edge + (1 if remainder > 0 else 0))
    edge2_y = np.linspace(corners[1][1], corners[2][1], points_per_edge + (1 if remainder > 0 else 0))
    remainder -= 1

    edge3_x = np.linspace(corners[2][0], corners[3][0], points_per_edge + (1 if remainder > 0 else 0))
    edge3_y = np.linspace(corners[2][1], corners[3][1], points_per_edge + (1 if remainder > 0 else 0))
    remainder -= 1

    edge4_x = np.linspace(corners[3][0], corners[0][0], points_per_edge + (1 if remainder > 0 else 0))
    edge4_y = np.linspace(corners[3][1], corners[0][1], points_per_edge + (1 if remainder > 0 else 0))

    # 拼接轨迹点
    x = np.concatenate((edge1_x, edge2_x, edge3_x, edge4_x))
    y = np.concatenate((edge1_y, edge2_y, edge3_y, edge4_y))

    # 如果有多余或不足点数，调整
    xy = np.column_stack((x, y))
    if len(xy) > total_points:
        xy = xy[:total_points]
    elif len(xy) < total_points:
        extra_points = total_points - len(xy)
        xy = np.vstack([xy, xy[:extra_points]])

    # 添加固定 z 轴
    z = np.full((total_points, 1), z_height)
    trajectory_xyz = np.hstack((z, xy))

    return trajectory_xyz

def save_to_csv(data, filename="trajectory.csv"):
    """
    将轨迹数组保存到 CSV 文件。
    
    参数:
        data (numpy.ndarray): 轨迹数组。
        filename (str): 保存的文件名。
    """
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z"])
        writer.writerows(data)

# 生成轨迹
trajectory = generate_square_trajectory(total_points=620, z_height=0.2)

# 保存到 CSV
save_to_csv(trajectory, filename="trajectory.csv")

print("轨迹保存到文件 trajectory.csv 成功！")
