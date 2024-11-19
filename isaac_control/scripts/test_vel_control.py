#!/home/wangjunjie/.conda/envs/leggym/bin/python
import rospy
from std_msgs.msg import Float64MultiArray
import tf2_ros
from geometry_msgs.msg import TransformStamped

import sys

# 添加 PyTorch 的安装路径
sys.path.append('/home/wangjunjie/.conda/envs/leggym/lib/python3.8/site-packages')

import torch
'''
    obs = ["cubeA_quat", "cubeA_pos", "cubeA_pos_relative", "eef_pos", "eef_quat"]
    都是基于世界坐标系的
    
'''
def get_eef_state():
    
    # 创建 TF 缓存对象
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    try:
        # 获取关节相对于参考坐标系的变换
        transform: TransformStamped = tf_buffer.lookup_transform(
            "base_link",  # 基准坐标系
            "flange",       # 目标关节的坐标系
            rospy.Time(0),    # 最近的可用时间点
            rospy.Duration(2.0)  # 等待超时时间
        )

        # 提取平移和旋转信息
        position = transform.transform.translation
        orientation = transform.transform.rotation
        position_torch = torch.tensor([position.x, position.y, position.z])
        orientation_torch = torch.tensor([orientation.x, orientation.y, orientation.z, orientation.w])
        
        return position_torch, orientation_torch
    
    except tf2_ros.LookupException as e:
        rospy.logerr(f"TF Lookup Error: {e}")
    except tf2_ros.ConnectivityException as e:
        rospy.logerr(f"TF Connectivity Error: {e}")
    except tf2_ros.ExtrapolationException as e:
        rospy.logerr(f"TF Extrapolation Error: {e}")
    return None, None

def test_vel_control():
    rospy.init_node('test_vel_control', anonymous=True)
    pub = rospy.Publisher('/joint_group_velocity_controller/command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(20) # 10hz

    # UR3e 机械臂的base_link高度,假设ur3e的base_link在table的表面
    table_height = 1.0
    table_thickness = 0.05
    table_stand_height = 0.1
    table_surface_height = table_height + table_thickness / 2
    ur3e_height = table_height + table_thickness / 2 + table_stand_height

    # obs-CubeA的位置和姿态
    cube_size = 0.05
    cubeA_pos = torch.tensor([0.45, 0, table_surface_height + cube_size/2])
    cubeA_quat = torch.tensor([0, 0, 0, 1])    

    target_vel = Float64MultiArray()

    model_path = '/home/wangjunjie/catkin_ws/src/isaac_control/models/policy_1.pt'
    model = torch.jit.load(model_path)
    while not rospy.is_shutdown():
        # update obs
        eef_pose, eef_quat = get_eef_state()
        eef_pose[2] += ur3e_height

        cubeA_pos_relative = cubeA_pos - eef_pose
        obs_buf = torch.cat([cubeA_quat, cubeA_pos, cubeA_pos_relative, eef_pose, eef_quat], dim = -1)

        actions = model.forward(obs_buf)
        target_vel.data = actions
        pub.publish(target_vel)
        reletive_dis = torch.norm(cubeA_pos_relative)
        print("reletive_distance: ", reletive_dis)
        rate.sleep()

if __name__ == '__main__':
    try:
        test_vel_control()
    except rospy.ROSInterruptException:
        pass