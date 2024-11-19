#！/home/wangjunjie/.conda/envs/leggym/bin/python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

import torch
'''
    obs = ["cubeA_quat", "cubeA_pos", "cubeA_pos_relative", "eef_pos", "eef_quat"]
    都是基于世界坐标系的
    
'''



